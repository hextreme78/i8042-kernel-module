/*
 * Copyright 2021 Danish Makbari
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 2 of the License, or
 *     any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/input.h>

#include <asm/io.h>
#include <asm/bitops.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Danish Makbari Alexandrovich");
MODULE_DESCRIPTION("Driver for PS/2 devices");

/* i8042 can trigger irq line 1 and irq line 12 */
#define I8042_IRQ1 1
#define I8042_IRQ12 12

/* Default io ports for i8042 */
#define I8042_DATA_REG 0x60
#define I8042_STATUS_REG 0x64
#define I8042_COMMAND_REG 0x64

/* Commmands for i8042 controller */
#define I8042_READ_CONFIG_BYTE 0x20
#define I8042_WRITE_CONFIG_BYTE 0x60
#define I8042_READ_OUTPUT_PORT 0xD0
#define I8042_WRITE_OUTPUT_PORT 0xD1
#define I8042_WRITE_FIRST_PS2_OUTPUT_BUFFER 0xD2
#define I8042_WRITE_SECOND_PS2_OUTPUT_BUFFER 0xD3
#define I8042_WRITE_SECOND_PS2_INPUT_BUFFER 0xD4
#define I8042_DISABLE_FIRST_PS2_PORT 0xAD
#define I8042_DISABLE_SECOND_PS2_PORT 0xA7
#define I8042_ENABLE_FIRST_PS2_PORT 0xAE
#define I8042_ENABLE_SECOND_PS2_PORT 0xA8
#define I8042_SELF_TEST 0xAA
#define I8042_FIRST_PORT_INTERFACE_TEST 0xAB
#define I8042_SECOND_PORT_INTERFACE_TEST 0xA9

/* Host-to-keyboard communication */
#define I8042_RESET 0xFF
#define I8042_DISABLE_SCANNING 0xF5
#define I8042_IDENTIFY 0xF2
#define I8042_CAPSLOCK 0xED
#define I8042_KBD_ENABLE 0xF4
#define I8042_KBD_DISABLE 0xF5

/* Keyboard-to-host communication */
#define I8042_ACK 0xFA
#define I8042_SELF_TEST_PASSED 0xAA
#define I8042_ECHO_RESPONSE 0xEE
#define I8042_RESEND_REQUEST 0xFE
#define I8042_ERROR1 0x00
#define I8042_ERROR2 0xFF

/* Devices */
#define UNDEFINED 0
#define KEYBOARD 1
#define MOUSE 2

static int first_port = 0, second_port = 0;
static struct input_dev *dev1, *dev2;

static uint8_t press_scancodes[] = {
				      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
				0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
				0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
				0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
				0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
				0x50, 0x51, 0x52, 0x53,                   0x57, 0x58
																};
static uint8_t release_scancodes[] = {
				      0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
				0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
				0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
				0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
				0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
				0xD0, 0xD1, 0xD2, 0xD3,                   0xD7, 0xD8
																};
static uint8_t keys[] = {
	 KEY_ESC, KEY_1,   KEY_2,     KEY_3,    KEY_4,       KEY_5,          KEY_6,          KEY_7,          KEY_8,     KEY_9,         KEY_0,          KEY_MINUS, KEY_EQUAL,    KEY_BACKSPACE,  KEY_TAB,
KEY_Q,   KEY_W,   KEY_E,   KEY_R,     KEY_T,    KEY_Y,       KEY_U,          KEY_I,          KEY_O,          KEY_P,     KEY_LEFTBRACE, KEY_RIGHTBRACE, KEY_ENTER, KEY_LEFTCTRL, KEY_A,          KEY_S,
KEY_D,   KEY_F,   KEY_G,   KEY_H,     KEY_J,    KEY_K,       KEY_L,          KEY_SEMICOLON,  KEY_APOSTROPHE, KEY_GRAVE, KEY_LEFTSHIFT, KEY_BACKSLASH,  KEY_Z,     KEY_X,        KEY_C,          KEY_V,
KEY_B,   KEY_N,   KEY_M,   KEY_COMMA, KEY_DOT,  KEY_SLASH,   KEY_RIGHTSHIFT, KEY_KPASTERISK, KEY_LEFTALT,    KEY_SPACE, KEY_CAPSLOCK,  KEY_F1,         KEY_F2,    KEY_F3,       KEY_F4,         KEY_F5,
KEY_F6 , KEY_F7,  KEY_F8,  KEY_F9,    KEY_F10,  KEY_NUMLOCK, KEY_SCROLLLOCK, KEY_KP7,        KEY_KP8,        KEY_KP9,   KEY_KPMINUS,   KEY_KP4,        KEY_KP5,   KEY_KP6,      KEY_KPPLUS,     KEY_KP1,
KEY_KP2, KEY_KP3, KEY_KP0, KEY_KPDOT,                        KEY_F11,        KEY_F12
};

static uint8_t esc_press_scancodes[] =		{0x1C, 0x1D, 0x2A, 0x36, 0x38, 0x47, 0x48, 0x49, 0x4B, 0x4D, 0x4F, 0x50, 0x51, 0x52, 0x53};
static uint8_t esc_release_scancodes[] =	{0x9C, 0x9D, 0xAA, 0xB6, 0xB8, 0xC7, 0xC8, 0xC9, 0xCB, 0xCD, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3};
static uint8_t esc_keys[] = {	KEY_KPENTER, KEY_RIGHTCTRL, KEY_LEFTSHIFT, KEY_RIGHTSHIFT, KEY_RIGHTALT, KEY_HOME, KEY_UP, KEY_PAGEUP, KEY_LEFT, KEY_RIGHT, KEY_END, KEY_DOWN,
			KEY_PAGEDOWN, KEY_INSERT, KEY_DELETE	};

static irqreturn_t i8042_handler(int irq, void *dev_data)
{
	int i;
	uint8_t scancode;
	struct input_dev *dev = (struct input_dev *) dev_data;
	scancode = inb(I8042_DATA_REG);
	if (irq == I8042_IRQ1) {
		/* Keyboard driver */
		if (scancode != 0xE0) {
			for (i = 0; i < 85; i++) {
				if (scancode == press_scancodes[i]) {
					input_report_key(dev, keys[i], 1);
					break;
				} else if (scancode == release_scancodes[i]) {
					input_report_key(dev, keys[i], 0);
					break;
				}
			}
		} else {
			scancode = inb(I8042_DATA_REG);
			for (i = 0; i < 15; i++) {
				if (scancode == esc_press_scancodes[i]) {
					input_report_key(dev, esc_keys[i], 1);
					break;
				} else if (scancode == esc_release_scancodes[i]) {
					input_report_key(dev, esc_keys[i], 0);
					break;
				}
			}
		}
	} else if (irq == I8042_IRQ12) {
		/* Mouse driver */


	}

	input_sync(dev);
	return IRQ_HANDLED;
}

/* Reads value from data register */
static int read_reg(uint8_t *byte, unsigned long wait_time)
{
	unsigned long j0, j1, delay;
	delay = msecs_to_jiffies(wait_time);
	j0 = jiffies;
	j1 = j0 + delay;
	while (time_before(jiffies, j1)) {
		uint8_t status = inb(I8042_STATUS_REG);
		if (test_bit(0, (void *) &status)) {
			*byte = inb(I8042_DATA_REG);
			return 0;
		}
	}
	return -1;
}

/* Wirtes to the device on first port */
static int write_dev1(uint8_t byte, unsigned long wait_time)
{
	unsigned long j0, j1, delay;
	delay = msecs_to_jiffies(wait_time);
	j0 = jiffies;
	j1 = j0 + delay;
	while (time_before(jiffies, j1)) {
		uint8_t status = inb(I8042_STATUS_REG);
		if (!test_bit(1, (void *) &status)) {
			outb(byte, I8042_DATA_REG);
			return 0;
		}
	}
	return -1;
}

/* Wirtes to the device on second port */
static int write_dev2(uint8_t byte, unsigned long wait_time)
{
	unsigned long j0, j1, delay;
	outb(I8042_WRITE_SECOND_PS2_INPUT_BUFFER, I8042_COMMAND_REG);
	delay = msecs_to_jiffies(wait_time);
	j0 = jiffies;
	j1 = j0 + delay;
	while (time_before(jiffies, j1)) {
		uint8_t status = inb(I8042_STATUS_REG);
		if (!test_bit(1, (void *) &status)) {
			outb(byte, I8042_DATA_REG);
			return 0;
		}
	}
	return -1;
}

int init_module(void)
{
	int error;
	uint8_t byte, dual_channel_test;

	/* This code disables PS/2 ports */
	outb(I8042_DISABLE_FIRST_PS2_PORT, I8042_COMMAND_REG);
	outb(I8042_DISABLE_SECOND_PS2_PORT, I8042_COMMAND_REG);

	/* This code flushes output buffer */
	outb(I8042_READ_CONFIG_BYTE, I8042_COMMAND_REG);
	if (read_reg(&byte, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded\n");
		return -ETIME;
	}

	/* This code sets config byte */
	outb(I8042_READ_CONFIG_BYTE, I8042_COMMAND_REG);
	if (read_reg(&byte, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded\n");
		return -ETIME;
	}
	__clear_bit(0, (void *) &byte);
	__clear_bit(1, (void *) &byte);
	__clear_bit(6, (void *) &byte);
	outb(byte, I8042_DATA_REG);
	if (read_reg(&byte, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded\n");
		return -ETIME;
	}
	outb(I8042_WRITE_CONFIG_BYTE, I8042_COMMAND_REG);
	dual_channel_test = test_bit(5, (void *) &byte) ? 1 : 0;

	/* This code performs i8042 self check */
	outb(I8042_SELF_TEST, I8042_COMMAND_REG);
	if (read_reg(&byte, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded\n");
		return -ETIME;
	}
	if (byte == 0x55) {
		printk(KERN_INFO "i8042: self test was successful\n");
	} else {
		printk(KERN_ERR "i8042: self test failed\n");
		return -EINVAL;
	}

	/* This code determines if there are 2 channels */
	if (dual_channel_test) {
		outb(I8042_ENABLE_SECOND_PS2_PORT, I8042_COMMAND_REG);
		outb(I8042_READ_CONFIG_BYTE, I8042_COMMAND_REG);
		if (read_reg(&byte, 250) < 0) {
			printk(KERN_ERR "i8042: time limit exceeded\n");
			return -ETIME;
		}
		if (test_bit(5, (void *) &byte)) {
			dual_channel_test = 0;
			printk(KERN_INFO "i8042: dualchannel is not supporting\n");
		} else {
			dual_channel_test = 1;
			printk(KERN_INFO "i8042: dualchannel is supporting\n");
		}
		outb(I8042_DISABLE_SECOND_PS2_PORT, I8042_COMMAND_REG);
	}

	/* This code performs interface check */
	outb(I8042_FIRST_PORT_INTERFACE_TEST, I8042_COMMAND_REG);
	if (read_reg(&byte, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded\n");
		return -ETIME;
	}
	if (byte == 0x00) {
		first_port = 1;
		printk(KERN_INFO "i8042: test of first port was successful\n");
	} else {
		printk(KERN_ERR "i8042: test of first port failed\n");
	}
	if (dual_channel_test) {
		outb(I8042_SECOND_PORT_INTERFACE_TEST, I8042_COMMAND_REG);
		if (read_reg(&byte, 250) < 0) {
			printk(KERN_ERR "i8042: time limit exceeded\n");
			return -ETIME;
		}
		if (byte == 0x00) {
			second_port = 1;
			printk(KERN_INFO "i8042: test of second port was successful\n");
		} else {
			printk(KERN_ERR "i8042: test of second port failed\n");
		}
	}
	if (!first_port && !second_port) {
		return -EINVAL;
	}

	/* This code enables ports */
	outb(I8042_READ_CONFIG_BYTE, I8042_COMMAND_REG);
	if (read_reg(&byte, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded\n");
		return -ETIME;
	}
	if (first_port) {
		outb(I8042_ENABLE_FIRST_PS2_PORT, I8042_COMMAND_REG);
		__set_bit(0, (void *) &byte);
	}
	if (second_port) {
		outb(I8042_ENABLE_SECOND_PS2_PORT, I8042_COMMAND_REG);
		__set_bit(1, (void *) &byte);
	}
	__set_bit(6, (void *) &byte);
	outb(byte, I8042_DATA_REG);
	outb(I8042_WRITE_CONFIG_BYTE, I8042_COMMAND_REG);

	/* This code resets devices */
	if (write_dev1(I8042_RESET, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded\n");
		return -ETIME;
	}
	if (read_reg(&byte, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded 1\n");
		return -ETIME;
	}
	if (write_dev2(I8042_RESET, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded\n");
		return -ETIME;
	}
	if (read_reg(&byte, 250) < 0) {
		printk(KERN_ERR "i8042: time limit exceeded\n");
		return -ETIME;
	}

	/* Detecting device on first port */
	if (first_port) {
		if (write_dev1(I8042_DISABLE_SCANNING, 250) < 0) {
			printk(KERN_INFO "i8042: can't detect device on first port\n");
			first_port = UNDEFINED;
			goto first_port_fail;
		}
		if (read_reg(&byte, 250) < 0) {
			printk(KERN_INFO "i8042: can't detect device on first port\n");
			first_port = UNDEFINED;
			goto first_port_fail;
		}
		if (write_dev1(I8042_IDENTIFY, 250) < 0) {
			printk(KERN_INFO "i8042: can't detect device on first port\n");
			first_port = UNDEFINED;
			goto first_port_fail;
		}
		if (read_reg(&byte, 250) < 0) {
			printk(KERN_INFO "i8042: can't detect device on first port\n");
			first_port = UNDEFINED;
			goto first_port_fail;
		}
		if (byte == 0xFA) {
			if (read_reg(&byte, 250) < 0) {
				printk(KERN_INFO "i8042: can't detect device on first port\n");
				first_port = UNDEFINED;
				goto first_port_fail;
			}
			if (byte == 0x00) {
				printk(KERN_INFO "i8042: standard mouse on first port\n");
				first_port = MOUSE;
			} else if (byte == 0x03) {
				printk(KERN_INFO "i8042: mouse with wheel on first port\n");
				first_port = MOUSE;
			} else if (byte == 0x04) {
				printk(KERN_INFO "i8042: 5 button mouse on first port\n");
				first_port = MOUSE;
			} else if (byte == 0xAB) {
				uint8_t byte2;
				if (read_reg(&byte2, 250) < 0) {
					printk(KERN_INFO "i8042: can't detect device on first port\n");
					first_port = UNDEFINED;
					goto first_port_fail;
				}
				if (byte == 0xAB && (byte2 == 0x41 || byte2 == 0xC1)) {
					printk(KERN_INFO "i8042: MF2 keyboard with translation on first port\n");
					first_port = KEYBOARD;
				} else if (byte == 0xAB && byte2 == 0x83) {
					printk(KERN_INFO "i8042: MF2 keyboard on first port\n");
					first_port = KEYBOARD;
				}
				else {
					printk(KERN_INFO "i8042: can't detect device on first port\n");
					first_port = UNDEFINED;
				}
			} else {
				printk(KERN_INFO "i8042: can't detect device on first port\n");
				first_port = UNDEFINED;
			}
		} else {
			printk(KERN_INFO "i8042: can't detect device on first port\n");
			first_port = UNDEFINED;
		}
	}
first_port_fail:

	/* Detecting device on second port */
	if (second_port) {
		if (write_dev2(I8042_DISABLE_SCANNING, 250) < 0) {
			printk(KERN_INFO "i8042: can't detect device on second port\n");
			second_port = UNDEFINED;
			goto second_port_fail;
		}
		if (read_reg(&byte, 250) < 0) {
			printk(KERN_INFO "i8042: can't detect device on second port\n");
			second_port = UNDEFINED;
			goto second_port_fail;
		}
		if (write_dev2(I8042_IDENTIFY, 250) < 0) {
			printk(KERN_INFO "i8042: can't detect device on second port\n");
			second_port = UNDEFINED;
			goto second_port_fail;
		}
		if (read_reg(&byte, 250) < 0) {
			printk(KERN_INFO "i8042: can't detect device on second port\n");
			second_port = UNDEFINED;
			goto second_port_fail;
		}
		if (byte == 0xFA) {
			if (read_reg(&byte, 250) < 0) {
				printk(KERN_INFO "i8042: can't detect device on second port\n");
				second_port = UNDEFINED;
				goto second_port_fail;
			}
			if (byte == 0x00) {
				printk(KERN_INFO "i8042: standard mouse on second port\n");
				second_port = MOUSE;
			} else if (byte == 0x03) {
				printk(KERN_INFO "i8042: mouse with wheel on second port\n");
				second_port = MOUSE;
			} else if (byte == 0x04) {
				printk(KERN_INFO "i8042: 5 button mouse on second port\n");
				second_port = MOUSE;
			} else if (byte == 0xAB) {
				uint8_t byte2;
				if (read_reg(&byte2, 250) < 0) {
					printk(KERN_INFO "i8042: can't detect device on second port\n");
					second_port = UNDEFINED;
					goto second_port_fail;
				}
				if (byte2 == 0x41 || byte2 == 0xC1) {
					printk(KERN_INFO "i8042: MF2 keyboard with translation on second port\n");
					second_port = KEYBOARD;
				} else {
					printk(KERN_INFO "i8042: can't detect device on second port\n");
					second_port = UNDEFINED;
				}
			} else {
				printk(KERN_INFO "i8042: can't detect device on second port\n");
				second_port = UNDEFINED;
			}
		} else {
			printk(KERN_INFO "i8042: can't detect device on second port\n");
			second_port = UNDEFINED;
		}
	}
second_port_fail:

	if (first_port) {
		dev1 = input_allocate_device();
		if (!dev1) {
			printk(KERN_ERR "i8042: can't allocate enough memory\n");
			return -ENOMEM;
		}

		dev1->name = "i8042_dev1";
		__set_bit(EV_KEY, dev1->evbit);
		bitmap_fill(dev1->keybit, KEY_CNT);

		if ((error = input_register_device(dev1))) {
			printk(KERN_ERR "i8042: can't register dev1\n");
			goto err_dev1_free;
		}
		if (request_irq(I8042_IRQ1, i8042_handler, IRQF_SHARED, "i8042_dev1", dev1)) {
			printk(KERN_ERR "i8042: can't register irq %d\n", I8042_IRQ1);
			error = -EBUSY;
			goto err_dev1_unreg;
		}

		if (write_dev1(I8042_KBD_ENABLE, 250) < 0) {
			printk(KERN_ERR "i8042: time limit exceeded\n");
			error = -ETIME;
			goto err_irq1_free;
		}
		if (read_reg(&byte, 250) < 0) {
			printk(KERN_ERR "i8042: time limit exceeded\n");
			error = -ETIME;
			goto err_irq1_free;
		}
	}

	if (second_port) {
		dev2 = input_allocate_device();
		if (!dev2) {
			printk(KERN_ERR "i8042: can't allocate enough memory\n");
			error = -ENOMEM;
			if (first_port)
				goto err_irq1_free;
			else
				return error;
		}

		dev2->name = "i8042_dev2";
		__set_bit(EV_KEY, dev2->evbit);
		__set_bit(BTN_0, dev2->keybit);

		if ((error = input_register_device(dev2))) {
			printk(KERN_ERR "i8042: can't register dev2\n");
			if (first_port)
				goto err_first_dev2_free;
			else
				goto err_second_dev2_free;
		}
		if (request_irq(I8042_IRQ12, i8042_handler, IRQF_SHARED, "i8042_dev2", dev2)) {
			printk(KERN_ERR "i8042: can't register irq %d\n", I8042_IRQ12);
			error = -EBUSY;
			if (first_port)
				goto err_first_dev2_unreg;
			else
				goto err_second_dev2_unreg;
		}

		if (write_dev2(I8042_KBD_ENABLE, 250) < 0) {
			printk(KERN_ERR "i8042: time limit exceeded\n");
			error = -ETIME;
			if (first_port)
				goto err_first_irq12_free;
			else
				goto err_second_irq12_free;
		}
		if (read_reg(&byte, 250) < 0) {
			printk(KERN_ERR "i8042: time limit exceeded\n");
			error = -ETIME;
			if (first_port)
				goto err_first_irq12_free;
			else
				goto err_second_irq12_free;
		}
	}


	return 0;

err_first_irq12_free:
	free_irq(I8042_IRQ12, dev2);
err_first_dev2_unreg:
	input_unregister_device(dev2);
err_irq1_free:
	free_irq(I8042_IRQ1, dev1);
err_dev1_unreg:
	input_unregister_device(dev1);
	return error;

err_second_irq12_free:
	free_irq(I8042_IRQ12, dev2);
err_second_dev2_unreg:
	input_unregister_device(dev2);
	return error;

err_dev1_free:
	input_free_device(dev1);
	return error;

err_first_dev2_free:
	input_free_device(dev2);
	free_irq(I8042_IRQ1, dev1);
	input_unregister_device(dev1);
	return error;

err_second_dev2_free:
	input_free_device(dev2);
	return error;
}

void cleanup_module(void)
{
	if (first_port) {
		free_irq(I8042_IRQ1, dev1);
		input_unregister_device(dev1);
	}
	if (second_port) {
		free_irq(I8042_IRQ12, dev2);
		input_unregister_device(dev2);
	}
}
