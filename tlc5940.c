/*
 * Copyright 2016
 * Andrei Andreyanau <a.andreyanau@sam-solutions.com
 *
 * Based on:
 * 	- leds-tlc5940.c by Jordan Yelloz <jordan@yelloz.me>
 * 	- leds-dac124s085.c by Guennadi Liakhovetski <lg@denx.de>
 *
 * 	This file is subject to the terms and conditions of version 2 of
 * 	the GNU General Public License. See the file LICENSE in the main
 * 	directory of this archive for more details.
 *
 * 	LED driver for the TLC5940 SPI LED Controller
 */
#define DEBUG
#include <linux/init.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>

#define DRIVER_NAME						"tlc5940"
#define OF_MAX_CHAIN_SZ					"chain-sz-max"	// Number of connected devices from the device tree (16)
#define OF_XLAT_GPIO					"gpio-xlat"		// Latch for loading the data into the output register (device tree)
#define OF_BLANK_GPIO					"gpio-blank"	// Blank for turning on/off all leds (device tree)

#define TLC5940_DEF_MAX_CHAIN_SZ		4096		// Used to limit the number of loop cycles when performing the discovery (default if not set by the DT)
#define TLC5940_SPI_MAX_SPEED			30000000	// Maximum possible SPI speed for the device
#define TLC5940_SPI_BITS_PER_WORD		8			// Word width
#define TLC5940_DEV_MAX_LEDS			16			// Maximum number of leds per device
#define TLC5940_GS_CHANNEL_WIDTH		12			// Grayscale PWM Control resolution (bits)
#define TLC5940_FRAME_SIZE				24			// (12 bits * 16 channels) / 8 bit
#define TLC5940_LED_NAME_SZ				16
#define TLC5940_HRTMR_DEF_DELAY_NS		5000000		// 5ms

#define XLAT 49 // hard coded io  P9_23 (GPIO49)

struct tlc5940_led {
	struct tlc5940_dev	*tlc;
	struct led_classdev	ldev;
	int					id;
	int					brightness;
	char				name[TLC5940_LED_NAME_SZ];

	spinlock_t			lock;
};

struct tlc5940_dev {
	struct tlc5940_led leds[TLC5940_DEV_MAX_LEDS];		// Number of available leds per device
	struct spi_device	*spi;							// SPI Device handler
	struct list_head	list;

	struct mutex		mlock;
	struct hrtimer		timer;
	struct work_struct	work;

	int 				bank_id;						// Device's number
	int					chain_sz;						// Number of devices in the chain
	int					xlat_gpio;						// Latch
	int					blank_gpio;						// Blank
    int                 xlat_gpio_hard;                 // hardcoded xlatch address
	bool				new_data;
};

static unsigned long hrtimer_delay = TLC5940_HRTMR_DEF_DELAY_NS;

static enum hrtimer_restart tlc5940_timer(struct hrtimer *timer)
{
	struct tlc5940_dev *tlc = container_of(timer, struct tlc5940_dev, timer);

	if (tlc->new_data)
		schedule_work(&tlc->work);

	hrtimer_forward_now(timer, ktime_set(0, hrtimer_delay));

	return HRTIMER_RESTART;
}

static void tlc5940_work(struct work_struct *work)
{
	struct tlc5940_dev *tlc = container_of(work, struct tlc5940_dev, work);
	struct spi_device *spi = tlc->spi;
	struct device *dev = &spi->dev;
	struct tlc5940_dev *cur_item;
	struct tlc5940_dev *next_item;
	struct spi_transfer tx;
	struct spi_message msg;
	__u8 *message_tx;
	__u8 *message_rx;
    int led_br_cur = 0;
	int led_br_next = 0;
	int ret = 0;
	int i = 0;

	message_tx = kmalloc(TLC5940_FRAME_SIZE, GFP_KERNEL);
	message_rx = kmalloc(TLC5940_FRAME_SIZE, GFP_KERNEL);

	memset(message_tx, 0x00, TLC5940_FRAME_SIZE);
	memset(message_rx, 0x00, TLC5940_FRAME_SIZE);
	memset(&tx, 0x00, sizeof(tx));
	spi_message_init(&msg);
	tx.rx_buf = message_rx;
	tx.tx_buf = message_tx;
	tx.len = TLC5940_FRAME_SIZE * 2; // (philipp) don't recieve data....
	spi_message_add_tail(&tx, &msg);

	if (mutex_lock_interruptible(&tlc->mlock)) {
		dev_err(dev, "unable to set lock");
		return;
	}

	list_for_each_entry_safe_reverse(cur_item, next_item, &tlc->list, list) {
		memset(message_tx, 0x00, TLC5940_FRAME_SIZE);
		memset(message_rx, 0x00, TLC5940_FRAME_SIZE);

		for (i = 0; i < (TLC5940_DEV_MAX_LEDS >> 1); i++) {
			led_br_cur = cur_item->leds[i * 2].brightness;
			led_br_next = cur_item->leds[i * 2 + 1].brightness;

			message_tx[3 * i + 0] = (led_br_cur >> 4) & 0xff;
			message_tx[3 * i + 1] =
					((led_br_cur & 0x0f) << 4) + ((led_br_next >> 8) & 0x0f);
			message_tx[3 * i + 2] = led_br_next & 0xff;
		}

		ret = spi_sync(spi, &msg);
		if (ret) {
			dev_err(dev, "spi sync error");
			break;
		}

#ifdef DEBUG

	
        
		printk("\ntx->");
		for (ret = 0; ret < TLC5940_FRAME_SIZE; ret++) {
			printk(KERN_CONT "0x%02x ", message_tx[ret]);
        }
		printk("\nrx->");
        for (ret = 0; ret < TLC5940_FRAME_SIZE; ret++) {
            printk(KERN_CONT "0x%02x ", message_rx[ret]);
        }
#endif /* DEBUG */
    }

#ifdef DEBUG
    printk(KERN_INFO "xlat_gpio adress is %x \n", tlc->xlat_gpio);
    printk(KERN_INFO "blank address is %x \n", tlc->blank_gpio);
    printk(KERN_INFO "xlat_gpio is %x \n", gpio_get_value(tlc->xlat_gpio));
    printk(KERN_INFO "blank address is %x \n", gpio_get_value(tlc->blank_gpio));
#endif
	gpio_set_value(tlc->xlat_gpio, 1);
	gpio_set_value(tlc->blank_gpio, 1);
	// TODO: add delay
#ifdef DEBUG
    printk(KERN_INFO "xlat_gpio is %x \n", gpio_get_value(tlc->xlat_gpio));
#endif
    udelay(5); //2usec
	gpio_set_value(tlc->xlat_gpio, 0); // (philipp), debug
#ifdef DEBUG
    printk(KERN_INFO "xlat_gpio is %x \n", gpio_get_value(tlc->xlat_gpio));
#endif
	tlc->new_data = 0;

	kfree(message_tx);
	kfree(message_rx);

	mutex_unlock(&tlc->mlock);
}

static void tlc5940_set_brightness(struct led_classdev *ldev,
		const enum led_brightness brightness)
{
	struct tlc5940_led *led = container_of(ldev, struct tlc5940_led, ldev);
	struct tlc5940_dev *dev = led->tlc;

	spin_lock(&led->lock);
	led->brightness = brightness;
	spin_unlock(&led->lock);

	dev->new_data = 1;
}

static int tlc5940_discover(struct tlc5940_dev *dev, struct spi_device *spi,
		int max_sz)
{
	struct spi_transfer tx;
	struct spi_message msg;

	int ret = 0;
	int i = 0;

#ifdef DEBUG
	int k = 0;
#endif /* DEBUG */

	__u8 *frame_tx;
	__u8 *frame_rx;
	__u8 *frame_void;

	if (mutex_lock_interruptible(&dev->mlock)) {
		return 0;
	}

	frame_tx = kmalloc(TLC5940_FRAME_SIZE, GFP_KERNEL);
	frame_rx = kmalloc(TLC5940_FRAME_SIZE, GFP_KERNEL);
	frame_void = kmalloc(TLC5940_FRAME_SIZE, GFP_KERNEL);

	// Fill TX frame with 0xAA mask
	memset(frame_tx, 0xAA, TLC5940_FRAME_SIZE);
	// Clear RX frame
	memset(frame_rx, 0x00, TLC5940_FRAME_SIZE);
	// Fill empty frame with 0xFF mask
	memset(frame_void, 0xFF, TLC5940_FRAME_SIZE);

	memset(&tx, 0x00, sizeof(tx));
	spi_message_init(&msg);
	tx.rx_buf = frame_rx;
	tx.tx_buf = frame_tx;
	tx.len = TLC5940_FRAME_SIZE * 2;
	spi_message_add_tail(&tx, &msg);

	/*
	 * Send a packet to the device and read its response.
	 * How it works. After sending a packet and not receiving it
	 * within the range of 0 .. max_sz - it means that there are
	 * no TLC5940s' at all, so we return 0. Otherwise we increase
	 * the counter until we receive the packet back within the
	 * range of 0 .. max_sz. After sending each packet and not receiving
	 * the same packet back we increase the counter until we either
	 * reach max_sz or lesser value meaning that we've found all
	 * devices in a chain.
	 */
	for (i = 0; i < max_sz; i++) {
		// Clear receive buffer
		memset(frame_rx, 0x00, TLC5940_FRAME_SIZE);
		// Start synced SPI ops
		ret = spi_sync(spi, &msg);
		if (ret) {
			ret = 0;
			dev_err(&spi->dev, "spi sync error");

			break;
		}

#ifdef DEBUG
		printk("\ntx->");
		for (k = 0; k < TLC5940_FRAME_SIZE; k++) {
			printk(KERN_CONT "0x%02x ", frame_tx[k]);
		}

		printk("\nrx->");
		for (k = 0; k < TLC5940_FRAME_SIZE; k++) {
			printk(KERN_CONT "0x%02x ", frame_rx[k]);
		}
#endif /* DEBUG */

		// Compare buffers
		if (!strncmp(frame_rx, frame_tx, TLC5940_FRAME_SIZE) && (i == 0)) {
			// TODO: DEBUG ONLY
			ret = 16;
			//ret = 0;
			// Looks like it's a close loop -> no devices connected
			dev_warn(&spi->dev, "close loop detected");

			break;
		} else if (!strncmp(frame_rx, frame_void, TLC5940_FRAME_SIZE)
				&& (i == (max_sz - 1))) {
			// We've scanned the bus, but didn't find anything
			printk(KERN_INFO "actually none detected\n");
            ret = 1;// (philipp) made 1 the minimum of devices
			break;
		} else if (!strncmp(frame_rx, frame_tx, TLC5940_FRAME_SIZE)){
			// It seems that we've found something so just return ret
			break;
		}

		ret++;
	}

	kfree(frame_tx);
	kfree(frame_rx);
	kfree(frame_void);

	mutex_unlock(&dev->mlock);

	return ret;
}

static const struct of_device_id tlc5940_of_match[] = {
	{ .compatible = "ti,tlc5940", },
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, tlc5940_of_match);

static int tlc5940_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct device *dev = &spi->dev;
	struct work_struct *work;
	struct hrtimer *timer;
	struct tlc5940_dev *tlcdev;
	struct tlc5940_dev *item;
	struct tlc5940_led *leddev;
	int count = 0;
	int gpio = -EINVAL;
	int i = 0;
	int ret = 0;
    
    printk(KERN_INFO "Hello. Loading tlc5940 driver...\n");

	if (!np) {
		printk(KERN_ERR "tlc5940: No platform data specified");
		return -ENODEV;
	}

	tlcdev = devm_kzalloc(dev, sizeof(struct tlc5940_dev), GFP_KERNEL);
	if (!tlcdev)
		return -ENOMEM;

	mutex_init(&tlcdev->mlock);

	// Set the amount of bits to be transferred at once
	spi->bits_per_word = TLC5940_SPI_BITS_PER_WORD;

	// Set SPI master device transfer rate
	if (spi->max_speed_hz > TLC5940_SPI_MAX_SPEED) {
		dev_warn(dev, "spi max speed (%u) is too high, "
				"setting to default %u", tlcdev->spi->max_speed_hz,
				TLC5940_SPI_MAX_SPEED);
		spi->max_speed_hz = TLC5940_SPI_MAX_SPEED;
	}

	/*
	 * Get the number of connected device from the device tree structure
	 * and then scan and compare to actual number of connected devices.
	 */
	if (of_match_device(of_match_ptr(tlc5940_of_match), dev)) {
		if (!of_property_read_u32(np, OF_MAX_CHAIN_SZ, &count)) {
			tlcdev->chain_sz = tlc5940_discover(tlcdev, spi, count);
			if (!tlcdev->chain_sz) {
				// Device is not connected at all
				dev_err(dev, "%u device(s) found",
						tlcdev->chain_sz);
				return -ENODEV;
			} else {
				dev_info(dev, "found %u device(s)",
						tlcdev->chain_sz);
			}
		}
	} else {
		count = tlc5940_discover(tlcdev, spi, TLC5940_DEF_MAX_CHAIN_SZ);
		if (count > 0) {
			tlcdev->chain_sz = count;
			dev_info(dev, "found %u device(s) in a chain",
					tlcdev->chain_sz);
		} else {
			// Device is not connected at all
			dev_err(dev, "%u device(s) found", count);
			return -ENODEV;
		}
	}

	// Get XLAT pin
	if (of_match_device(of_match_ptr(tlc5940_of_match), dev)) {
		gpio = of_get_named_gpio(np, OF_XLAT_GPIO, 0);
		if (gpio_is_valid(gpio)) {
			if (devm_gpio_request(dev, gpio, OF_XLAT_GPIO)) {
				dev_err(dev, "unable to request gpio "
						"for XLAT pin");
				return -EINVAL;
			}
			tlcdev->xlat_gpio = gpio;
		} else {
			dev_err(dev, "specified gpio pin for XLAT "
					"is invalid");
			return -EINVAL;
		}
	}

	// Get BLANK pin
	if (of_match_device(of_match_ptr(tlc5940_of_match), dev)) {
		gpio = of_get_named_gpio(np, OF_BLANK_GPIO, 0);
		if (gpio_is_valid(gpio)) {
			if (devm_gpio_request(dev, gpio, OF_BLANK_GPIO)) {
				dev_err(dev, "unable to request gpio "
						"for BLANK pin");
				return -EINVAL;
			}
			tlcdev->blank_gpio = gpio;
		} else {
			dev_err(dev, "specified gpio pin for BLANK "
					"is invalid");
			return -EINVAL;
		}
	}



	// Set the direction of XLAT pin as OUT, set it low by-default
	ret = gpio_direction_output(tlcdev->xlat_gpio, 0);
	if (ret) {
		dev_err(dev, "Failed to configure XLAT pin for "
				"output: %d\n", ret);

		return -EINVAL;
	}

	// Set the direction of BLANK pin as OUT, set it low by-default
	ret = gpio_direction_output(tlcdev->blank_gpio, 0);
	if (ret) {
		dev_err(dev, "Failed to configure BLANK pin for "
				"output: %d\n", ret);

		return -EINVAL;
	}

	tlcdev->spi = spi;
	tlcdev->bank_id = 0;
	tlcdev->new_data = 0;
	work = &tlcdev->work;
	timer = &tlcdev->timer;

	INIT_WORK(work, tlc5940_work);

	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer->function = tlc5940_timer;

	INIT_LIST_HEAD(&tlcdev->list);

	for (count = 0; count < tlcdev->chain_sz; count++) {
		item = devm_kzalloc(dev, sizeof(struct tlc5940_dev), GFP_KERNEL);
		if (!item)
			return -ENOMEM;

		item->bank_id = count;
		item->chain_sz = tlcdev->chain_sz;
		item->mlock = tlcdev->mlock;
		item->spi = tlcdev->spi;
		item->work = tlcdev->work;
		item->timer = tlcdev->timer;
		item->xlat_gpio = tlcdev->xlat_gpio;
		item->blank_gpio = tlcdev->blank_gpio;
		item->new_data = tlcdev->new_data;

		for (i = 0; i < TLC5940_DEV_MAX_LEDS; i++) {
			leddev = &item->leds[i];
			leddev->tlc = tlcdev;
			leddev->id = i;

			memset(leddev->name, 0, TLC5940_LED_NAME_SZ);
			sprintf(leddev->name, "led%u-%u", item->bank_id, leddev->id);

			leddev->brightness = LED_OFF;
			leddev->ldev.name = leddev->name;
			leddev->ldev.brightness = LED_OFF;
			leddev->ldev.max_brightness = 0xfff;
			spin_lock_init(&leddev->lock);
			leddev->ldev.brightness_set = tlc5940_set_brightness;

			ret = devm_led_classdev_register(dev, &leddev->ldev);
			if (ret < 0) {
				dev_err(dev, "unable to register SPI led device %s",
						leddev->name);

				return ret;
			}
		}
		list_add(&item->list, &tlcdev->list);
	}

	spi_set_drvdata(spi, tlcdev);
	hrtimer_delay = TLC5940_HRTMR_DEF_DELAY_NS * tlcdev->chain_sz;
	hrtimer_start(timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	dev_info(dev, "TI tlc5940 SPI driver registered");

	return ret;
}

static int tlc5940_remove(struct spi_device *spi)
{
	struct tlc5940_dev *tlc = spi_get_drvdata(spi);
	struct device *dev = &tlc->spi->dev;
	struct work_struct *work = &tlc->work;
	struct hrtimer *timer = &tlc->timer;
	struct tlc5940_led *led;
	struct tlc5940_dev *next_node;
	struct tlc5940_dev *cur_node;
	int i = 0;

	hrtimer_cancel(timer);
	cancel_work_sync(work);

	list_for_each_entry_safe(cur_node, next_node, &tlc->list, list) {
		for (i = 0; i < TLC5940_DEV_MAX_LEDS; i++) {
			led = &cur_node->leds[i];
			devm_led_classdev_unregister(dev, &led->ldev);
		}
		list_del(&cur_node->list);

		if (cur_node)
			devm_kfree(dev, cur_node);
	}

	dev_info(dev, "driver removed");
    printk(KERN_INFO "Bye / TLC5940 \n");
	return 0;
}

static const struct spi_device_id tlc5940_id[] = {
	{ DRIVER_NAME, 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, tlc5940_id);

static struct spi_driver tlc5940_driver = {
	.driver = {
		.name			= DRIVER_NAME,
		.owner			= THIS_MODULE,
		.of_match_table	= of_match_ptr(tlc5940_of_match),
	},
	.id_table	= tlc5940_id,
	.probe		= tlc5940_probe,
	.remove		= tlc5940_remove,
};
module_spi_driver(tlc5940_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrei Andreyanau <a.andreyanau@sam-solutions.com>");
MODULE_DESCRIPTION("TI TLC5940 driver");
MODULE_ALIAS("spi:" DRIVER_NAME);
