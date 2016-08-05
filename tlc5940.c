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

#include <linux/init.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
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

#define DRIVER_NAME						"tlc5940"
#define OF_MAX_CHAIN_SZ					"chain-sz"	// Number of connected devices from the device tree
#define OF_XLAT_BLANK					"gpio-xlat"	// Latch for loading the data into the output register

#define TLC5940_MAX_CHAIN_SZ			4096		// Used to limit the number of loop cycles when performing the discovery
#define TLC5940_SPI_MAX_SPEED			30000000	// Maximum possible SPI speed for the device
#define TLC5940_SPI_BITS_PER_WORD		8			// Word width
#define TLC5940_DEV_MAX_LEDS			16			// Maximum number of leds per device
#define TLC5940_GS_CHANNEL_WIDTH		12			// Grayscale PWM Control resolution (bits)

struct tlc5940_led {
	struct led_classdev	ldev;
	int					id;
	int					brightness;
	const char			*name;
	struct tcl5940_dev	*tlc;

	spinlock_t			lock;
};

struct tlc5940_dev {
	struct tlc5940_led	leds[TLC5940_DEV_MAX_LEDS];

	struct spi_device	*spi;			// SPI Device handler
	__u32				chain_sz;		// Number of devices in the chain
	int					xlat_gpio;		// Latch
};

/*
 * This function will return the number of discovered LT's
 * connected in a chain.
 */
//static int lt8500_discover(struct lt8500_dev *dev)
//{
//	__u8 frame[FRAME_SIZE];
//	__u8 feedback[FRAME_SIZE];
//	int ret = 0;
//	int i = 0;
//
//	/*
//	 * Fill the frame with the mask: 0x0AAA starting from MSB,
//	 * so it will look like this:
//	 * 0x0A < MSB
//	 * 0xAA < LSB
//	 */
//	for (i = 0; i < FRAME_SIZE - 1; i += 2) {
//		frame[i] = 0x0A;
//		frame[i + 1] = 0xAA;
//	}
//	frame[FRAME_SIZE - 1] = CMD_ASYNC_UPDATE_FRAME;
//
//	// Disable output before performing discovery sequence
//	lt8500_cmd_output_toggle(dev, STATE_DISABLED);
//
//	/*
//	 * Begin transfer in a loop untill we receive the same
//	 * pattern back which means that we reached end of chain.
//	 */
//
//	return ret;
//}

static const struct of_device_id tlc5940_of_match[] = {
	{ .compatible = "ti,tlc5940", },
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, tlc5940_of_match);

static int tlc5940_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct tlc5940_dev *tlcdev;
	__u32 count;
	int latch;
	int ret = 0;

	if (!np) {
		printk(KERN_ERR "tlc5940: No platform data specified");
		return -ENODEV;
	}

	tlcdev = devm_kzalloc(&spi->dev, sizeof(*tlcdev), GFP_KERNEL);
	if (!tlcdev)
		return -ENOMEM;

	tlcdev->spi = spi;

	if (tlcdev->spi->max_speed_hz > TLC5940_SPI_MAX_SPEED) {
		dev_warn(&tlcdev->spi->dev, "spi max speed (%u) is too high, setting to default %u",
				tlcdev->spi->max_speed_hz, TLC5940_SPI_MAX_SPEED);
		tlcdev->spi->max_speed_hz = TLC5940_SPI_MAX_SPEED;
	}

	/*
	 * Get the number of connected device from the device tree structure
	 * and then scan and compare to actual number of connected devices.
	 */
	if (of_match_device(of_match_ptr(tlc5940_of_match), &spi->dev)) {
		if (!of_property_read_u32(np, OF_MAX_CHAIN_SZ, &count)) {
			tlcdev->chain_sz = count;
		}
	} else {
		// TODO: call scan function to obtain the number of connected devices
	}

	/*
	 * Get XLAT pin
	 */
	if (of_match_device(of_match_ptr(tlc5940_of_match), &spi->dev)) {
		latch = of_get_named_gpio(np, OF_XLAT_BLANK, 0);
		if (gpio_is_valid(latch)) {
			if (devm_gpio_request(&spi->dev, latch, OF_XLAT_BLANK)) {
				dev_err(&tlcdev->spi->dev, "unable to request gpio for XLAT pin");
				return -EINVAL;
			}
			tlcdev->xlat_gpio = latch;
		} else {
			dev_err(&tlcdev->spi->dev, "specified gpio pin for XLAT is invalid");
			return -EINVAL;
		}
	}

	// Set the direction of XLAT pin
	ret = gpio_direction_output(tlcdev->xlat_gpio, 1);
	if (ret) {
		dev_err(&tlcdev->spi->dev, "Failed to configure XLAT pin for output: %d\n", ret);

		return ret;
	}

	/*
	 * Set the amount of bits to be transferred at once
	 */
	spi->bits_per_word = TLC5940_SPI_BITS_PER_WORD;

	// TODO: Init locks

	dev_info(&tlcdev->spi->dev, "TI tlc5940 SPI driver registered");

	return ret;
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
};
module_spi_driver(tlc5940_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrei Andreyanau <a.andreyanau@sam-solutions.com>");
MODULE_DESCRIPTION("TI TLC5940 driver");
MODULE_ALIAS("spi:" DRIVER_NAME);
