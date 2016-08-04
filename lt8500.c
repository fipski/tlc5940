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
#define OF_MAX_CHAIN_SZ					"chain-sz"
#define FRAME_SIZE						0x49
//#define CMD_SYNC_UPDATE_FRAME			0x00
//#define CMD_ASYNC_UPDATE_FRAME			0x10
//#define CMD_CORR_FRAME					0x20
//#define CMD_OUTPUT_ENABLE_FRAME			0x30
//#define CMD_OUTPUT_DISABLE_FRAME		0x40
//#define CMD_SELF_TEST_FRAME				0x50
//#define CMD_PHASE_SHIFT_TOGGLE_FRAME	0x60
//#define CMD_CORR_TOGGLE_FRAME			0x70

typedef struct {
	__u8	data[FRAME_SIZE];
} tlc5940_frame;

struct tlc5940_dev {
	struct spi_device	*spi;			// SPI Device handler
	__u16				chain_sz;		// Number of devices in the chain
	int					ldi_blank_gpio;	// LDI/Blank GPIO pin
};

typedef enum {
	STATE_DISABLED,
	STATE_ENABLED
} stateOutput;

/*
 * TODO: pass the amount of LT's in the chain
 */
//static void lt8500_cmd_output_toggle(struct lt8500_dev *dev, stateOutput state)
//{
//	__u8 frame[FRAME_SIZE];
//	__u8 i = 0;
//
//	memset(frame, 0x00, FRAME_SIZE);
//
//	spin_lock(&dev->lock);
//
//	for (i = 0; i < FRAME_SIZE - 1; i++) {
//		spi_write(dev->spi, (const __u8 *)&frame[i], 1);
//	}
//
//	if (state == STATE_ENABLED)
//		spi_write(dev->spi, (const __u8 *)CMD_OUTPUT_ENABLE_FRAME, 1);
//	else
//		spi_write(dev->spi, (const __u8 *)CMD_OUTPUT_DISABLE_FRAME, 1);
//
//	gpio_set_value(dev->ldi_blank_gpio, 1);
//	// TODO: Here should be a delay less than 5us
//	gpio_set_value(dev->ldi_blank_gpio, 0);
//
//	spin_unlock(&dev->lock);
//}

//static void lt8500_update_frame(struct work_struct *work)
//{
//	struct lt8500_dev *dev = container_of(work, struct lt8500_dev, work);
//
//}

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
	struct tlc5940_dev *ltdev;
	__u16 count;
	int ret = 0;

	if (!np) {
		printk(KERN_ERR "No platform data specified");
		return -ENODEV;
	}

	ltdev = devm_kzalloc(&spi->dev, sizeof(*ltdev), GFP_KERNEL);
	if (!ltdev)
		return -ENOMEM;

	/*
	 * Get the number of
	 */
	if (of_match_device(of_match_ptr(tlc5940_of_match), &spi->dev)) {
		if (!of_property_read_u16(np, OF_MAX_CHAIN_SZ, &count)) {
			ltdev->chain_sz = count;
			printk(KERN_INFO "%s = %d", OF_MAX_CHAIN_SZ, ltdev->chain_sz);
		}
	}

	/*
	 * Set the amount of bits to be transferred at once
	 */
	spi->bits_per_word = 8;

	// TODO: Init locks

	printk(KERN_INFO "TI TLC5940 SPI device registered");
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrei Andreyanau <a.andreyanau@sam-solutions.com>");
MODULE_DESCRIPTION("TI TLC5940 driver");
MODULE_ALIAS("spi:tlc5940");
