#include <linux/init.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define CMD_SYNC_UPDATE_FRAME			0x00
#define CMD_ASYNC_UPDATE_FRAME			0x10
#define CMD_CORR_FRAME					0x20
#define CMD_OUTPUT_ENABLE_FRAME			0x30
#define CMD_OUTPUT_DISABLE_FRAME		0x40
#define CMD_SELF_TEST_FRAME				0x50
#define CMD_PHASE_SHIFT_TOGGLE_FRAME	0x60
#define CMD_CORR_TOGGLE_FRAME			0x70

typedef struct {
	__u16	brightness[48];
	__u8	cmd;
} lt8500_update_frame;

typedef struct {
	__u8	cor[48];
	__u8	cmd;
} lt8500_cor_frame;

typedef struct {
	__u8	cor[48];
	__u8	last_cmd;
	__u8	flags;
} lt8500_status_frame;

struct lt8500_dev {
	struct led_classdev	ldev;
	struct spi_device	*spi;			// Device handler
	struct mutex		mutex;

	int					count;			// Number of devices in the chain
	int					ldi_blank_gpio;	// LDI/Blank GPIO pin
	spinlock_t			lock;			// In-use lock for a device
	spinlock_t			ldi_blank_lock;	// In-use lock for LDIBLANK pin
};

typedef enum {
	STATE_DISABLED,
	STATE_ENABLED
} stateOutput;

/*
 * TODO: pass the amount of LT's in the chain
 * TODO: requires LDIBLANK toggle
 */
static void lt8500_cmd_output_toggle(struct lt8500_dev *dev, stateOutput state)
{
	__u16 data[48];
	__u16 word;
	__u8 i = 0;

	memset(data, 0x0000, ARRAY_SIZE(data));

	spin_lock(&dev->lock);

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		word = cpu_to_le16(data[i]);
		spi_write(dev->spi, (const __u8 *)(word >> 8), 1);
		spi_write(dev->spi, (const __u8 *)(word & 0xff), 1);
	}

	if (state == STATE_ENABLED)
		spi_write(dev->spi, (const __u8 *)CMD_OUTPUT_ENABLE_FRAME, 1);
	else
		spi_write(dev->spi, (const __u8 *)CMD_OUTPUT_DISABLE_FRAME, 1);

	spin_unlock(&dev->lock);
}

static int lt8500_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct lt8500_dev *ltdev;
	int ret = 0;

	if (!np) {
		printk("No platform data specified");
		return -ENODEV;
	}

	ltdev = devm_kzalloc(&spi->dev, sizeof(*ltdev), GFP_KERNEL);

	/*
	 * Set the amount of bits to be transferred at once
	 */
	spi->bits_per_word = 8;

	return ret;
}

static int lt8500_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver lt8500_driver = {
		.driver = {
			.name = "lt8500-spi",
		},
		.probe = lt8500_probe,
		.remove = lt8500_remove,
};

module_spi_driver(lt8500_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrei Andreyanau <a.andreyanau@sam-solutions.com>");
MODULE_DESCRIPTION("Linear Technology LT8500 driver.");
MODULE_ALIAS("spi:lt8500");
