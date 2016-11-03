/*
 * Copyright 2018
 *
 * Based on:
 * 	- leds-tlc5940.c by Jordan Yelloz <jordan@yelloz.me>
 *  - branch of Andrei Andreyanau <a.andreyanau@sam-solutions.com
 * 	- leds-dac124s085.c by Guennadi Liakhovetski <lg@denx.de>
 *
 * 	This file is subject to the terms and conditions of version 2 of
 * 	the GNU General Public License. See the file LICENSE in the main
 * 	directory of this archive for more details.
 *
 * 	LED driver for the TLC5940 SPI LED Controller
 */
/* #define DEBUG */ 
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
#include <linux/pwm.h>

#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/uaccess.h>


/* Framebuffer limited by spi_bufsiz! Node: bufsiz = rx + tx!
 * TODO some cleanup */


#define DRIVER_NAME						"tlc5940"
#define OF_MAX_CHAIN_SZ					"chain-sz-max"	// Number of connected devices from the device tree (16)
#define OF_XLAT_GPIO					"gpio-xlat"		// Latch for loading the data into the output register (device tree)
#define OF_BLANK_GPIO					"gpio-blank"	// Blank for turning on/off all leds (device tree)
#define OF_PWM                          "pwms"
//cycles when performing the discovery (if not set by the DT)
/* #define TLC5940_SPI_MAX_SPEED			30000	// Maximum possible SPI speed for the device */
#define TLC5940_SPI_BITS_PER_WORD		8	   // Word width
#define TLC5940_DEVICES                 1      // Deactivate auto detect
#define TLC5940_LED_NAME_SZ				16
#define TLC5940_LEDS                    (2*48)
#define TLC5940_FRAME_SIZE	    	  	(TLC5940_LEDS * 3 / 2)	//24 per chip (12b*16)/8b
#define TLC5940_RESOLUTION              1024  // Number of PWM Greyscales, max 4096
#define TLC5940_GSCLK_PERIOD_NS         (50)  // 20 MHz GSCLK
#define TLC5940_GSCLK_DUTY_CYCLE_NS     (TLC5940_GSCLK_PERIOD_NS / 2)
#define TLC5940_BLANK_PERIOD_NS         (TLC5940_RESOLUTION * TLC5940_GSCLK_PERIOD_NS) 

#define DEVICE_NAME "fbuf"            // SYSFS device name
#define CLASS_NAME "tlc_framebuffer"


/* 
 * Prototypes 
 */
int init_module(void);
void cleanup_module(void);
static int dev_open(struct inode *inode, struct file *);
static int dev_release(struct inode *inode, struct file *);
static ssize_t dev_read(struct file *fp, char *buf, size_t len, loff_t *off);
static ssize_t dev_write(struct file *, const char *buf, size_t len, 
        loff_t *off);

/*
 * Variables 
 */

bool    new_data;
bool    latching;
struct  kobject *kobj_ref;
u8 framebuffer_tx[TLC5940_FRAME_SIZE];
static atomic_t atomic;

struct tlc5940_led {
    struct tlc5940_dev	*tlc;
    struct led_classdev	ldev;
    int					id;
    int                 brightness;
    char				name[TLC5940_LED_NAME_SZ];

    spinlock_t			lock;
};


struct tlc5940_dev {
    struct tlc5940_led leds[TLC5940_LEDS];		// Number of available colors
    struct spi_device	*spi;							// SPI Device handler
    struct pwm_device   *pwm;
    struct list_head	list;

    struct mutex		mlock;
    struct hrtimer		timer;
    struct work_struct	work;

    int 				bank_id;						// Device's number
    int					chain_sz;						// Number of devices in chain
    int					xlat_gpio;						// Latch
    int					blank_gpio;						// Blank
};




static unsigned long hrtimer_delay = TLC5940_BLANK_PERIOD_NS;

/* variables for sysfs */
static struct cdev *gko_cdev;    
static struct device *gko_device;
static struct class *gko_class;
static dev_t gko_dev;
static char *gko_buffer;           /* dynamic allocated */
static int gko_buffer_end = -1;
static atomic_t gko_buffer_start; /* may be changed by sysfs attr */

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = dev_read,
    .write = dev_write,
    .open = dev_open,
    .release = dev_release
};

/*
 * Called when device is opened
 */
static int dev_open(struct inode *inode, struct file *fp)
{
    int old;
    old = atomic_cmpxchg(&atomic, 0, 1);
    if (old == 0) {
        printk("Succes: first open\n");

    } else {
        printk("Failed: already opened\n");
        return -EBUSY;
    }

    return 0;

}

/* 
 * Called when device is released. The device is
 * released when there is no process using it.
 */
static int dev_release(struct inode *inode, struct file *fp)
{
    int old;

    printk("cdev_release\n");

    old = atomic_cmpxchg(&atomic, 1, 0);
    printk("Release: old = %d\n", old);
    return 0;
}

/*
 * always read the whole buffer
 */
static ssize_t dev_read(struct file *fp, char *buf, size_t len, loff_t *off)
{
    unsigned long rval;

    if (*off == 0) {
        rval = copy_to_user(buf, 
                &framebuffer_tx,
                TLC5940_FRAME_SIZE);

        if (rval != 0)
            return -EFAULT;

        *off += TLC5940_FRAME_SIZE;

        return TLC5940_FRAME_SIZE;
    } else {
        return 0;
    }

}

/*
 * start writing 
 */
static ssize_t dev_write(struct file *fp, const char *buf, size_t len, 
        loff_t *off)
{
    unsigned long rval;

    printk(KERN_DEBUG DEVICE_NAME 
            " dev_write(fp, buf, len = %zu, off = %d )\n", len, (int)*off);
    /* if (len > TLC5940_FRAME_SIZE) { */
    /*     printk(KERN_DEBUG DEVICE_NAME " data too long!"); */
    /*     return -1; */
    /* } */

    /* if (len > TLC5940_FRAME_SIZE) */
    /*     len = TLC5940_FRAME_SIZE; */

    rval = copy_from_user(&framebuffer_tx, 
            buf,
            TLC5940_FRAME_SIZE);

    if (rval != 0) {
        printk(KERN_DEBUG DEVICE_NAME " copy_from_user() failed\n");
        return -EFAULT;
    }


    *off += TLC5940_FRAME_SIZE;
    printk(KERN_DEBUG DEVICE_NAME " String read: %s", gko_buffer);
    
    new_data = 1;

    return TLC5940_FRAME_SIZE;
}



static ssize_t buffer_start_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gko_buffer_start));
}

static ssize_t buffer_end_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", gko_buffer_end);
}

static ssize_t buffer_start_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int tmp;
    sscanf(buf, "%d", &tmp);
    if (tmp < 0 || gko_buffer_end < 0)
        tmp = 0;
    else if (tmp > gko_buffer_end)
        tmp = gko_buffer_end - 1;

    atomic_set(&gko_buffer_start, tmp);
    return PAGE_SIZE;
}

static DEVICE_ATTR(buffer_start, S_IRUSR | S_IWUSR, buffer_start_show, buffer_start_store);
static DEVICE_ATTR(buffer_end, S_IRUSR, buffer_end_show, NULL);        


static enum hrtimer_restart tlc5940_timer(struct hrtimer *timer)
{
    struct tlc5940_dev *tlc = container_of(timer, struct tlc5940_dev, timer);

    hrtimer_forward_now(timer, ktime_set(0, hrtimer_delay));
    if (new_data){
        new_data = 0;
        schedule_work(&tlc->work);
    }
    if (latching)
        return HRTIMER_RESTART;

    /* toggle blank pin tu reset tlc5940's GS Counter */
    gpio_set_value(tlc->blank_gpio, 1);
    /* 1 u sec gets stable resets */
    udelay(1);
    gpio_set_value(tlc->blank_gpio, 0);

    return HRTIMER_RESTART;
}

static void tlc5940_work(struct work_struct *work)
{
    struct tlc5940_dev *tlc = container_of(work, struct tlc5940_dev, work);
    struct spi_device *spi = tlc->spi;
    struct device *dev = &spi->dev;
    struct spi_transfer tx;
    struct spi_message msg;
    u8 *message_tx;
    /* u8 *message_rx; */
    
    int ret = 0;

    message_tx = kmalloc(TLC5940_FRAME_SIZE, GFP_KERNEL);
    /* message_rx = kmalloc(TLC5940_FRAME_SIZE, GFP_KERNEL); */ 
    

    memcpy(message_tx,framebuffer_tx,TLC5940_FRAME_SIZE);
    memset(&tx, 0x00, sizeof(tx));
    tx.len = TLC5940_FRAME_SIZE;
    tx.tx_buf = message_tx;
    /* tx.rx_buf = message_rx; */

    mutex_lock(&tlc->mlock);
    spi_message_init(&msg);
    spi_message_add_tail(&tx, &msg);
    ret = spi_sync(spi, &msg);
    gpio_set_value(tlc->blank_gpio, 1); // recommended in datasheet, causes flicker
    gpio_set_value(tlc->xlat_gpio, 1);
    udelay(1);
    gpio_set_value(tlc->xlat_gpio, 0);
    gpio_set_value(tlc->blank_gpio, 0);

    if (ret) {
        dev_err(dev, "spi sync error %d",ret);
    }

#ifdef DEBUG
printk("tx->");
for (ret = 0; ret < TLC5940_FRAME_SIZE; ret++) {
    printk(KERN_CONT "0x%02x ", framebuffer_tx[ret]);
}

#endif /* DEBUG */

    kfree(message_tx);
    mutex_unlock(&tlc->mlock);
}

static void tlc5940_set_brightness(struct led_classdev *ldev,
        const enum led_brightness brightness)
{
    struct tlc5940_led *led = container_of(ldev, struct tlc5940_led, ldev);

    spin_lock(&led->lock);
    led->brightness = brightness;
    if (led->id % 2){
        framebuffer_tx[(led->id -1)*3/2 +1] &= 0xf0;
        framebuffer_tx[(led->id -1)*3/2 +1] |= brightness >> 8;
        framebuffer_tx[(led->id -1)*3/2 +2] = brightness & 0xff;
    } else {
        framebuffer_tx[led->id *3/2] = brightness >> 4; //TODO
        framebuffer_tx[led->id *3/2 +1] &= 0x0f;
        framebuffer_tx[led->id *3/2 +1] |= (brightness << 4) & 0xf0 ;
    }
    spin_unlock(&led->lock);

    new_data = 1;
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
    struct pwm_device *pwm;
    int count = 0;
    int gpio = -EINVAL;
    int i = 0;
    int ret = 0;

    dev_info(dev, "Loading TI TLC5940 Driver...");

    if (!np) {
        printk(KERN_ERR "tlc5940: No platform data specified");
        return -ENODEV;
    }

    memset(framebuffer_tx, 0x00, TLC5940_FRAME_SIZE);
    tlcdev = devm_kzalloc(dev, sizeof(struct tlc5940_dev), GFP_KERNEL);
    if (!tlcdev)
        return -ENOMEM;

    mutex_init(&tlcdev->mlock);


    /* Set bits per word, shoud be 8 by default */
    spi->bits_per_word = TLC5940_SPI_BITS_PER_WORD;

    /* Set SPI master device transfer rate */
    /* TODO fails, kernel error */
    /* if (spi->max_speed_hz > TLC5940_SPI_MAX_SPEED) { */
    /*     dev_warn(dev, "spi max speed (%u) is too high, " */
    /*             "setting to default %u", tlcdev->spi->max_speed_hz, */
    /*             TLC5940_SPI_MAX_SPEED); */
    /*     spi->max_speed_hz = TLC5940_SPI_MAX_SPEED; */
    /* } */

    /*
     * Get the number of connected device from the device tree structure
     * and then scan and compare to actual number of connected devices.
     */
    if (of_match_device(of_match_ptr(tlc5940_of_match), dev)) {
        if (!of_property_read_u32(np, OF_MAX_CHAIN_SZ, &count)) {
            tlcdev->chain_sz = TLC5940_DEVICES;
            if (!tlcdev->chain_sz) {
                // Device is not connected at all
                dev_err(dev, "%u device(s)",
                        tlcdev->chain_sz);
                return -ENODEV;
            } else {
                dev_info(dev, "configured %u device(s)",
                        tlcdev->chain_sz);
            }
        }
    } else {
            return -ENODEV;
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
#ifdef DEBUG
            printk(KERN_INFO "xlat pin assigned: %d", gpio);
#endif
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
#ifdef DEBUG
            printk(KERN_INFO "blank pin assigned: %d", gpio);
#endif
        } else {
            dev_err(dev, "specified gpio pin for BLANK "
                    "is invalid");
            return -EINVAL;
        }
    }

    // Get PWM pin
    pwm = devm_of_pwm_get(dev, np, NULL);
    if (IS_ERR(pwm)) {
        ret = PTR_ERR(pwm);
        dev_err(dev, "failed to get GSCLK PWM pin: %d\n", ret);
        return ret;
    }

    //Configure PWM
    ret = pwm_config(pwm, TLC5940_GSCLK_DUTY_CYCLE_NS, TLC5940_GSCLK_PERIOD_NS);
    if (ret) {
        dev_err(
                dev,
                "failed to configure pwm with period %d, duty cycle %d: %d\n",
                TLC5940_GSCLK_PERIOD_NS,
                TLC5940_GSCLK_DUTY_CYCLE_NS,
                ret
               );
        return ret;
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
    tlcdev->pwm = pwm;
    tlcdev->bank_id = 0;
    new_data = 0;
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

        for (i = 0; i < TLC5940_LEDS; i++) {
            leddev = &item->leds[i];
            leddev->tlc = tlcdev;
            leddev->id = i;

            memset(leddev->name, 0, TLC5940_LED_NAME_SZ);
            sprintf(leddev->name, "led%u-%u", item->bank_id, leddev->id);

            leddev->brightness = LED_OFF;
            leddev->ldev.name = leddev->name;
            leddev->ldev.brightness = LED_OFF;
            leddev->ldev.max_brightness = 0xfff; //maybe add GS resolution here, 3ff for 10bit
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
    pwm_enable(pwm);
    spi_set_drvdata(spi, tlcdev);
    //hrtimer_delay = TLC5940_BLANK_PERIOD_NS * tlcdev->chain_sz;
    hrtimer_start(timer, ktime_set(1, 0), HRTIMER_MODE_REL);

    dev_info(dev, "SPI driver registered");

    /* 
     * sysfs stuff
     */

    /* Alloc buffer */
    gko_buffer = kmalloc(PAGE_SIZE, GFP_KERNEL);
    if (!gko_buffer)
        return -ENOMEM;
    gko_buffer_end = PAGE_SIZE;

    /* Alloc a device region */
    ret = alloc_chrdev_region(&gko_dev, 1, 1, DEVICE_NAME);
    if (ret != 0)          /* error */
        goto cdev_alloc_err;

    /* Registring */
    gko_cdev = cdev_alloc();
    if (!gko_cdev) 
        goto cdev_alloc_err;

    /* Init it! */
    cdev_init(gko_cdev, &fops); 

    /* Tell the kernel "hey, I'm exist" */
    ret = cdev_add(gko_cdev, gko_dev, 1);
    if (ret < 0) 
        goto cdev_add_out;

    /* class */
    gko_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gko_class)) {
        printk(KERN_ERR DEVICE_NAME " cant create class %s\n", CLASS_NAME);
        goto class_err;
    }

    /* device */
    gko_device = device_create(gko_class, NULL, gko_dev, NULL, DEVICE_NAME);
    if (IS_ERR(gko_device)) {
        printk(KERN_ERR DEVICE_NAME " cant create device %s\n", DEVICE_NAME);
        goto device_err;
    }

    /* device attribute on sysfs */
    ret = device_create_file(gko_device, &dev_attr_buffer_start);
    if (ret < 0) {
        printk(KERN_ERR DEVICE_NAME " cant create device attribute %s %s\n", 
                DEVICE_NAME, dev_attr_buffer_start.attr.name);
    }

    ret = device_create_file(gko_device, &dev_attr_buffer_end);
    if (ret < 0) {
        printk(KERN_ERR DEVICE_NAME " cant create device attribute %s %s\n", 
                DEVICE_NAME, dev_attr_buffer_start.attr.name);
    }

    return ret;

/* some errors to return */
device_err:
        device_destroy(gko_class, gko_dev);
class_err:
        class_unregister(gko_class);
        class_destroy(gko_class);
cdev_add_out:
        cdev_del(gko_cdev);
cdev_alloc_err:
        kfree(gko_buffer);
        return -EFAULT;
}

static int tlc5940_remove(struct spi_device *spi)
{
    struct tlc5940_dev *tlc = spi_get_drvdata(spi);
    struct pwm_device *pwm = tlc->pwm;
    struct device *dev = &tlc->spi->dev;
    struct work_struct *work = &tlc->work;
    struct hrtimer *timer = &tlc->timer;
    struct tlc5940_led *led;
    struct tlc5940_dev *next_node;
    struct tlc5940_dev *cur_node;
    int i = 0;

    pwm_disable(pwm);
    hrtimer_cancel(timer);
    cancel_work_sync(work);

    list_for_each_entry_safe(cur_node, next_node, &tlc->list, list) {
        for (i = 0; i < TLC5940_LEDS; i++) {
            led = &cur_node->leds[i];
            devm_led_classdev_unregister(dev, &led->ldev);
        }
        list_del(&cur_node->list);

        if (cur_node)
            devm_kfree(dev, cur_node);
    }

    /* Remove SYSFS Framebuffer file */
    /* kobject_put(kobj_ref); */
    /* void sysfs_remove_file ( struct kobject *  kobj, const struct attribute * attr); */

    /* 
     * sysfs stuff 
     */

    device_destroy(gko_class, gko_dev);
    class_unregister(gko_class);
    class_destroy(gko_class);
    cdev_del(gko_cdev);
    kfree(gko_buffer);

    dev_info(dev, "driver removed");
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
MODULE_AUTHOR("Philipp Sauerbrunn");
MODULE_DESCRIPTION("video driver for tlc5940");
MODULE_ALIAS("spi:" DRIVER_NAME);
