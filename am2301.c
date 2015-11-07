/*
 * am2301.c
 *
 * AM2301 (DHT21) Relative humidity and temperature [C].
 *
 * Usage:
 *
 * insmod ./am2301.ko gpioArray = 1,2,3		# using GPIO 1, 2, and 3
 * cat /proc/am2301
 *
 * Example output when measurement has been made successfully:
 *    GPIO 1, 43.2 RH, 25.2 C, ok
 *    GPIO 2, 43.3 RH, 25.1 C, ok
 *    GPIO 3, 43.1 RH, 25.2 C, ok
 * or if there are errors
 *    43.2 RH, 25.1 C, error, checkum
 * or (no data, wrong pin or broken module)
 *    0.0 RH, 0.0 C, error, no data
 *
 * The module does not use any GPIO pin by default and exits if none given
 * GPIO24 pin (Raspberry Pi 2 B physical pin 18) so GPIO numbers are used
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/irqflags.h>
#include <linux/spinlock.h>

// GPIO pin array for input and output
// note that physical connector pin number is different
static int gpioArray[26];
static int gpioCount;

// last measurement time, next measurement can not be done
// until measurement period has passed (see MEASUREMENT_PERIOD)
static long last_time = 0;

// minimum measurement period 2.0 seconds according to specifications
// driver will sleep until this period has passed
#define MEASUREMENT_PERIOD 2000

// if no data, loop max count and exit
#define MAX_LOOP_COUNT 10000

// GPIO high (3.3V or 5V)
#define HIGH 1

// GPIO low (0V)
#define LOW 0

/*
 * Wait until GPIO changes to state
 *
 * returns 0 if changed successfully, 1 if no change
 */
int wait_for_gpio(int state, int gpio)
{
	int i;

	/// MAX_LOOP_COUNT prevents driver from crashing when there is no data
	for (i=0; i<MAX_LOOP_COUNT; i++) {
		if (gpio_get_value(gpio) == state) {
			return 0;
		}
	}

	// always fail if max loop count hit
	return 1;
}

static int am2301_show(struct seq_file *m, void *v)
{
	int i, j, res;
	char status[20];
	int data[5];
	int rh,t;

	ktime_t start, stop;
	int nodata_error = 0; // 1 if no data from AM2301

	for (i = 0; i < gpioCount; i++)
	{
		gpio_direction_output(gpioArray[i], 1);
	}

	udelay(2000);

	/*
	 * Set pin low and wait for at least 800 us.
	 * Set it high again, then wait for the sensor to put out a low pulse.
	 */
	for (i = 0; i < gpioCount; i++)
	{
		gpio_set_value(gpioArray[i], 0);
	}

	udelay(1000);

	// Disable interrupts during measurement, this is critical for
	// reliable time measurements when receiving high speed data.
	// All Kernel interrupts are disabled for about 2-3 milliseconds.


	for (i = 0; i < gpioCount; i++)
	{
		int databyte = 0L;
		int d = 0, b = 0;
		local_irq_disable();

		gpio_set_value(gpioArray[i], 1);

		gpio_direction_input(gpioArray[i]);

		nodata_error |= wait_for_gpio(LOW, gpioArray[i]);
		nodata_error |= wait_for_gpio(HIGH, gpioArray[i]);
		nodata_error |= wait_for_gpio(LOW, gpioArray[i]);


		for ( j=0; j<40; j++) {
			wait_for_gpio(HIGH, gpioArray[i]);

			// now measure length of high state in nanoseconds

			start = ktime_get_real();
			wait_for_gpio(LOW, gpioArray[i]);
			stop = ktime_get_real();
			res = (int)(stop.tv64 - start.tv64);

			// 1st data bit is shorter, why ?

			if ((j==0 && res > 29000) || ( j!=0 && res > 40000)) {
				databyte|=0x01;
			}
			d++;
			if (d==8) {
				d=0;
				data[b]=databyte;
				b=b+1;
				databyte=0x00;
			}
			databyte<<=1;
		}

		local_irq_enable();
		wait_for_gpio(HIGH, gpioArray[i]);

		gpio_direction_output(gpioArray[i], 1);

		if (((data[0] + data[1] + data[2] + data[3]) & 0xff) == data[4]) {
			strcpy(status, "ok");
		} else  {
			strcpy(status, "error, checksum");
		}
		if (nodata_error) {
			strcpy(status, "error, no data");
		}

		// check flag for negative temperatures
		rh = ((data[0]<<8) + data[1] );

		if (data[2] & 0x80) {
			data[2] = data[2] & 0x7f;
			t  = (data[2]<<8) + data[3];
			seq_printf(m, "GPIO %d, %d.%d RH, -%d.%d C, %s\n", gpioArray[i], rh/10, rh%10, t/10, t%10, status);
		} else {
			t = (data[2]<<8) + data[3];
			seq_printf(m, "GPIO %d, %d.%d RH, %d.%d C, %s\n", gpioArray[i], rh/10, rh%10, t/10, t%10, status);
		}

	}
	return 0;
}



static int am2301_open(struct inode *inode, struct  file *file)
{
	long time_since_last_read = jiffies_to_msecs(jiffies - last_time);

	if (time_since_last_read >= 0 && time_since_last_read < MEASUREMENT_PERIOD) {
		msleep(MEASUREMENT_PERIOD - time_since_last_read);
	}
	last_time = jiffies;

	return single_open(file, am2301_show, NULL);
}


static const struct file_operations am2301_fops =
{
	.owner = THIS_MODULE,
	.open = am2301_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init am2301_init(void)
{
	int i, ret;

	printk(KERN_INFO "Initializing am2301 (dht21)\n");
	for (i = 0; i < gpioCount; i++)
	{
		printk(KERN_INFO "Using GPIO%d\n", gpioArray[i]);

		ret = gpio_request_one(gpioArray[i], GPIOF_OUT_INIT_HIGH, "AM2301");

		if (ret != 0) {
			printk(KERN_ERR "Unable to request GPIO, err: %d\n", ret);
			return ret;
		}
	}
	proc_create_data("am2301", 0, NULL, &am2301_fops, NULL);
	return 0;
}

static void __exit am2301_exit(void)
{
	int i;
	for (i = 0; i < gpioCount; i++)
	{
		(void) gpio_direction_output(gpioArray[i], 1);
		gpio_free(gpioArray[i]);
	}
	remove_proc_entry("am2301", NULL);
	printk(KERN_INFO "am2301 exit module\n");
}




module_init(am2301_init);
module_exit(am2301_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kari Aarnio, modified by Casimir Blonski");
MODULE_DESCRIPTION("AM2301 (DHT21) driver");

module_param_array(gpioArray, int, &gpioCount, S_IRUGO);

MODULE_PARM_DESC(gpioArray, "Pin number array for data input and output (GPIO24 Raspberry Model B physical pin #18)");
