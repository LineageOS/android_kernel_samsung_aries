/* arch/arm/mach-s5pv210/p1-vibrator.c
 *
 * Copyright (C) 2010 Samsung Electronics Co. Ltd. All Rights Reserved.
 * Author: Rom Lemarchand <rlemarchand@sta.samsung.com>
 *         Humberto Borba <kernel@humberos.com.br>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>

#include <asm/mach-types.h>

#include <../../../drivers/staging/android/timed_output.h>
#include <mach/gpio-p1.h>

#include <linux/device.h>
#include <linux/miscdevice.h>


#if defined(CONFIG_PHONE_P1_GSM)
#define PWM_PERIOD      44540
#define PWM_DUTY_MAX    44500
#define PWM_DUTY_MIN    22250
#elif defined(CONFIG_PHONE_P1_CDMA)
#define PWM_PERIOD      44640
#define PWM_DUTY_MAX    42408
#define PWM_DUTY_MIN    21204
#endif

#define VIB_EN          S5PV210_GPJ1(3)
#define VIB_PWM         S5PV210_GPD0(1)

#define MAX_TIMEOUT		5000 /* 5s */

static unsigned int multiplier = (PWM_DUTY_MAX - PWM_DUTY_MIN) / 100;
static unsigned int pwm_duty = 100;
static unsigned int pwm_duty_value = PWM_DUTY_MAX;
static struct regulator *regulator_motor;

static struct vibrator {
	struct wake_lock wklock;
	struct pwm_device *pwm_dev;
	struct hrtimer timer;
	struct work_struct work;
	spinlock_t lock;
	bool running;
	int timeout;
} vibdata;

static int p1_vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibdata.timer)) {
		ktime_t r = hrtimer_get_remaining(&vibdata.timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000000;
	} else
		return 0;
}

static void p1_vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long flags;

	cancel_work_sync(&vibdata.work);
	hrtimer_cancel(&vibdata.timer);

	if (value > 0 && value < 30)
		value = 30;

	vibdata.timeout = value;
	schedule_work(&vibdata.work);
	spin_lock_irqsave(&vibdata.lock, flags);

	if (value > 0) {
		if (value > MAX_TIMEOUT)
			value = MAX_TIMEOUT;

		hrtimer_start(&vibdata.timer,
			ns_to_ktime((u64)value * NSEC_PER_MSEC),
			HRTIMER_MODE_REL);

	}
	spin_unlock_irqrestore(&vibdata.lock, flags);
}

static ssize_t p1_vibrator_set_duty(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	sscanf(buf, "%d\n", &pwm_duty);

	if (pwm_duty < 0 || pwm_duty > 100) {
		pr_err("[VIB] %s :invalid interval [0-100]: %d",__FUNCTION__ ,pwm_duty);
		pwm_duty = 100;
	}

	if (pwm_duty >= 0 && pwm_duty <= 100)
		pwm_duty_value = (pwm_duty * multiplier) + PWM_DUTY_MIN;

	printk(KERN_DEBUG "[VIB] %s pwm_duty_value: %d\n",__FUNCTION__ ,pwm_duty_value);
	return size;
}

static ssize_t p1_vibrator_show_duty(struct device *dev,
					struct device_attribute *attr,
					const char *buf)
{
	return sprintf(buf, "%d", pwm_duty);
}

static DEVICE_ATTR(pwm_duty, S_IRUGO | S_IWUGO, p1_vibrator_show_duty, p1_vibrator_set_duty);

static struct attribute *pwm_duty_attributes[] = {
	&dev_attr_pwm_duty,
	NULL
};

static struct attribute_group pwm_duty_group = {
	.attrs = pwm_duty_attributes,
};

static struct miscdevice pwm_duty_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pwm_duty",
};

static struct timed_output_dev to_dev = {
	.name		= "vibrator",
	.get_time	= p1_vibrator_get_time,
	.enable		= p1_vibrator_enable,
};

static enum hrtimer_restart p1_vibrator_timer_func(struct hrtimer *timer)
{
	vibdata.timeout = 0;
	schedule_work(&vibdata.work);
	return HRTIMER_NORESTART;
}

static void p1_vibrator_work(struct work_struct *work)
{

	if (0 == vibdata.timeout) {

		if (!vibdata.running)
			return;

		pwm_disable(vibdata.pwm_dev);
		s3c_gpio_cfgpin(VIB_PWM, S3C_GPIO_OUTPUT);
		if (regulator_is_enabled(regulator_motor))
			regulator_force_disable(regulator_motor);
		gpio_direction_output(VIB_EN, GPIO_LEVEL_LOW);
		vibdata.running = false;
	} else {
		if (vibdata.running)
			return;

		printk(KERN_DEBUG "[VIB] %s timeout: %dms\n",
			__FUNCTION__ ,vibdata.timeout);

		s3c_gpio_cfgpin(VIB_PWM, S3C_GPIO_SFN(2));
		if (!regulator_is_enabled(regulator_motor))
			regulator_enable(regulator_motor);
		pwm_config(vibdata.pwm_dev, pwm_duty_value, PWM_PERIOD);
		pwm_enable(vibdata.pwm_dev);
		gpio_direction_output(VIB_EN, GPIO_LEVEL_HIGH);
		vibdata.running = true;
	}
}

static int __init p1_init_vibrator(void)
{
	int ret = 0;

#ifdef CONFIG_MACH_P1
	if (!machine_is_p1())
		return 0;
#endif
	regulator_motor = regulator_get(NULL, "vcc_motor");
	if (IS_ERR_OR_NULL(regulator_motor)) {
		pr_err("failed to get motor regulator");
		return -EINVAL;
	}

	ret = gpio_request(VIB_EN, "VIB_EN");
	if (ret < 0)
		return ret;

	ret = gpio_request(VIB_PWM, "VIB_PWM");
	if (ret < 0)
		return ret;

	s3c_gpio_cfgpin(VIB_EN, 1);
	gpio_set_value(VIB_EN, 0);
	s3c_gpio_setpull(VIB_EN, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(VIB_PWM, S3C_GPIO_OUTPUT);
	gpio_set_value(VIB_PWM, 0);
	s3c_gpio_setpull(VIB_PWM, S3C_GPIO_PULL_NONE);
	gpio_free(VIB_PWM);

	vibdata.pwm_dev = pwm_request(1, "vibrator-pwm");
	if (IS_ERR(vibdata.pwm_dev)) {
		ret = PTR_ERR(vibdata.pwm_dev);
		goto err_pwm_req;
	}

	hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibdata.timer.function = p1_vibrator_timer_func;
	INIT_WORK(&vibdata.work, p1_vibrator_work);
	spin_lock_init(&vibdata.lock);

	ret = timed_output_dev_register(&to_dev);
	if (ret < 0)
		goto err_to_dev_reg;

	if (misc_register(&pwm_duty_device))
		printk("%s misc_register(%s) failed\n", __FUNCTION__, pwm_duty_device.name);
	else {
		if (sysfs_create_group(&pwm_duty_device.this_device->kobj,&pwm_duty_group))
			pr_err("failed to create sysfs group for device vibrator-pwm");
	}

	return 0;

err_to_dev_reg:
	pwm_free(vibdata.pwm_dev);
err_pwm_req:
	gpio_free(VIB_EN);
	return ret;
}

device_initcall(p1_init_vibrator);
