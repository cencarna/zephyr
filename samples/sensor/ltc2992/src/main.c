#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

K_SEM_DEFINE(sem, 0, 1);
LOG_MODULE_DECLARE(LTC2992);

#define SENSOR_ATTR_LTC2992_SENSE_CHAN		(SENSOR_ATTR_PRIV_START + 1)
#define SENSOR_ATTR_LTC2992_MAX_DATA		(SENSOR_ATTR_PRIV_START + 2)
#define SENSOR_ATTR_LTC2992_MIN_DATA		(SENSOR_ATTR_PRIV_START + 3)

int main(void)
{
	int i;
	struct sensor_value vbus, current, power;
	struct sensor_value selected_channel;
	const struct device *dev = DEVICE_DT_GET_ONE(adi_ltc2992);

	if (!dev) {
		printk("Failed to get binding device\n");
		return -1;
	}

	while (1) {
		sensor_sample_fetch(dev);

		for (i = 1; i <= 2; i++) {
			selected_channel.val1 = i;
			sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_LTC2992_SENSE_CHAN,
					&selected_channel);

			sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &vbus);
			sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &current);
			sensor_channel_get(dev, SENSOR_CHAN_POWER, &power);

			printk("SENSE%d: vbus = %d.%06d V, current = %d.%06d A, power = %d.%06d\n",
			       i, vbus.val1, vbus.val2, current.val1, current.val2, power.val1,
			       power.val2);
		}

		printk("\n");
		k_msleep(1000);
	}
}
