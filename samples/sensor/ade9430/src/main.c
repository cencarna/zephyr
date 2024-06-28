#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

K_SEM_DEFINE(sem, 0, 1);
LOG_MODULE_DECLARE(ADE9430);

static const struct device *dev = DEVICE_DT_GET_ONE(adi_ade9430);

#if CONFIG_SHELL
int main(void)
{
	if (dev == NULL) {
		printk("Failed to get binding device\n");
	}
 
	while(1) {
		k_msleep(100);
	}
	return 0;
}
#else

#define DELAY_MS 1000

void sensor_task(void)
{
	struct sensor_value vrms, irms, watt, temp;
	uint64_t count = 0;

	printk("1-phase polling VRMS, IRMS, WATT, TEMP every %d ms.\n", DELAY_MS);
	for (;;) {
		sensor_sample_fetch(dev);

		k_msleep(DELAY_MS);

		sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &vrms);
		sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &irms);
		sensor_channel_get(dev, SENSOR_CHAN_POWER, &watt);
		sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);

		printk("[%llu] vrms = %f, irms = %f, watt = %f, temp = %f \n",
		       count,
		       sensor_value_to_double(&vrms),
		       sensor_value_to_double(&irms),
		       sensor_value_to_double(&watt),
		       sensor_value_to_double(&temp));

		count++;
	}
}

int main(void)
{
	if (dev == NULL) {
		printk("Failed to get binding device\n");
	}

	sensor_task();

	return 0;
}

#endif

