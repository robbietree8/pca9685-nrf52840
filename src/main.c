#include <zephyr/kernel.h>
#include <stdio.h>
#include "pca9685.h"


static struct k_timer led_timer;
PCA9685 pca9685;
static bool led_is_on = true;

static void period0_expire(struct k_timer *timer)
{
	if(led_is_on) {
		pca9685_setPWM(&pca9685, 0, 4096, 0);       // turns pin fully on
		led_is_on = false;
	} else {
		pca9685_setPWM(&pca9685, 0, 0, 4096);       // turns pin fully off
		led_is_on = true;
	}
}

/**
 * @brief working case
*/
void good_case()
{
	while(1){
		pca9685_setPWM(&pca9685, 0, 4096, 0);       // turns pin fully on
		k_sleep(K_SECONDS(1));
		pca9685_setPWM(&pca9685, 0, 0, 4096);       // turns pin fully off
		k_sleep(K_SECONDS(1));
	}
}

/**
 * @brief with following error
 * 
 * 00> [00:00:12.408,569] <err> i2c_nrfx_twim: Error on I2C line occurred for message 0
*/
void bad_case()
{
	k_timer_init(&led_timer, period0_expire, NULL);
	k_timer_start(&led_timer, K_SECONDS(1), K_SECONDS(1));

	while(1) {
		k_sleep(K_SECONDS(1));
	}
}

void main(void)
{
	printk("start");
	pca9685_init(&pca9685);

	for (uint8_t i=0; i<16; i++) {
		pca9685_setPWM(&pca9685, i, 4096, 0);
	}

	// good_case();
	bad_case();
}