#ifndef _PCA9685_H
#define _PCA9685_H

#include <stdint.h>
#include <stdbool.h>

// REGISTER ADDRESSES
#define PCA9685_MODE1		0x00    /**< Mode Register 1 */
#define PCA9685_MODE2 		0x01    /**< Mode Register 2 */
#define PCA9685_SUBADR1 	0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 	0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 	0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 	0x05 	/**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 	0x06  	/**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 	0x07  	/**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 	0x08 	/**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 	0x09 	/**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  	/**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  	/**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC 	/**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD 	/**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 	0xFE    /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 	0xFF    /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 	0x01  	/**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 		0x02  	/**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 		0x04  	/**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 		0x08  	/**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 	0x10  	/**< Low power mode. Oscillator off */
#define MODE1_AI 		0x20  	/**< Auto-Increment enabled */
#define MODE1_EXTCLK 	0x40  	/**< Use EXTCLK pin clock */
#define MODE1_RESTART 	0x80	/**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 	0x01 	/**< Active LOW output enable input */
#define MODE2_OUTNE_1   0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 	0x04 	/**< totem pole structure vs open-drain */
#define MODE2_OCH 		0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 	0x10  	/**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 	0x40		/**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 	25000000 	/**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 	3   	/**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 	255 	/**< maximum prescale value */

typedef enum PCA9685_clk {
  PCA9685_intclk = 0,
  PCA9685_extclk
} PCA9685_clk;

typedef struct PCA9685 {
	uint8_t _i2caddr;
	uint8_t _prescaler;
	uint16_t _pwmfeq;
	PCA9685_clk _clksrc;
	uint32_t _oscillator_freq;
} PCA9685;

void pca9685_getconfig(PCA9685 *pca9685_module);
void pca9685_init(PCA9685 *pca9685_module);
void pca9685_reset(PCA9685 *pca9685_module);
void pca9685_sleep(PCA9685 *pca9685_module);
void pca9685_wakeup(PCA9685 *pca9685_module);
void pca9685_setExtClk(PCA9685 *pca9685_module);
void pca9685_setPWMFreq(PCA9685 *pca9685_module);
void pca9685_setOutputMode(PCA9685 *pca9685_module, bool totempole);
uint8_t* pca9685_getPWM(PCA9685 *pca9685_module, uint8_t num);
void pca9685_setPWM(PCA9685 *pca9685_module, uint8_t num, uint16_t on, uint16_t off);
void pca9685_setPin(PCA9685 *pca9685_module, uint8_t num, uint16_t val, bool invert /*= false*/);
uint8_t pca9685_readPrescale(PCA9685 *pca9685_module);
void pca9685_writeMicroseconds(PCA9685 *pca9685_module, uint8_t num, uint16_t Microseconds);

void pca9685_setOscillatorFrequency(PCA9685 *pca9685_module, uint32_t freq);
uint32_t pca9685_getOscillatorFrequency(PCA9685 *pca9685_module);

#endif