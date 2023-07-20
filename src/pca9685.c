/*!
 *  @file Adafruit_PWMServoDriver.cpp
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "pca9685.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#include <stdio.h>
#include <string.h>

#define PCA_NODE DT_NODELABEL(pca9685)
static const struct i2c_dt_spec pca_i2c = I2C_DT_SPEC_GET(PCA_NODE);

uint8_t pca9685_txBuff[8];
uint8_t pca9685_rxBuff[8];
#define ENABLE_DEBUG_OUTPUT 1
static uint8_t pca9685_read8(PCA9685 *pca9685_module, uint8_t addr);
static void pca9685_write8(PCA9685 *pca9685_module, uint8_t addr, uint8_t d);


/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 */
void pca9685_getconfig(PCA9685 *pca9685_module){
	pca9685_module->_i2caddr = PCA9685_I2C_ADDRESS;
	pca9685_module->_prescaler = 0;
	pca9685_module->_pwmfeq = 1000;
	pca9685_module->_clksrc = PCA9685_intclk;
	pca9685_module->_oscillator_freq = FREQUENCY_OSCILLATOR;
}


/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 */
void pca9685_init(PCA9685 *pca9685_module) {
	pca9685_reset(pca9685_module);

	if (pca9685_module->_prescaler) {
		pca9685_setExtClk(pca9685_module);
	} else {
		// set a default frequency
		pca9685_setPWMFreq(pca9685_module);
	}
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void pca9685_reset(PCA9685 *pca9685_module) {
	pca9685_write8(pca9685_module, PCA9685_MODE1, MODE1_RESTART);
	k_sleep(K_MSEC(10));
}

/*!
 *  @brief  Puts board into sleep mode
 */
void pca9685_sleep(PCA9685 *pca9685_module) {
	uint8_t awake = pca9685_read8(pca9685_module, PCA9685_MODE1);
	uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
	pca9685_write8(pca9685_module, PCA9685_MODE1, sleep);
	k_sleep(K_MSEC(5)); // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep
 */
void pca9685_wakeup(PCA9685 *pca9685_module) {
  uint8_t sleep = pca9685_read8(pca9685_module, PCA9685_MODE1);
  uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
  pca9685_write8(pca9685_module, PCA9685_MODE1, wakeup);
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
void pca9685_setExtClk(PCA9685 *pca9685_module) {
  uint8_t oldmode = pca9685_read8(pca9685_module, PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  pca9685_write8(pca9685_module, PCA9685_MODE1, newmode); // go to sleep, turn off internal oscillator

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  pca9685_write8(pca9685_module, PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

  pca9685_write8(pca9685_module, PCA9685_PRESCALE, pca9685_module->_prescaler); // set the prescaler

  k_sleep(K_MSEC(5));
  // clear the SLEEP bit to start
  pca9685_write8(pca9685_module, PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
  printk("Mode now 0x%02X\n", pca9685_read8(pca9685_module, PCA9685_MODE1));
#endif
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
void pca9685_setPWMFreq(PCA9685 *pca9685_module) {
#ifdef ENABLE_DEBUG_OUTPUT
  printk("Attempting to set freq %ul\n", pca9685_module->_pwmfeq);
#endif
  // Range output modulation frequency is dependant on oscillator
  if (pca9685_module->_pwmfeq < 1)
	pca9685_module->_pwmfeq = 1;
  if (pca9685_module->_pwmfeq > 3500)
	pca9685_module->_pwmfeq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  float prescaleval = ((pca9685_module->_oscillator_freq / (pca9685_module->_pwmfeq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;
  pca9685_module->_prescaler = (uint8_t)prescaleval;


#ifdef ENABLE_DEBUG_OUTPUT
  printk("Final pre-scale: 0x%02X\n", pca9685_module->_prescaler);
#endif

  uint8_t oldmode = pca9685_read8(pca9685_module, PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  pca9685_write8(pca9685_module, PCA9685_MODE1, newmode);                             // go to sleep
  pca9685_write8(pca9685_module, PCA9685_PRESCALE, pca9685_module->_prescaler); // set the prescaler
  pca9685_write8(pca9685_module, PCA9685_MODE1, oldmode);
  k_sleep(K_MSEC(5));
  // This sets the MODE1 register to turn on auto increment.
  pca9685_write8(pca9685_module, PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
  printk("Mode now 0x%02X\n", pca9685_read8(pca9685_module, PCA9685_MODE1));
#endif
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
void pca9685_setOutputMode(PCA9685 *pca9685_module, bool totempole) {
  uint8_t oldmode = pca9685_read8(pca9685_module, PCA9685_MODE2);
  uint8_t newmode;
  if (totempole) {
    newmode = oldmode | MODE2_OUTDRV;
  } else {
    newmode = oldmode & ~MODE2_OUTDRV;
  }
  pca9685_write8(pca9685_module, PCA9685_MODE2, newmode);
#ifdef ENABLE_DEBUG_OUTPUT
  printk("Setting output mode: %s by setting MODE2 to 0x%02X\n", (totempole) ? "totempole" : "open drain", newmode);
#endif
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
uint8_t pca9685_readPrescale(PCA9685 *pca9685_module) {
  return pca9685_read8(pca9685_module, PCA9685_PRESCALE);
}

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @return requested PWM output value
 */
uint8_t* pca9685_getPWM(PCA9685 *pca9685_module, uint8_t num) {
  pca9685_txBuff[0] = (PCA9685_LED0_ON_L + 4 * num);
  memset(pca9685_rxBuff, 0, 4);

  i2c_write_dt(&pca_i2c,pca9685_txBuff,1);
  i2c_read_dt(&pca_i2c,pca9685_rxBuff,4);

  return pca9685_rxBuff;
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void pca9685_setPWM(PCA9685 *pca9685_module, uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT
  printk("Setting PWM %d : %d -> %d\n", num, on, off);
#endif
  pca9685_txBuff[0] = PCA9685_LED0_ON_L + 4 * num;
  pca9685_txBuff[1] = on;
  pca9685_txBuff[2] = on>>8;
  pca9685_txBuff[3] = off;
  pca9685_txBuff[4] = off>>8;

  int ret = i2c_write_dt(&pca_i2c,pca9685_txBuff,5);
  printk("ret = %d\n", ret);
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void pca9685_setPin(PCA9685 *pca9685_module, uint8_t num, uint16_t val, bool invert) {
  // Clamp value between 0 and 4095 inclusive.
  val = (val < (uint16_t)4095)? val: 4095;
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
    	pca9685_setPWM(pca9685_module, num, 4095, 0);
    } else if (val == 4095) {
      // Special value for signal fully off.
    	pca9685_setPWM(pca9685_module, num, 0, 4095);
    } else {
    	pca9685_setPWM(pca9685_module, num, 0, 4095 - val);
    }
  } else {
    if (val == 4095) {
      // Special value for signal fully on.
    	pca9685_setPWM(pca9685_module, num, 4095, 0);
    } else if (val == 0) {
      // Special value for signal fully off.
    	pca9685_setPWM(pca9685_module, num, 0, 4095);
    } else {
    	pca9685_setPWM(pca9685_module, num, 0, val);
    }
  }
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
void pca9685_writeMicroseconds(PCA9685 *pca9685_module, uint8_t num,
                                                uint16_t Microseconds) {
#ifdef ENABLE_DEBUG_OUTPUT
  printk("Setting PWM Via Microseconds on output %d : %d -> \n", num, Microseconds);
#endif

  double pulse = Microseconds;
  double pulselength;
  pulselength = 1000000; // 1,000,000 us per second

  // Read prescale
  uint16_t prescale = pca9685_readPrescale(pca9685_module);

#ifdef ENABLE_DEBUG_OUTPUT
  printk("0x%02X PCA9685 chip prescale\n", prescale);
#endif

  // Calculate the pulse for PWM based on Equation 1 from the datasheet section
  // 7.3.5
  prescale += 1;
  pulselength *= prescale;
  pulselength /= pca9685_module->_oscillator_freq;

#ifdef ENABLE_DEBUG_OUTPUT
  printk("%02f  us per bit\n", pulselength);
#endif

  pulse /= pulselength;

#ifdef ENABLE_DEBUG_OUTPUT
  printk("%02f  pulse for PWM\n", pulse);
#endif

  pca9685_setPWM(pca9685_module, num, 0, pulse);
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq
 * calculations
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
 * introspect)
 */
uint32_t pca9685_getOscillatorFrequency(PCA9685 *pca9685_module) {
  return pca9685_module->_oscillator_freq;
}

/*!
 *  @brief Setter for the internally tracked oscillator used for freq
 * calculations
 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void pca9685_setOscillatorFrequency(PCA9685 *pca9685_module, uint32_t freq) {
  pca9685_module->_oscillator_freq = freq;
}

/******************* Low level I2C interface */

uint8_t pca9685_read8(PCA9685 *pca9685_module, uint8_t addr) {
  pca9685_txBuff[0] = addr;
  memset(pca9685_rxBuff, 0, 1);

  i2c_write_dt(&pca_i2c,pca9685_txBuff,1);
  i2c_read_dt(&pca_i2c,pca9685_rxBuff,1);

  return pca9685_rxBuff[0];
}

void pca9685_write8(PCA9685 *pca9685_module, uint8_t addr, uint8_t d) {
  pca9685_txBuff[0] = addr;
  pca9685_txBuff[1] = d;

  i2c_write_dt(&pca_i2c,pca9685_txBuff,2);
}