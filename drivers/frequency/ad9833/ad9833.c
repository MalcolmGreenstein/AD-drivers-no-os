/**************************************************************************//**
*   @file   ad9833.c
*   @brief  Implementation of ad9833 Driver for Microblaze processor.
*   @author Lucian Sin (Lucian.Sin@analog.com)
*
*******************************************************************************
* Copyright 2013(c) Analog Devices, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived from this
*    software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
* EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*****************************************************************************/

#include <stdlib.h>
#include "ad9833.h"
#include "no_os_error.h"
#include "no_os_alloc.h"

/******************************************************************************/
/************************ Debug Configuration ********************************/
/******************************************************************************/

/**
 * @brief Debug control macro - Set to 1 to enable debug output, 0 to disable
 * 
 * Change this single define to enable/disable all debug output throughout
 * the driver. When enabled, debug messages will be printed to help with
 * troubleshooting and understanding driver operation.
 */
#define AD9833_DEBUG_ENABLE 0

#if AD9833_DEBUG_ENABLE
    #include <stdio.h>
    #define DEBUG_PRINT(fmt, ...) printf("[AD9833 DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...) ((void)0)
#endif

/******************************************************************************/
/************************ Constants and Global Variables ********************/
/******************************************************************************/

/**
 * @brief Phase calculation constant
 * 
 * This constant is used to convert phase values in degrees to the 12-bit
 * phase register format. The calculation is: phase_register = phase_degrees * phase_const
 * Value derived from: 4096 / 2π ≈ 651.8986469
 */
float phase_const = 651.8986469f;

/**
 * @brief Chip-specific information table
 * 
 * Contains device-specific parameters for different AD983x family members:
 * - mclk: Master clock frequency in Hz
 * - freq_const: Frequency calculation constant = 2^28 / mclk
 * 
 * The frequency register value is calculated as:
 * freq_register = frequency_hz * freq_const
 */
static const struct ad9833_chip_info chip_info[] = {
	[ID_AD9833] = {
		.mclk       = 25000000,    /* 25 MHz master clock */
		.freq_const = 10.7374182f, /* 2^28 / 25MHz */
	},
	[ID_AD9834]  = {
		.mclk       = 75000000,    /* 75 MHz master clock */
		.freq_const = 3.5791394f,  /* 2^28 / 75MHz */
	},
	[ID_AD9837] = {
		.mclk       = 16000000,    /* 16 MHz master clock */
		.freq_const = 16.777216f,  /* 2^28 / 16MHz */
	},
	[ID_AD9838]  = {
		.mclk       = 16000000,    /* 16 MHz master clock */
		.freq_const = 16.777216f,  /* 2^28 / 16MHz */
	}
};

/***************************************************************************//**
 * @brief Initialize the SPI communication with the device.
 *
 * This function performs complete initialization of the AD9833/AD9834 device:
 * 1. Allocates memory for device structure
 * 2. Configures GPIO pins for device control
 * 3. Initializes SPI interface
 * 4. Performs hardware reset sequence
 * 5. Sets default frequency and phase values
 *
 * @param device     - Pointer to device structure pointer (output)
 * @param init_param - Structure containing device initial parameters
 *
 * @return status - Result of the initialization procedure.
 *                  Example:  0 - if initialization was successful;
 *                           -1 - if initialization was unsuccessful.
*******************************************************************************/
int8_t ad9833_init(struct ad9833_dev **device,
		   struct ad9833_init_param init_param)
{
	struct ad9833_dev *dev;
	uint16_t spi_data = 0;
	int8_t status = -1;

	DEBUG_PRINT("Starting AD9833 initialization for device type: %d", init_param.act_device);

	/* Allocate memory for device structure */
	dev = (struct ad9833_dev *)no_os_malloc(sizeof(*dev));
	if (!dev) {
		DEBUG_PRINT("ERROR: Failed to allocate memory for device structure");
		return -1;
	}

	DEBUG_PRINT("Memory allocated successfully, setting up GPIO pins");

	/* Setup GPIO pins for device control */
	/* These pins control frequency/phase register selection and device states */
	status |= no_os_gpio_get(&dev->gpio_psel,
				 &init_param.gpio_psel);
	status |= no_os_gpio_get(&dev->gpio_fsel,
				 &init_param.gpio_fsel);
	status |= no_os_gpio_get(&dev->gpio_reset,
				 &init_param.gpio_reset);
	status |= no_os_gpio_get(&dev->gpio_sleep,
				 &init_param.gpio_sleep);

	/* Configure GPIO directions and initial states */
	/* All pins are set as outputs with initial low state */
	AD9834_PSEL_OUT;   /* Phase register select pin */
	AD9834_PSEL_LOW;
	AD9834_FSEL_OUT;   /* Frequency register select pin */
	AD9834_FSEL_LOW;
	AD9834_RESET_OUT;  /* Hardware reset pin */
	AD9834_RESET_LOW;
	AD9834_SLEEP_OUT;  /* Sleep control pin */
	AD9834_SLEEP_LOW;

	DEBUG_PRINT("GPIO pins configured successfully");

	/* Initialize device parameters */
	dev->act_device = init_param.act_device;
	dev->prog_method = 0;        /* Start with software programming method */
	dev->ctrl_reg_value = 0;     /* Clear control register shadow */
	dev->test_opbiten = 0;       /* Clear test bit flag */

	DEBUG_PRINT("Device parameters initialized, setting up SPI interface");

	/* Setup SPI interface for communication with device */
	status = no_os_spi_init(&dev->spi_desc,
				&init_param.spi_init);
	if (status != 0) {
		DEBUG_PRINT("ERROR: SPI initialization failed with status: %d", status);
		no_os_free(dev);
		return status;
	}

	DEBUG_PRINT("SPI interface initialized successfully, performing device reset");

	/* Perform hardware reset sequence */
	/* Set RESET bit in control register to reset internal state */
	spi_data |= AD9833_CTRLRESET;
	ad9833_tx_spi(dev, spi_data);
	DEBUG_PRINT("Reset command sent, waiting 10ms");
	
	no_os_mdelay(10);  /* Wait for reset to complete */
	
	/* Clear RESET bit to bring device out of reset */
	spi_data &= ~AD9833_CTRLRESET;
	ad9833_tx_spi(dev, spi_data);
	DEBUG_PRINT("Device brought out of reset");

	/* Initialize frequency and phase registers to zero */
	/* This ensures a known starting state for all registers */
	DEBUG_PRINT("Setting default frequency and phase values");
	ad9833_set_freq(dev, 0, 0);  /* Frequency register 0 = 0 Hz */
	ad9833_set_freq(dev, 1, 0);  /* Frequency register 1 = 0 Hz */
	ad9833_set_phase(dev, 0, 0); /* Phase register 0 = 0 degrees */
	ad9833_set_phase(dev, 1, 0); /* Phase register 1 = 0 degrees */

	/* Return initialized device structure */
	*device = dev;

	DEBUG_PRINT("AD9833 initialization completed successfully");
	return status;
}

/***************************************************************************//**
 * @brief Free the resources allocated by ad9833_init().
 *
 * This function performs complete cleanup of the AD9833 device:
 * 1. Releases SPI interface resources
 * 2. Releases all GPIO pin resources
 * 3. Frees allocated device structure memory
 *
 * @param dev - The device structure to be cleaned up
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad9833_remove(struct ad9833_dev *dev)
{
	int32_t ret;

	if (!dev) {
		DEBUG_PRINT("ERROR: Null device pointer passed to remove function");
		return -1;
	}

	DEBUG_PRINT("Starting AD9833 device cleanup");

	/* Release SPI interface resources */
	ret = no_os_spi_remove(dev->spi_desc);
	if (ret != 0) {
		DEBUG_PRINT("WARNING: SPI cleanup returned error: %d", ret);
	}

	/* Release GPIO pin resources */
	ret |= no_os_gpio_remove(dev->gpio_psel);
	ret |= no_os_gpio_remove(dev->gpio_fsel);
	ret |= no_os_gpio_remove(dev->gpio_reset);
	ret |= no_os_gpio_remove(dev->gpio_sleep);

	if (ret != 0) {
		DEBUG_PRINT("WARNING: GPIO cleanup returned error: %d", ret);
	}

	/* Free device structure memory */
	no_os_free(dev);

	DEBUG_PRINT("AD9833 device cleanup completed");
	return ret;
}

/**************************************************************************//**
 * @brief Transmits 16 bits on SPI.
 *
 * This function handles all SPI communication with the AD9833 device.
 * The 16-bit data is split into two 8-bit packets for transmission.
 * If SPI transmission fails, the function attempts a recovery by
 * performing a reset sequence.
 *
 * @param dev   - The device structure containing SPI interface
 * @param value - 16-bit data to be transmitted to device
 *
******************************************************************************/
void ad9833_tx_spi(struct ad9833_dev *dev,
		   int16_t value)
{
	uint16_t spi_data = 0;
	uint8_t tx_buffer[4]  = {0, 0, 0, 0}; /* SPI transmit buffer */

	DEBUG_PRINT("Transmitting SPI data: 0x%04X", (uint16_t)value);

	/* Split 16-bit value into two 8-bit packets */
	/* MSB first, then LSB (big-endian format) */
	tx_buffer[0] = (uint8_t)((value & 0x00ff00) >> 8);  /* High byte */
	tx_buffer[1] = (uint8_t)(value & 0x0000ff);         /* Low byte */

	/* Attempt SPI transmission */
	if (no_os_spi_write_and_read(dev->spi_desc, tx_buffer, 2) != 0) {
		DEBUG_PRINT("ERROR: SPI transmission failed, attempting recovery reset");
		
		/* SPI communication failed - attempt recovery */
		/* Perform reset sequence to restore communication */
		spi_data |= AD9833_CTRLRESET;
		ad9833_tx_spi(dev,
			      spi_data);
		no_os_mdelay(10);
		spi_data &= ~ AD9833_CTRLRESET;
		ad9833_tx_spi(dev,
			      spi_data);
		
		DEBUG_PRINT("Recovery reset sequence completed");
	} else {
		DEBUG_PRINT("SPI transmission successful");
	}
}

/**************************************************************************//**
 * @brief Selects the type of output.
 *
 * This function configures the output waveform type for the AD9833 family devices.
 * Different devices support different output modes:
 * 
 * AD9833/AD9837 devices support:
 * - Mode 0: Sinusoidal output (default)
 * - Mode 1: Triangle wave output
 * - Mode 2: DAC Data MSB/2 (square wave, half amplitude)
 * - Mode 3: DAC Data MSB (square wave, full amplitude)
 * 
 * AD9834/AD9838 devices support:
 * - Mode 0: Sinusoidal output (default)
 * - Mode 1: Triangle wave output (only if OPBITEN is not set)
 *
 * @param dev      - The device structure
 * @param out_mode - Output mode selection (0-3 depending on device)
 *
 * @return status - Result of mode selection
 *                  0  - Output mode successfully configured
 *                  -1 - Invalid mode for current device configuration
******************************************************************************/
int8_t ad9833_out_mode(struct ad9833_dev *dev,
		       uint8_t out_mode)
{
	uint16_t spi_data = 0;
	int8_t status = 0;

	DEBUG_PRINT("Setting output mode: %d for device type: %d", out_mode, dev->act_device);

	/* Handle AD9833 and AD9837 devices */
	if ((dev->act_device == ID_AD9833) || (dev->act_device == ID_AD9837)) {
		/* Clear output mode control bits while preserving other settings */
		spi_data = (dev->ctrl_reg_value & ~(AD9833_CTRLMODE    |
						    AD9833_CTRLOPBITEN |
						    AD9833_CTRLDIV2));
		switch (out_mode) {
		case 1:     /* Triangle wave output */
			spi_data += AD9833_CTRLMODE;
			DEBUG_PRINT("Configured for triangle wave output");
			break;
		case 2:     /* DAC Data MSB/2 (square wave, half amplitude) */
			spi_data += AD9833_CTRLOPBITEN;
			DEBUG_PRINT("Configured for square wave output (MSB/2)");
			break;
		case 3:     /* DAC Data MSB (square wave, full amplitude) */
			spi_data += AD9833_CTRLOPBITEN + AD9833_CTRLDIV2;
			DEBUG_PRINT("Configured for square wave output (MSB)");
			break;
		default:    /* Sinusoidal output (mode 0) */
			DEBUG_PRINT("Configured for sinusoidal output (default)");
			break;
		}
		ad9833_tx_spi(dev, spi_data);
		dev->ctrl_reg_value = spi_data;  /* Update shadow register */
	} else {
		/* Handle AD9834 and AD9838 devices */
		if ((dev->act_device == ID_AD9834) || (dev->act_device == ID_AD9838)) {
			/* Clear MODE bit while preserving other settings */
			spi_data = (dev->ctrl_reg_value & ~AD9833_CTRLMODE);
			/* Check if OPBITEN is currently set */
			dev->test_opbiten = dev->ctrl_reg_value & AD9833_CTRLOPBITEN;

			switch (out_mode) {
			case 1:     /* Triangle wave output */
				/* Triangle mode only available when OPBITEN is not set */
				if (dev->test_opbiten == 0) {
					spi_data += AD9833_CTRLMODE;
					DEBUG_PRINT("Configured for triangle wave output");
				} else {
					DEBUG_PRINT("ERROR: Triangle mode not available when OPBITEN is set");
					status = -1;
				}
				break;
			default:    /* Sinusoidal output (mode 0) */
				DEBUG_PRINT("Configured for sinusoidal output (default)");
				break;
			}
			ad9833_tx_spi(dev, spi_data);
			dev->ctrl_reg_value = spi_data;  /* Update shadow register */
		}
	}

	DEBUG_PRINT("Output mode configuration completed with status: %d", status);
	return status;
}

/**************************************************************************//**
 * @brief Enable / Disable the sleep function.
 *
 * This function controls power management for the AD9833 family devices.
 * Two programming methods are available:
 * 
 * Software method (prog_method = 0, all devices):
 * Uses SPI control register bits to manage power states:
 * - Mode 0: No power-down (normal operation)
 * - Mode 1: DAC powered down (clock runs, no output)
 * - Mode 2: Internal clock disabled (saves more power)
 * - Mode 3: DAC powered down AND internal clock disabled (maximum power savings)
 * 
 * Hardware method (prog_method = 1, AD9834 & AD9838 only):
 * Uses dedicated SLEEP pin for power control:
 * - Mode 0: SLEEP pin LOW (no power-down)
 * - Mode 1: SLEEP pin HIGH (DAC powered down)
 *
 * @param dev        - The device structure
 * @param sleep_mode - Power management mode (0-3 for software, 0-1 for hardware)
 *
******************************************************************************/
void ad9833_sleep_mode(struct ad9833_dev *dev,
		       uint8_t sleep_mode)
{
	uint16_t spi_data = 0;

	DEBUG_PRINT("Setting sleep mode: %d using method: %d", sleep_mode, dev->prog_method);

	/* Software programming method - use SPI control register */
	if (dev->prog_method == 0) {
		/* Clear existing sleep control bits while preserving other settings */
		spi_data = (dev->ctrl_reg_value & ~(AD9833_CTRLSLEEP12 | AD9833_CTRLSLEEP1));
		
		switch (sleep_mode) {
		case 1:     /* DAC powered down only */
			spi_data += AD9833_CTRLSLEEP12;
			DEBUG_PRINT("DAC powered down via software method");
			break;
		case 2:     /* Internal clock disabled only */
			spi_data += AD9833_CTRLSLEEP1;
			DEBUG_PRINT("Internal clock disabled via software method");
			break;
		case 3:     /* Both DAC powered down and internal clock disabled */
			spi_data += AD9833_CTRLSLEEP1 + AD9833_CTRLSLEEP12;
			DEBUG_PRINT("DAC and clock disabled via software method (maximum power savings)");
			break;
		default:    /* No power-down (normal operation) */
			DEBUG_PRINT("Normal operation (no power-down) via software method");
			break;
		}
		
		/* Send new control register value */
		ad9833_tx_spi(dev, spi_data);
		dev->ctrl_reg_value = spi_data;  /* Update shadow register */
	} else {
		/* Hardware programming method - use dedicated SLEEP pin */
		if (dev->prog_method == 1) {
			switch (sleep_mode) {
			case 0:     /* No power-down */
				AD9834_SLEEP_LOW;
				DEBUG_PRINT("Sleep pin set LOW (normal operation) via hardware method");
				break;
			case 1:     /* DAC powered down */
				AD9834_SLEEP_HIGH;
				DEBUG_PRINT("Sleep pin set HIGH (DAC powered down) via hardware method");
				break;
			default:    /* Invalid mode for hardware method */
				DEBUG_PRINT("WARNING: Invalid sleep mode %d for hardware method", sleep_mode);
				break;
			}
		}
	}

	DEBUG_PRINT("Sleep mode configuration completed");

}

/**************************************************************************//**
 * @brief Loads a frequency value in a register.
 *
 * @param dev             - The device structure.
 * @param register_number - Number of the register (0 / 1).
 * @param frequency_value - Frequency value.
 *
******************************************************************************/
void ad9833_set_freq(struct ad9833_dev *dev,
		     uint8_t register_number,
		     uint32_t frequency_value)
{
	uint32_t ul_freq_register;
	uint16_t i_freq_lsb, i_freq_msb;

	ul_freq_register = (uint32_t)(frequency_value *
				      chip_info[dev->act_device].freq_const);
	i_freq_lsb = (ul_freq_register & 0x0003FFF);
	i_freq_msb = ((ul_freq_register & 0xFFFC000) >> 14);
	dev->ctrl_reg_value |= AD9833_CTRLB28;
	ad9833_tx_spi(dev,
		      dev->ctrl_reg_value);
	if (register_number == 0) {
		ad9833_tx_spi(dev,
			      BIT_F0ADDRESS + i_freq_lsb);
		ad9833_tx_spi(dev,
			      BIT_F0ADDRESS + i_freq_msb);
	} else {
		ad9833_tx_spi(dev,
			      BIT_F1ADDRESS + i_freq_lsb);
		ad9833_tx_spi(dev,
			      BIT_F1ADDRESS + i_freq_msb);
	}
}

/**************************************************************************//**
 * @brief Loads a phase value in a register.
 *
 * @param dev             - The device structure.
 * @param register_number - Number of the register (0 / 1).
 * @param phase_value     - Phase value.
 *
******************************************************************************/
void ad9833_set_phase(struct ad9833_dev *dev,
		      uint8_t register_number,
		      float phase_value)
{
	uint16_t phase_calc;

	phase_calc = (uint16_t)(phase_value * phase_const);
	if (register_number == 0) {
		ad9833_tx_spi(dev,
			      BIT_P0ADDRESS + phase_calc);
	} else {
		ad9833_tx_spi(dev,
			      BIT_P1ADDRESS + phase_calc);
	}
}

/**************************************************************************//**
 * @brief Select the frequency register to be used.
 *
 * @param dev      - The device structure.
 * @param freq_reg - Number of frequency register. (0 / 1)
 *
******************************************************************************/
void ad9833_select_freq_reg(struct ad9833_dev *dev,
			    uint8_t freq_reg)
{
	uint16_t spi_data = 0;

	if (dev->prog_method == 0) {
		spi_data = (dev->ctrl_reg_value & ~AD9833_CTRLFSEL);
		// Select soft the working frequency register according to parameter
		if (freq_reg == 1) {
			spi_data += AD9833_CTRLFSEL;
		}
		ad9833_tx_spi(dev,
			      spi_data);
		dev->ctrl_reg_value = spi_data;
	} else {
		if (dev->prog_method == 1) {
			// Select hard the working frequency register according to parameter
			if (freq_reg == 1) {
				AD9834_FSEL_HIGH;
			} else {
				if (freq_reg == 0) {
					AD9834_FSEL_LOW;
				}
			}
		}
	}
}

/**************************************************************************//**
 * @brief Select the phase register to be used.
 *
 * @param dev       - The device structure.
 * @param phase_reg - Number of phase register. (0 / 1)
 *
******************************************************************************/
void ad9833_select_phase_reg(struct ad9833_dev *dev,
			     uint8_t phase_reg)
{
	uint16_t spi_data = 0;

	if (dev->prog_method == 0) {
		spi_data = (dev->ctrl_reg_value & ~AD9833_CTRLPSEL);
		// Select soft the working phase register according to parameter
		if (phase_reg == 1) {
			spi_data += AD9833_CTRLPSEL;
		}
		ad9833_tx_spi(dev,
			      spi_data);
		dev->ctrl_reg_value = spi_data;
	} else {
		if (dev->prog_method == 1) {
			// Select hard the working phase register according to parameter
			if (phase_reg == 1) {
				AD9834_PSEL_HIGH;
			} else {
				if (phase_reg == 0) {
					AD9834_PSEL_LOW;
				}
			}
		}
	}
}

/**************************************************************************//**
 * @brief Sets the programming method. (only for AD9834 & AD9838)
 *
 * @param dev   - The device structure.
 * @param value - soft or hard method. (0 / 1)
 *
******************************************************************************/
void ad9834_select_prog_method(struct ad9833_dev *dev,
			       uint8_t value)
{
	uint16_t spi_data = (dev->ctrl_reg_value & ~AD9834_CTRLPINSW);

	dev->prog_method = 0;        // software method
	if (value == 1) {
		spi_data += AD9834_CTRLPINSW;
		dev->prog_method = 1;    // hardware method
	}
	ad9833_tx_spi(dev,
		      spi_data);
	dev->ctrl_reg_value = spi_data;
}

/**************************************************************************//**
 * @brief Configures the control register for logic output.
 *        (only for AD9834 & AD9838)
 *
 * @param dev     - The device structure.
 * @param opbiten - Enables / disables the logic output.
 * @param signpib - Connects comparator / MSB to the SIGN BIT OUT pin.
 * @param div2    - MSB / MSB/2
 *
******************************************************************************/
void ad9834_logic_output(struct ad9833_dev *dev,
			 uint8_t opbiten,
			 uint8_t signpib,
			 uint8_t div2)
{
	uint16_t spi_data = 0;

	spi_data = (dev->ctrl_reg_value & ~(AD9833_CTRLOPBITEN |
					    AD9833_CTRLMODE    |
					    AD9834_CTRLSIGNPIB |
					    AD9833_CTRLDIV2));


	if (opbiten == 1) {
		spi_data |= AD9833_CTRLOPBITEN;

		if (signpib == 1) {
			spi_data |= AD9834_CTRLSIGNPIB;
		} else {
			if (signpib == 0) {
				spi_data &= ~AD9834_CTRLSIGNPIB;
			}
		}
		if (div2 == 1) {
			spi_data |= AD9833_CTRLDIV2;
		} else {
			if (div2 == 0) {
				spi_data &= ~AD9833_CTRLDIV2;
			}
		}
	} else {
		if (opbiten == 0) {
			spi_data &= ~AD9833_CTRLOPBITEN;
		}
	}
	ad9833_tx_spi(dev,
		      spi_data);
	dev->ctrl_reg_value = spi_data;
}


