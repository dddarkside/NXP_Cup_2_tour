extern "C"
{
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

#include "Modules/mSpi.h"
#include "Modules/mDac.h"
#include "Modules/mAccelMagneto.h"
#include "Modules/mGyro.h"
#include "Modules/mTimer.h"
#include "Modules/mCpu.h"
#include "Modules/mSwitch.h"
#include "Modules/mLeds.h"
#include "Modules/mAd.h"
#include "Modules/mDelay.h"
#include "Modules/mRS232.h"
#include "Modules/mVL6180x.h"
}

void setup_mk()
{
	//--------------------------------------------------------------------
		// Device and card setup
		//--------------------------------------------------------------------
		// PLL Config --> CPU 100MHz, bus and periph 50MH z
		mCpu_Setup();

		// Config and start switches and pushers
		mSwitch_Setup();
		mSwitch_Open();

		// Config and start of LEDs
		mLeds_Setup();
		mLeds_Open();

		// Config and start of ADC
		mAd_Setup();
		mAd_Open();

		// Config and start of SPI
		mSpi_Setup();
		mSpi_Open();

		// Config and start non-blocking delay by PIT
		mDelay_Setup();
		mDelay_Open();

		// Timer Config for Speed Measurement and PWM Outputs for Servos
		mTimer_Setup();
		mTimer_Open();

		// Setup FXOS8700CQ
		mAccelMagneto_Setup();
		mAccelMagneto_Open();

		// Setup FXAS21002C
		mGyro_Setup();
		mGyro_Open();

		// Config and start of the DAC0 used to drive the driver LED lighting
		mDac_Setup();
		mDac_Open();

		// Setup and start of motor and servo PWM controls and speed measurement
		mTimer_Setup();
		mTimer_Open();

		// Enable IRQ at the CPU -> Primask
		__enable_irq();

		// UART 4 monitoring image
		mRs232_Setup();
		mRs232_Open();
}
