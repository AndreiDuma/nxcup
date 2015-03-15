
#include "mbed.h"

/** @file test.h*/

/**
 * @defgroup FRDM-TFC_API FRDM-TFC_API
 *
 * @{
 */


/**

@addtogroup FRDM-TFC_API

@{

Resources used by the TFC Library\n

I/O:\n
-------------------------------------------------------------------------------------------------\n

	PTB0   (Servo Channel 0 - TPM1)\n
	PTB1   (Servo Channel 1 - TPM1)\n
\n
	PTB8   (Battery LED0)\n
	PTB9   (Battery LED1)\n
	PTB10  (Battery LED2)\n
	PTB11  (Battery LED3)\n
\n
	PTD7   (Camera SI)\n
	PTE0   (Camera CLK)\n
	PTD5   (Camera A0  - ADC_SE6b)\n
	PTD6   (Camera A1 - ADC_SE7b)\n
\n
	PTE2    DIP Switch 0\n
	PTE3    DIP Switch 1\n
	PTE4    DIP Switch 2\n
	PTE5    DIP Switch 3\n

	PTC13   Pushbutton SW1\n
	PTC17   Pushbutton SW2\n

	PTC3    H-Bridge A - 1 FTM0_CH3\n
	PTC4    H-Bridge A - 2 FTM0_CH4\n
	PTC1    H-Bridge B - 1 FTM0_CH1\n
	PTC2    H-Bridge B - 2 FTM0_CH2\n

	PTE21   H-Bridge Enable\n
	PTE20   H-Bridge Fault\n

	PTE23   H-Bridge A - IFB\n
	PTE22   H-Bridge B - IFB\n

	}
*/




#define TFC_HBRIDGE_EN_LOC          (uint32_t)(1<<21)
#define TFC_HBRIDGE_FAULT_LOC       (uint32_t)(1<<20)

#define TFC_HBRIDGE_ENABLE          PTE->PSOR = TFC_HBRIDGE_EN_LOC
#define TFC_HBRIDGE_DISABLE         PTE->PCOR = TFC_HBRIDGE_EN_LOC

#define TFC_DIP_SWITCH0_LOC         ((uint32_t)(1<<2))
#define TFC_DIP_SWITCH1_LOC         ((uint32_t)(1<<3))
#define TFC_DIP_SWITCH2_LOC         ((uint32_t)(1<<4))
#define TFC_DIP_SWITCH3_LOC         ((uint32_t)(1<<5))

#define TFC_PUSH_BUTT0N0_LOC        ((uint32_t)(1<<13))
#define TFC_PUSH_BUTT0N1_LOC        ((uint32_t)(1<<17))

#define TFC_BAT_LED0_LOC            ((uint32_t)(1<<11))
#define TFC_BAT_LED1_LOC            ((uint32_t)(1<<10))
#define TFC_BAT_LED2_LOC            ((uint32_t)(1<<9))
#define TFC_BAT_LED3_LOC            ((uint32_t)(1<<8))

#define TAOS_CLK_HIGH  PTE->PSOR = (1<<1)
#define TAOS_CLK_LOW   PTE->PCOR = (1<<1)
#define TAOS_SI_HIGH   PTD->PSOR = (1<<7)
#define TAOS_SI_LOW    PTD->PCOR = (1<<7)


/**

@addtogroup FRDM-TFC_API
@{
*/

/**Macro to turn on LED 0 in the battery indicator array*/
#define TFC_BAT_LED0_ON             PTB->PSOR = TFC_BAT_LED0_LOC
/** Macro to turn on LED 1 in the battery indicator array*/
#define TFC_BAT_LED1_ON             PTB->PSOR = TFC_BAT_LED1_LOC
/** Macro to turn on LED 2 in the battery indicator array*/
#define TFC_BAT_LED2_ON             PTB->PSOR = TFC_BAT_LED2_LOC
/** Macro to turn on LED 3 in the battery indicator array*/
#define TFC_BAT_LED3_ON             PTB->PSOR = TFC_BAT_LED3_LOC


/** Macro to turn off LED 0 in the battery indicator array*/
#define TFC_BAT_LED0_OFF            PTB->PCOR = TFC_BAT_LED0_LOC
/** Macro to turn off LED 1 in the battery indicator array*/
#define TFC_BAT_LED1_OFF            PTB->PCOR = TFC_BAT_LED1_LOC
/** Macro to turn off LED 2 in the battery indicator array*/
#define TFC_BAT_LED2_OFF            PTB->PCOR = TFC_BAT_LED2_LOC
/** Macro to turn off LED 3 in the battery indicator array*/
#define TFC_BAT_LED3_OFF            PTB->PCOR = TFC_BAT_LED3_LOC


/** Macro to toggle LED 0 in the battery indicator array*/
#define TFC_BAT_LED0_TOGGLE         PTB->PTOR = TFC_BAT_LED0_LOC
/** Macro to toggle LED 1 in the battery indicator array*/
#define TFC_BAT_LED1_TOGGLE         PTB->PTOR = TFC_BAT_LED1_LOC
/** Macro to toggle LED 2 in the battery indicator array*/
#define TFC_BAT_LED2_TOGGLE         PTB->PTOR = TFC_BAT_LED2_LOC
/** Macro to toggle LED 3 in the battery indicator array*/
#define TFC_BAT_LED3_TOGGLE         PTB->PTOR = TFC_BAT_LED3_LOC


/** Macro to read the state of the pushbutton SW1*/
#define TFC_PUSH_BUTTON_0_PRESSED   ((PTC->PDIR&TFC_PUSH_BUTT0N0_LOC)>0)
/** Macro to read the state of the pushbutton SW1*/
#define TFC_PUSH_BUTTON_1_PRESSED   ((PTC->PDIR&TFC_PUSH_BUTT0N1_LOC)>0)

/** Macro to read the state of switch 0 in the 4 position DIP switch*/
#define TFC_DIP_SWITCH_0_ON         ((TFC_GetDIP_Switch()&0x01)>0)

/** Macro to read the state of switch 1 in the 4 position DIP switch*/
#define TFC_DIP_SWITCH_1_ON         ((TFC_GetDIP_Switch()&0x02)>0)

/** Macro to read the state of switch 2 in the 4 position DIP switch*/
#define TFC_DIP_SWITCH_2_ON         ((TFC_GetDIP_Switch()&0x04)>0)

/** Macro to read the state of switch 3 in the 4 position DIP switch*/
#define TFC_DIP_SWITCH_3_ON         ((TFC_GetDIP_Switch()&0x08)>0)

#define FTM1_CLK_PRESCALE                                                                  6   // Prescale Selector value - see comments in Status Control (SC) section for more details
#define SERVO_DEFAULT_PERIOD                                                   (float)(.010)   // Desired Frequency of PWM Signal - Here 50Hz => 20ms period
#define TAOS_CLK_COUNT                                                                   200   // Number of cycles for CLK Signal on camera

// use these to dial in servo steering to your particular servo
#define SERVO_MIN_PULSE_WIDTH_DEFAULT                                          (float)(.00060)  // The number here should be be *pulse width* in seconds to move servo to its left limit
#define SERVO_MAX_PULSE_WIDTH_DEFAULT                                          (float)(.00124)   // The number here should be be *pulse width* in seconds to move servo to its left limit


#define FTM0_CLOCK                                             (SystemCoreClock/2)
#define FTM0_CLK_PRESCALE                                      (0)  // Prescale Selector value - see comments in Status Control (SC) section for more details
#define FTM0_DEFAULT_SWITCHING_FREQUENCY                      (4000.0)

#define ADC_MAX_CODE    (4095)

#define TAOS_CLK_HIGH  PTE->PSOR = (1<<1)
#define TAOS_CLK_LOW   PTE->PCOR = (1<<1)
#define TAOS_SI_HIGH   PTD->PSOR = (1<<7)
#define TAOS_SI_LOW    PTD->PCOR = (1<<7)

#define ADC_STATE_INIT                          0
#define ADC_STATE_CAPTURE_POT_0                 1
#define ADC_STATE_CAPTURE_POT_1                 2
#define ADC_STATE_CAPTURE_BATTERY_LEVEL         3
#define ADC_STATE_CAPTURE_LINE_SCAN             4


#define TFC_POT_0_ADC_CHANNEL       13
#define TFC_POT_1_ADC_CHANNEL       12
#define TFC_BAT_SENSE_CHANNEL       4
#define TFC_LINESCAN0_ADC_CHANNEL   6
#define TFC_LINESCAN1_ADC_CHANNEL   7


#define ADC0_irq_no   57
#define ADC1_irq_no   58

#define ADC0_CHANA    19                                    // set to desired ADC0 channel trigger A    
#define ADC0_CHANB    20                                    // set to desired ADC0 channel trigger B    

#define ADC1_CHANA    20                                    // set to desired ADC1 channel trigger A  20 defaults to potentiometer in TWRK60     
#define ADC1_CHANB    20                                    // set to desired ADC1 channel trigger B

#define ADC0_DLYA     0x2000                                // ADC0 trigger A delay 
#define ADC0_DLYB     0x4000                                // ADC0 trigger B delay 
#define ADC1_DLYA     0x6000                                // ADC1 trigger A delay
#define ADC1_DLYB     0x7fff                                // ADC1 trigger B delay 


#define ADC0A_DONE   0x01
#define ADC0B_DONE   0x02
#define ADC1A_DONE   0x04
#define ADC1B_DONE   0x08


// Bit shifting of bitfiled is already taken into account so
// bitfiled values are always represented as relative to their position.

/************************* #Defines ******************************************/

#define A                 0x0
#define B                 0x1

/////// NOTE: the following defines relate to the ADC register definitions
/////// and the content follows the reference manual, using the same symbols.


//// ADCSC1 (register)

// Conversion Complete (COCO) mask
#define COCO_COMPLETE     ADC_SC1_COCO_MASK
#define COCO_NOT          0x00

// ADC interrupts: enabled, or disabled.
#define AIEN_ON           ADC_SC1_AIEN_MASK
#define AIEN_OFF          0x00

// Differential or Single ended ADC input
#define DIFF_SINGLE       0x00
#define DIFF_DIFFERENTIAL ADC_SC1_DIFF_MASK

//// ADCCFG1

// Power setting of ADC
#define ADLPC_LOW         ADC_CFG1_ADLPC_MASK
#define ADLPC_NORMAL      0x00

// Clock divisor
#define ADIV_1            0x00
#define ADIV_2            0x01
#define ADIV_4            0x02
#define ADIV_8            0x03

// Long samle time, or Short sample time
#define ADLSMP_LONG       ADC_CFG1_ADLSMP_MASK
#define ADLSMP_SHORT      0x00

// How many bits for the conversion?  8, 12, 10, or 16 (single ended).
#define MODE_8            0x00
#define MODE_12           0x01
#define MODE_10           0x02
#define MODE_16           0x03



// ADC Input Clock Source choice? Bus clock, Bus clock/2, "altclk", or the
//                                ADC's own asynchronous clock for less noise
#define ADICLK_BUS        0x00
#define ADICLK_BUS_2      0x01
#define ADICLK_ALTCLK     0x02
#define ADICLK_ADACK      0x03

//// ADCCFG2

// Select between B or A channels
#define MUXSEL_ADCB       ADC_CFG2_MUXSEL_MASK
#define MUXSEL_ADCA       0x00

// Ansync clock output enable: enable, or disable the output of it
#define ADACKEN_ENABLED   ADC_CFG2_ADACKEN_MASK
#define ADACKEN_DISABLED  0x00

// High speed or low speed conversion mode
#define ADHSC_HISPEED     ADC_CFG2_ADHSC_MASK
#define ADHSC_NORMAL      0x00

// Long Sample Time selector: 20, 12, 6, or 2 extra clocks for a longer sample time
#define ADLSTS_20          0x00
#define ADLSTS_12          0x01
#define ADLSTS_6           0x02
#define ADLSTS_2           0x03

////ADCSC2

// Read-only status bit indicating conversion status
#define ADACT_ACTIVE       ADC_SC2_ADACT_MASK
#define ADACT_INACTIVE     0x00

// Trigger for starting conversion: Hardware trigger, or software trigger.
// For using PDB, the Hardware trigger option is selected.
#define ADTRG_HW           ADC_SC2_ADTRG_MASK
#define ADTRG_SW           0x00

// ADC Compare Function Enable: Disabled, or Enabled.
#define ACFE_DISABLED      0x00
#define ACFE_ENABLED       ADC_SC2_ACFE_MASK

// Compare Function Greater Than Enable: Greater, or Less.
#define ACFGT_GREATER      ADC_SC2_ACFGT_MASK
#define ACFGT_LESS         0x00

// Compare Function Range Enable: Enabled or Disabled.
#define ACREN_ENABLED      ADC_SC2_ACREN_MASK
#define ACREN_DISABLED     0x00

// DMA enable: enabled or disabled.
#define DMAEN_ENABLED      ADC_SC2_DMAEN_MASK
#define DMAEN_DISABLED     0x00

// Voltage Reference selection for the ADC conversions
// (***not*** the PGA which uses VREFO only).
// VREFH and VREFL (0) , or VREFO (1).

#define REFSEL_EXT         0x00
#define REFSEL_ALT         0x01
#define REFSEL_RES         0x02     /* reserved */
#define REFSEL_RES_EXT     0x03     /* reserved but defaults to Vref */

////ADCSC3

// Calibration begin or off
#define CAL_BEGIN          ADC_SC3_CAL_MASK
#define CAL_OFF            0x00

// Status indicating Calibration failed, or normal success
#define CALF_FAIL          ADC_SC3_CALF_MASK
#define CALF_NORMAL        0x00

// ADC to continously convert, or do a sinle conversion
#define ADCO_CONTINUOUS    ADC_SC3_ADCO_MASK
#define ADCO_SINGLE        0x00

// Averaging enabled in the ADC, or not.
#define AVGE_ENABLED       ADC_SC3_AVGE_MASK
#define AVGE_DISABLED      0x00

// How many to average prior to "interrupting" the MCU?  4, 8, 16, or 32
#define AVGS_4             0x00
#define AVGS_8             0x01
#define AVGS_16            0x02
#define AVGS_32            0x03

////PGA

// PGA enabled or not?
#define PGAEN_ENABLED      ADC_PGA_PGAEN_MASK
#define PGAEN_DISABLED     0x00

// Chopper stabilization of the amplifier, or not.
#define PGACHP_CHOP        ADC_PGA_PGACHP_MASK
#define PGACHP_NOCHOP      0x00

// PGA in low power mode, or normal mode.
#define PGALP_LOW          ADC_PGA_PGALP_MASK
#define PGALP_NORMAL       0x00

// Gain of PGA.  Selectable from 1 to 64.
#define PGAG_1             0x00
#define PGAG_2             0x01
#define PGAG_4             0x02
#define PGAG_8             0x03
#define PGAG_16            0x04
#define PGAG_32            0x05
#define PGAG_64            0x06


#define ADC_STATE_INIT                            0
#define ADC_STATE_CAPTURE_POT_0                   1
#define ADC_STATE_CAPTURE_POT_1                   2
#define ADC_STATE_CAPTURE_BATTERY_LEVEL           3
#define ADC_STATE_CAPTURE_LINE_SCAN               4

#ifndef _TFC_H
#define _TFC_H

/** Initialized the TFC API.   Call before using any other API calls.
*
*/
void TFC_Init();

void TFC_GPIO_Init();

/** ServoTicker will increment once every servo cycle.
* It can be used to synchronize events to the start of a servo cycle. ServoTicker is a volatile uint32_t and is updated in the TPM1 overlflow interrupt.  This means you will see ServoTicker increment on the rising edge of the servo PWM signal
*
*/
 extern volatile uint32_t TFC_ServoTicker;


/** Gets the state of the 4-positiomn DIP switch on the FRDM-TFC
*
*  @returns The lower 4-bits of the return value map to the 4-bits of the DIP switch
*/
uint8_t TFC_GetDIP_Switch();


/** Reads the state of the pushbuttons (SW1, SW2)  on the FRDM-TFC
*  @param Index Selects the pushbutton (0 for SW1 and 1 for SW2)
*  @returns A non-zero value if the button is pushed
*/
uint8_t TFC_ReadPushButton(uint8_t Index);


/** Controls the 4 battery level LEDs on the FRDM-TFC boards.
*
*  @param Value  The lower 4-bits of the parameter maps to the 4 LEDs.
*/
void TFC_SetBatteryLED(uint8_t Value);


/** Sets the servo channels
*
*  @param ServoNumber  Which servo channel on the FRDM-TFC to use (0 or 1).  0 is the default channel for steering
*  @param Position     Angle setting for servo in a normalized (-1.0 to 1.0) form.   The range of the servo can be changed with the InitServos function.
*                       This is called in the TFC constructor with some useful default values-->  20mSec period,  0.5mS min and 2.0mSec max.  you may need to adjust these for your own particular setup.
*/
void TFC_SetServo(uint8_t ServoNumber, float Position);

/** Initializes TPM for the servoes.  It also sets the max and min ranges
*
*  @param ServoPulseWidthMin    Minimum pulse width (in seconds) for the servo.   The value of -1.0 in SetServo is mapped to this pulse width.  I.E.  .001
*  @param ServoPulseWidthMax    Maximum pulse width (in seconds) for the servo.   The value of +1.0 in SetServo is mapped to this pulse width.  I.E.  .002
*  @param ServoPeriod           Period of the servo pulses (in seconds).  I.e.  .020 for 20mSec
*/

void TFC_InitServos(float ServoPulseWidthMin, float ServoPulseWidthMax, float ServoPeriod);


/** Initialized TPM0 to be used for generating PWM signals for the the dual drive motors.   This method is called in the TFC constructor with a default value of 4000.0Hz
*
*  @param SwitchingFrequency PWM Switching Frequency in floating point format.   Pick something between 1000 and 9000.   Maybe you can modulate it and make a tune.
*/
void TFC_InitMotorPWM(float SwitchingFrequency);

/** Sets the PWM value for each motor.
*
*  @param MotorA    The PWM value for HBridgeA. The value is normalized to the floating point range of -1.0 to +1.0.    -1.0 is 0% (Full Reverse on the H-Bridge) and 1.0 is 100% (Full Forward on the H-Bridge)
*  @param MotorB    The PWM value for HBridgeB. The value is normalized to the floating point range of -1.0 to +1.0.    -1.0 is 0% (Full Reverse on the H-Bridge) and 1.0 is 100% (Full Forward on the H-Bridge)
*/
void TFC_SetMotorPWM(float MotorA ,float MotorB);

/** Reads the potentiometers
*
*  @param Channel   Selects which pot is read.   I.e.  0 for POT0 or 1 for POT1
*  @returns    Pot value from -1.0 to 1.0
*/
float TFC_ReadPot(uint8_t Channel);

/** Gets the current battery voltage
*
*  @returns    Battery voltage in floating point form.
*/
float TFC_ReadBatteryVoltage();



/** Sets the Battery level indiciate
*
*  @param BattLevel   A number betwween 0 and 4.   This will light the bar from left to right with the specifified number of segments.
*
*/
void TFC_SetBatteryLED_Level(uint8_t BattLevel);


void TFC_SetServoDutyCycle(uint8_t ServoNumber, float DutyCycle);

/** Pointer to two channels of line scan camera data.   Each channel is 128 points of uint8_t's.  Note that the underlying implementation is ping-pong buffer  These pointers will point to the 
*inactive buffer.   
*
*/

extern volatile uint16_t * TFC_LineScanImage0;
extern volatile uint16_t * TFC_LineScanImage1;


/** This flag will increment when a new frame is ready.  Check for a non zero value (and reset to zero!) when you want to read the camera(s)
*
*/

extern volatile uint8_t TFC_LineScanImageReady;





/** @} */


#endif
