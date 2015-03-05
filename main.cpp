#include "mbed.h"
#include "TFC.h"
#include "SpeedSensor.h"

/* TODO
 * Comparator class uses Vref of 3.3Vdd
 * It might be needed to provide other Vref source
 */

// CAMERA PTC7
// TODO SpeedSensor mutex (atomic operation)

#define BLUETOOTH_ON 1

#if BLUETOOTH_ON
	Serial bluetooth(MBED_UART1);
	SpeedSensor left(PTA1);
	SpeedSensor right(PTA2);
	
	bool check_bt_state(char *str)
	{
		int i;
		
		for (i = 0; str[i] != '\0'; i++)
			if (str[i] != bluetooth.getc())
				return false;
	
		return true; 
	}
#endif

void TFC_InitADC_System();
// 3014
int main()
{
	/*
	 * UART2 on PTD2 and PDT3 (J10 zone) might be available for bluetooth
	 * set to altenative 3
	 * TODO: signal detection might not work well
	 */
	float speed = 0.085;
	TFC_GPIO_Init();
	TFC_InitADC_System();
	TFC_InitServos(SERVO_MIN_PULSE_WIDTH_DEFAULT , SERVO_MAX_PULSE_WIDTH_DEFAULT, SERVO_DEFAULT_PERIOD);
	TFC_InitMotorPWM(FTM0_DEFAULT_SWITCHING_FREQUENCY);
	TFC_HBRIDGE_ENABLE;
	TFC_SetMotorPWM(speed,speed);
	#if BLUETOOTH_ON
	TFC_BAT_LED1_ON;
	bluetooth.baud(115200);

	TFC_BAT_LED3_ON;
	for (;;) {
		int i;
		
		
		bluetooth.puts("\nNew data:\n");
		for (i = 0; i < 20; i++) {
			//speed = i * 0.05;
			//TFC_SetMotorPWM(0.0, speed); 
			wait_ms(1000);
			bluetooth.printf("blabla\n");
			bluetooth.printf("\t%f%%\t%d\t%d\n", speed * 100, left.speed, right.speed);
		}
	}
	#else


	for (;;) {
		if(TFC_PUSH_BUTTON_0_PRESSED)
			TFC_BAT_LED0_ON;
		else
			TFC_BAT_LED0_OFF;

		if(TFC_PUSH_BUTTON_1_PRESSED)
			TFC_BAT_LED3_ON;
		else
			TFC_BAT_LED3_OFF;

		TFC_SetServo(0, TFC_ReadPot(0));
		wait_ms(50);
	}
	#endif
}
