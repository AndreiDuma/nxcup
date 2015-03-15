#include "mbed.h"
#include "TFC.h"
#include "SpeedSensor.h"
#include "Camera.h"
#include "LightSensor.h"

// CAMERA PTC7
// TODO SpeedSensor nu e stabil semnalul pizdii, d-asta citeste viteza mare la viteze mici -_-'
// TODO La viteza mare s-ar putea sa nu mai citeasca corect
// TODO dezactivat intreruperile pentru potentiometre and other in TFC.cpp

// activate histerizis
// trecut BT pe buffered serial
void car_logic();

static inline void set_camera_integ_time_ms() __attribute__((always_inline));

Serial bluetooth(MBED_UART1);
SpeedSensor left(PTA1, 0, NULL);
SpeedSensor right(PTA2, 21, NULL);
Camera camera(car_logic);
LightSensor light(PTA16, &set_camera_integ_time_ms);
InterruptIn btn13(PTA13);
Timer bluetooth_clk;

volatile float integ_value;

static inline void set_camera_integ_time_ms()
{
	if (light.light_value >= 15)
		camera.set_integration_time(9.0);
	else if (light.light_value >= 6)
		camera.set_integration_time(13.5);
	else
		camera.set_integration_time(integ_value); // 15.5
}

char bt_data[130];
float speed = 1.0;

volatile bool debouncer_btn_pressed = false;

void init_light_sensor()
{
	if (!debouncer_btn_pressed) {
		debouncer_btn_pressed = true;
		TFC_BAT_LED3_ON;
		light.start();
	}
}

bool check_bt_state(char *str)
{
	int i;
	
	for (i = 0; str[i] != '\0'; i++)
		if (str[i] != bluetooth.getc())
			return false;

	return true;
}

void TFC_InitADC_System();
void TFC_InitLineScanCamera();

void car_logic()
{
	int i;

	bt_data[128] = '\n';
	bt_data[129] = '\0';

	memcpy(bt_data, camera.pixels, 128);
	for (i = 0; i < 128; i++)
		bt_data[i] += '0';
}

static inline void send_data_to_bt() __attribute__((always_inline));

static inline void send_data_to_bt()
{
	if (bluetooth_clk.read_ms() > 1000) {
		bluetooth_clk.stop();
		bluetooth.puts(bt_data);
		bluetooth.printf("light = %d\n", light.light_value);
		bluetooth_clk.reset();
		bluetooth_clk.start();
	}
}

void change_th()
{
	float th;

	bluetooth.scanf("%f", &th);

	integ_value = th;
}

// 3014
int main()
{
	int left_line, right_line;
	/*
	 * UART2 on PTD2 and PDT3 (J10 zone) might be available for bluetooth
	 * set to altenative 3
	 * TODO: signal detection might not work well
	 */
	TFC_GPIO_Init();
	//TFC_InitADC_System();
	TFC_InitLineScanCamera();
	TFC_InitServos(SERVO_MIN_PULSE_WIDTH_DEFAULT , SERVO_MAX_PULSE_WIDTH_DEFAULT, SERVO_DEFAULT_PERIOD);
	TFC_InitMotorPWM(FTM0_DEFAULT_SWITCHING_FREQUENCY);
	//TFC_HBRIDGE_ENABLE;
	TFC_SetMotorPWM(0.0,0.0);
	TFC_SetServo(0.0, 0.0);
	TFC_BAT_LED1_ON;
	bluetooth.baud(115200);
	bluetooth.attach(&change_th);

	camera.init_camera_read();
	bluetooth_clk.start();
	btn13.rise(&init_light_sensor);
	for (;;) {
		camera.iterate();
		/*
		int i;
		bluetooth.puts("\nNew data:\n");
		for (i = 0; i < 20; i++) {
			//speed = i * 0.05;
			//TFC_SetMotorPWM(0.0, speed); 
			wait_ms(1000);
			bluetooth.printf("blabla\n");
			bluetooth.printf("\t%f%%\t%d\t%d\n", speed * 100, left.speed, right.speed);
		}
		*/
		send_data_to_bt();
		/*
		if (camera.sum_left - 5 > camera.sum_right)
			TFC_SetServo(0.0, -0.5);
		else if (camera.sum_left + 5 > camera.sum_right)
			TFC_SetServo(0.0, 0.5);
		else
			TFC_SetServo(0.0, 0.0);
		*/


		/*
		left_line = camera.get_left_line();
		right_line = camera.get_right_line();

		if (left_line != NO_LINE_DETECTED) {
			TFC_SetServo(0.0, -0.5);
		} else if (right_line != NO_LINE_DETECTED) {
			TFC_SetServo(0.0, 0.5);
		} else {
			TFC_SetServo(0.0, 0.0);
		}
		*/

	}
}
