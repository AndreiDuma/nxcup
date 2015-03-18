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
void camera_handle_irq();

static inline void light_sensor_handle_irq() __attribute__((always_inline));

Serial bluetooth(MBED_UART1);
SpeedSensor left(PTA1, 0, NULL);
SpeedSensor right(PTA2, 21, NULL);
Camera camera(camera_handle_irq);
LightSensor light(PTA16, &light_sensor_handle_irq);
InterruptIn btn13(PTA13);
Timer bluetooth_clk;
Timer pid_clk;

#include "car_logic.h"

volatile float integ_value;

/*
 * tell camera integration time based on read level of light
 * status: light sensor - camera calibration
 */
static inline void light_sensor_handle_irq()
{
	/*
	if (light.light_value > 22)
		camera.set_integration_time(0.3);
	else if (light.light_value > 20)
		camera.set_integration_time(1.0);
	else
		camera.set_integration_time(5.0);
	*/
	camera.set_integration_time(integ_value);
}

char bt_data[130];
float speed = 1.0;

volatile bool debouncer_btn_pressed = false;

/* unused */
void red_btn_irq()
{
	if (!debouncer_btn_pressed) {
		debouncer_btn_pressed = true;
		TFC_BAT_LED3_ON;
		light.start();
	}
}

/* check ACKs for BT cmd */
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

/* copy data from camera after it has been read */
void camera_handle_irq()
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

void bt_recv_irq()
{
	float th;

	bluetooth.scanf("%f", &th);

	integ_value = th;
}

// 3014
int main()
{
	TFC_GPIO_Init();
	TFC_InitLineScanCamera();
	TFC_InitServos(SERVO_MIN_PULSE_WIDTH_DEFAULT , SERVO_MAX_PULSE_WIDTH_DEFAULT, SERVO_DEFAULT_PERIOD);
	TFC_InitMotorPWM(FTM0_DEFAULT_SWITCHING_FREQUENCY);
	TFC_HBRIDGE_ENABLE;
	TFC_SetMotorPWM(0.5,0.5);
	TFC_SetServo(0.0, 0.0);
	TFC_BAT_LED1_ON;
	bluetooth.baud(115200);
	bluetooth.attach(&bt_recv_irq);

	camera.init_camera_read();
	bluetooth_clk.start();
	pid_clk.start();
	//btn13.rise(&init_light_sensor);

	for (;;) {
		camera.iterate();
		send_data_to_bt();
		take_a_decission();
	}
}
