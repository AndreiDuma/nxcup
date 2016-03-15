#include "mbed.h"
#include "TFC.h"
#include <time.h>

#define KP 0.03
#define KD 0
#define MIN_LIMIT	7
#define MAX_LIMIT	120


float TIMP_EXPUNERE = 0.5;
double pixels[3][128];
double average_pixels[128];
int fixed_pixels[128];
double err;
int old_pos, new_pos;
double prag;
int direction;
int sum = 0;
double motor1, motor2;


Serial bluetooth(MBED_UART1);
AnalogIn camera(PTD5); // sau PTD6 (nu stiu exact care)


void TFC_InitLineScanCamera();

void camera_fix_pixels();
void camera_detect_line();
double pid_algoritm();
double camera_get_raw_data(int index);


static inline void camera_hclk()
{
	asm volatile("nop");
	asm volatile("nop");
}

static inline void camera_clk()
{
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
}


double camera_get_raw_data(int index)
{
	int i;
	double camera_value, prag;

	double high = -100.0;
	double low  =  100.0;

	TAOS_CLK_LOW;
	TAOS_SI_HIGH;
	camera_hclk();
	TAOS_CLK_HIGH;
	camera_hclk();
	TAOS_SI_LOW;
	camera_hclk();
	TAOS_CLK_LOW;
	camera_clk();

	for (i = 0; i < 128; i++) {
		camera_value = camera.read();
		pixels[index][i] = camera_value;
		if ( (i >= MIN_LIMIT) && (i <= MAX_LIMIT) ) {
			if (camera_value > high) 
				high = camera_value;
			else if (camera_value < low)
				low = camera_value;
		}
		TAOS_CLK_HIGH;
		camera_clk();
		TAOS_CLK_LOW;
	}
	prag = (high - low) / 2;
	return prag;
}


void camera_get_3_raw_data()
{
	double prag1, prag2, prag3;
	int i;

	/* init reglaj camera */
	camera_get_raw_data(0);
	wait_ms(TIMP_EXPUNERE);

	prag1 = camera_get_raw_data(0);
	wait_ms(TIMP_EXPUNERE);
	prag2 = camera_get_raw_data(1);
	wait_ms(TIMP_EXPUNERE);
	prag3 = camera_get_raw_data(2);

	prag = ( prag1 + prag2 + prag3 ) / 3;
	for (i = MIN_LIMIT; i <= MAX_LIMIT; i++)
		average_pixels[i] = ( pixels[0][i] + pixels[1][i] + pixels[2][i] ) / 3;
}


void camera_fix_pixels()
{
	int i;
	for (i = MIN_LIMIT; i <= MAX_LIMIT; i++) {
		if (average_pixels[i] <= prag) {
			fixed_pixels[i] = 1; 	/* black pixel */
			sum += fixed_pixels[i];
		}
		else 
			fixed_pixels[i] = 0;  	/* white pixel */
	}
}


void processing_data() 
{
	camera_detect_line();
	double servo_dir = KP * new_pos + KD * (new_pos - old_pos);

	if ((servo_dir > 0.75 && direction == 1) || (servo_dir < -0.75 && direction == -1)) {
		TFC_SetServo(0, servo_dir);
	}

	if ((servo_dir > 0.75 && direction == -1) || (servo_dir < -0.75 && direction == 1)) {
	 	return;
	}
	if (servo_dir > 1.00)
		servo_dir = 1.00;
	if (servo_dir < -1.00)
		servo_dir = -1.00;
	TFC_SetServo(0, servo_dir);
}


void camera_detect_line()
{
	int i, j;
	int left_sum, right_sum;

	left_sum  = fixed_pixels[63] + fixed_pixels[62] + fixed_pixels[61];
	right_sum = fixed_pixels[64] + fixed_pixels[65] + fixed_pixels[66];
	

	if ((left_sum + right_sum >= 3) && (direction != 0))
		return;

	if (direction == -1 && new_pos <= 3) {
	//	wait_ms(0.5);
		direction = 0;
	}
	if (direction == 1 && new_pos >= -3) {
	//	wait_ms(0.5);
		direction = 0;
	}

	/* Black on left => go right */
    if (left_sum >= 2) {
    	old_pos = new_pos;
    	new_pos = 63 - MIN_LIMIT;
    	direction = 1;
    	return;
    }
    
    /* Black on right => go left */
    if (right_sum >= 2) {
    	old_pos = new_pos;
    	new_pos = -63 + MIN_LIMIT;
    	direction = -1;
    	return;
    }
 
	for (i = 60, j = 67; i >= MIN_LIMIT && j <= MAX_LIMIT; i--, j++)
	{
		left_sum  = left_sum - fixed_pixels[i + 3] + fixed_pixels[i];
		if (left_sum >= 2) {
			old_pos = new_pos;
			new_pos = i + 2 - MIN_LIMIT;
			direction = 1;
			return;
		}
		
		right_sum = right_sum - fixed_pixels[j - 3] + fixed_pixels[j];
		if (right_sum >= 2) {
			old_pos = new_pos;
			new_pos = -MAX_LIMIT + j + 2 + MIN_LIMIT;
			direction = -1;
			return;
		}
	}
}


int main()
{
	clock_t t_start, t_stop;
	int i;
	float seconds;

	bluetooth.baud(19200);	

	TFC_GPIO_Init();
	TFC_InitLineScanCamera();
	TFC_InitServos(SERVO_MIN_PULSE_WIDTH_DEFAULT, SERVO_MAX_PULSE_WIDTH_DEFAULT, SERVO_DEFAULT_PERIOD);
	TFC_InitMotorPWM(FTM0_DEFAULT_SWITCHING_FREQUENCY);

	TFC_HBRIDGE_ENABLE;  /* enable motors */	
	TFC_SetServo(0, 0.0);

	TFC_BAT_LED0_ON;
	TFC_BAT_LED1_ON;
	TFC_BAT_LED2_ON;
	TFC_BAT_LED3_ON;
	direction = 0;
	t_start = clock();	/* Marcam momentul de inceput */

	while(1) {
		camera_get_3_raw_data();/* take data from camera 3 times in a row */		
		camera_fix_pixels(); 	/* convert pixels to 1 or 0 values */
		
		processing_data();		/* process data; PID algorithm */

		t_stop = clock();		/* Marcam momentul de sfarsit */
		seconds = ((float)(t_stop - t_start)) / CLOCKS_PER_SEC;
		/* Debug - bleutooth */
		if (seconds >= 1) { 
			for (i = MIN_LIMIT; i <= MAX_LIMIT; i ++)
				bluetooth.printf("%d", fixed_pixels[i]);
			bluetooth.printf("\n");
			bluetooth.printf("......%d\n", direction);
			t_start = clock();
		}

		wait_ms(TIMP_EXPUNERE);
		TFC_SetMotorPWM(-0.42, -0.42);	/* set power to the motor */
	}
}