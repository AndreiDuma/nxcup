#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <stdint.h>
#include "ComparatorIn.h"

#define NO_LINE_DETECTED -1

class Camera {
private:
	ComparatorIn comparator;
	void (*handler)(void);
	int left_line_pos;
	int right_line_pos;

	void get_raw_data();
	void set_left_line();
	void set_right_line();
public:
	Camera(void (*handler)(void));
	int get_left_line();
	int get_right_line();
	void init_camera_read();
	void set_th(float th);
	void set_integration_time(float integ_time_ms);
	void iterate();
	uint8_t pixels[128];
	volatile float pixels_integration_time_ms;

	int sum_left;
	int sum_right;
};

#endif /* _CAMERA_H_ */
