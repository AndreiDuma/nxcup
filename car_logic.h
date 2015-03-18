
enum {
	STRAIGHT_LINE_STATE = 0,
	LEFT_LINE_STATE = 1,
	RIGHT_LINE_STATE = 2
};

#define USE_OLD_VALUE 2.0
#define PID_INTERVAL_MS 10.0
#define STR8_L_TH_STUBBORNESS 10

static float get_value_from_pid(float position)
{
	static float old_pos = 0;
	static float KP = 7;
	static float KD = 2;
	float dGain;
	float rValue;


	dGain = position - old_pos;
	//old_pos = 1.0 - position;
	old_pos = position;

	rValue = KP * position + KD * dGain;

	/* used only to keep derivative part sane (rValue) */
	if (position == 0.0)
		return 0.0;

	if (rValue > 1.0)
		rValue = 1.0;

	return rValue;
}

static float get_next_pos(float position)
{
	static int old_state = STRAIGHT_LINE_STATE;
	static int dont_go_straight_stubborness = 0;

	if (position == 0.0) {
		if (dont_go_straight_stubborness < STR8_L_TH_STUBBORNESS) {
			dont_go_straight_stubborness++;
		} else {
			old_state = STRAIGHT_LINE_STATE;
			/* keep derivative part of PID sane */
			return get_value_from_pid(0.0);
		}
	}

	/*
	 * LEFT LINE STATE:
	 *	| car is still stubborn and doesn't want to go straight
	 *	| if actual camera read line is still left, use actual value
	 */
	if (old_state == LEFT_LINE_STATE) {
		if (position < 0) {
			dont_go_straight_stubborness = 0;
			return get_value_from_pid(position);
		}

		return USE_OLD_VALUE;
	}

	/*
	 * RIGHT LINE STATE:
	 *	| car is still stubborn and doesn't want to go straight
	 *	| if actual camera read line is still right, use actual value
	 */
	if (old_state == RIGHT_LINE_STATE) {
		if (position > 0) {
			dont_go_straight_stubborness = 0;
			return -get_value_from_pid(-position);
		}

		return USE_OLD_VALUE;
	}

	/* STRAIGHT_LINE_STATE */
	/* last position was straight, using actual camera value */
	if (position < 0) {
		dont_go_straight_stubborness = 0;
		old_state = LEFT_LINE_STATE;
		return get_value_from_pid(position);
	}

	if (position > 0) {
		dont_go_straight_stubborness = 0;
		old_state = RIGHT_LINE_STATE;
		return -get_value_from_pid(-position);
	}

	return USE_OLD_VALUE;
}

static void take_a_decission()
{
	int left_line;
	int right_line;
	float servo_value;

	if (pid_clk.read_ms() > PID_INTERVAL_MS) {
		pid_clk.stop();

		left_line = camera.get_left_line();
		right_line = camera.get_right_line();

		if (left_line != NO_LINE_DETECTED) {
			servo_value = get_next_pos(left_line / 54.0);
		} else if (right_line != NO_LINE_DETECTED) {
			servo_value = get_next_pos((right_line - 118.0) / 54.0);
		} else {
			servo_value = get_next_pos(0.0);
		}

		if (servo_value < USE_OLD_VALUE) {
			/* SPEED CHANGE NOT TESTED */
			/*
			if (servo_value < 0.0)
				TFC_SetMotorPWM(0.5, 0.7);
			else if (servo_value > 0.0)
				TFC_SetMotorPWM(0.7, 0.5);
			else
				TFC_SetMotorPWM(0.5, 0.5);
			*/

			TFC_SetServo(0, servo_value);
		}

		pid_clk.reset();
		pid_clk.start();
	}
}

