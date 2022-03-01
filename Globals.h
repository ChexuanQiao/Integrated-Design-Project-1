#ifndef GLOBALS_H
#define GLOBALS_H

extern const int L2LF_receive;
extern const int L1LF_receive = 8;
extern const int R1LF_receive = 9;
extern const int R2LF_receive;

// Distance sensor supply and receive pins.
extern const int DS_supply;
extern const int DS_receive;

// Tunable Parameters.
extern const float kp = 30; // Proportional gain.
extern const float ki = 0; // Integral gain.
extern const float kd = 0; // Derivative gain.

extern const int main_loop_delay_time = 100; // main loop delay.
extern const int delay_time = 100; // Misc delay.
extern const int max_speed = 255; // Maximum allowable motor speed.
extern const int ref_speed = 150; // Normal forward motor speed.
extern const int turn_speed = 150; // Turning speed.

extern const int intxn_queue_length = 5; // intersection queue length.
extern const int intxn_detection_threshold = 4; // IntersectionDetection Threshold, the number of 1s in intersection queue.
extern const int intxn_deb_time = 1000; // Intersection debounce threshold time.
extern int l_intxn_deb_prev; // Last Left intersection debounce start time.
extern int r_intxn_deb_prev; // Last Right intersection debounce start time.
extern int l_intxn_deb = 1; // Intersection debounce checker.
extern int r_intxn_deb = 1;

extern const int sweep_queue_length = 5; // Distance sensor sweep queue length.
extern const int sweep_queue_threshold = 4;
extern float max_dist;
extern float min_dist;
extern float dip_threshold = 1;
extern int block_found = 0;
extern int search_time;

// ---------------------

extern int task = 0;
extern int main_loop_counter = 0;

// PID parameters.
extern float PIDError = 0; // PID control feedback
extern float P, I, D;
extern float pre_I = 0;
extern float pre_P = 0;
extern unsigned long current_time = 0;
extern unsigned long prev_time = 0;

extern int Turn = 2;

extern int left_intxn_counter = 0;
extern int right_intxn_counter = 0;
extern int left_white_counter = 0;
extern int right_white_counter = 0;

// Line sensor data. 0 (black) or 1 (white).
extern int L2LF_data;
extern int L1LF_data;
extern int R1LF_data;
extern int R2LF_data;

extern float DS_data; // Distance Sensor data.

#endif
