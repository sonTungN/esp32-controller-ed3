#pragma once

extern double motor_cmd_fl, motor_cmd_fr, motor_cmd_bl, motor_cmd_br;
extern double speed_fl, speed_fr, speed_bl, speed_br;
extern double speed_fl_ref, speed_fr_ref, speed_bl_ref, speed_br_ref;

// Initialization & Setup functions
void init_setup_motor();
void init_setup_encoders();
void init_pid();

// Hardware handling functions
void compute_pid();
void run_motors();