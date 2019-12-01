#include "grbl.h"
#ifdef CPU_MAP_MIDTBOT2
#ifdef USE_KINEMATICS

void inverse_kinematics(float *target, plan_line_data_t *pl_data, float *position)
{
    float kins[N_AXIS]; // target location in polar coordinates

    float x_offset = gc_state.coord_system[X_AXIS]+gc_state.coord_offset[X_AXIS]; // offset from machine coordinate system
	float y_offset = gc_state.coord_system[Y_AXIS]+gc_state.coord_offset[Y_AXIS]; // offset from machine coordinate system

    // calculate cartesian move distance for each axis
	float dx = target[X_AXIS] - position[X_AXIS];
	float dy = target[Y_AXIS] - position[Y_AXIS];
	float dz = target[Z_AXIS] - position[Z_AXIS];

    // X = 1/4 (A + B)
    // Y = 1/2 (A - B)

    float dist = sqrt( (dx * dx) + (dy * dy) + (dz * dz));



    kins[X_AXIS] = (target[X_AXIS] * 2) + target[Y_AXIS] + x_offset;
    kins[Y_AXIS] = (target[X_AXIS] * 2) - target[Y_AXIS] + y_offset;
    kins[Z_AXIS] = target[Z_AXIS];

   // kins[X_AXIS] = position[X_AXIS] + ((dx * 2) + dy);
    //kins[Y_AXIS] = position[Y_AXIS] + ((dx * 2) -dy);
    //kins[Z_AXIS] = target[Z_AXIS];

    grbl_sendf(CLIENT_SERIAL, "[MSG:Kins offset %4.3f %4.3f]\r\n", x_offset, y_offset);
    grbl_sendf(CLIENT_SERIAL, "[MSG:Kins Target %4.3f %4.3f]\r\n", target[X_AXIS], target[Y_AXIS]);
    grbl_sendf(CLIENT_SERIAL, "[MSG:Kins Kins %4.3f %4.3f]\r\n", kins[X_AXIS], kins[Y_AXIS]);

    mc_line(kins, pl_data);
}

void forward_kinematics(float *position)
{
    float original_position[N_AXIS]; // temporary storage of original
	float print_position[N_AXIS];
	int32_t current_position[N_AXIS]; // Copy current state of the system position variable
	
	memcpy(current_position,sys_position,sizeof(sys_position));
	system_convert_array_steps_to_mpos(print_position,current_position);
	
	original_position[X_AXIS] = print_position[X_AXIS] - gc_state.coord_system[X_AXIS]+gc_state.coord_offset[X_AXIS];
	original_position[Y_AXIS] = print_position[Y_AXIS] - gc_state.coord_system[Y_AXIS]+gc_state.coord_offset[Y_AXIS];
	original_position[Z_AXIS] = print_position[Z_AXIS] - gc_state.coord_system[Z_AXIS]+gc_state.coord_offset[Z_AXIS];
	
	position[X_AXIS] = 0.25 * (original_position[X_AXIS] + original_position[Y_AXIS]);
	position[Y_AXIS] = 0.50 * (original_position[X_AXIS]- original_position[Y_AXIS]);
	position[Z_AXIS] = original_position[Z_AXIS];  // unchanged
}

// this get called before homing 
// return true to complete normal home
// return false to exit normal homing
bool kinematics_homing(uint8_t cycle_mask) 
{
	return true; // finish normal homing cycle
}



#endif
#endif