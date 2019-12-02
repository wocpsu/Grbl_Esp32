#include "grbl.h"
#ifdef CPU_MAP_MIDTBOT2
#ifdef USE_KINEMATICS

void inverse_kinematics(float *target, plan_line_data_t *pl_data, float *position)
{

    // compensate for feed_rate
    /*
        The midtbot is not a normal corexy machine. The "X" axis has to turn twice as a the 
        Y axis because the single belt is also moving during X moves. This affects the feed rate.

        Compensation for that can be done here. 


    */
   #define MIDTBOT_FEEDRATE_X_FACTOR 2.0

    // calculate cartesian move distance for each axis
	float dx = target[X_AXIS] - position[X_AXIS];
	float dy = target[Y_AXIS] - position[Y_AXIS];

    // We don't compensate if there is no X change or if it is a rapid
    if ((dx == 0) || (pl_data->condition & PL_COND_FLAG_RAPID_MOTION)) {
        mc_line(target, pl_data); 
        return;
    }

    float cartesian_dist = hypot_f(dx, dy);
    float actual_dist =  hypot_f(dx * MIDTBOT_FEEDRATE_X_FACTOR, dy);
    float feed_comp = (actual_dist / cartesian_dist);

    if (dy == 0) {
        pl_data->feed_rate *= MIDTBOT_FEEDRATE_X_FACTOR; // it is all x motion so multiply by 2
    }
    else
    {
        pl_data->feed_rate *= feed_comp;
    }
    
    mc_line(target, pl_data);
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