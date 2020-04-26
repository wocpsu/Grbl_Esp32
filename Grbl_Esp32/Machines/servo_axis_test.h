/*
    servo_axis.h
    Part of Grbl_ESP32

    Pin assignments for the Buildlog.net pen laser controller V1
    using servos.

    For laser mode, you do not need to change anything
    Note: You can use all 3 modes at the same time if you want

    2018    - Bart Dring
    2020    - Mitch Bradley

    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Grbl_ESP32.  If not, see <http://www.gnu.org/licenses/>.
*/

#define MACHINE_NAME "SERVO_AXIS_TESTING"

#define SPINDLE_TYPE            SPINDLE_TYPE_NONE

// #define                         USE_SERVO_AXES
#define SERVO_X_PIN             GPIO_NUM_27
#define SERVO_X_RANGE_MIN       0.0
#define SERVO_X_RANGE_MAX       20.0
