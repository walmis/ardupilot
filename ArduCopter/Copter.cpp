/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  constructor for main Copter class
 */
#if AP_HAL_BOARD_DRIVER == AP_HAL_SKYFALCON
extern const AP_HAL::HAL& hal;
#else
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
#endif

Copter::Copter(void) :
#if defined(HAL_BOARD_LOG_DIRECTORY)
    DataFlash(HAL_BOARD_LOG_DIRECTORY),
#endif
    ins_sample_rate(AP_InertialSensor::RATE_400HZ),
    flight_modes(&g.flight_mode1),
    sonar_enabled(true),
    mission(ahrs, 
            FUNCTOR_BIND_MEMBER(&Copter::start_command, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Copter::verify_command, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Copter::exit_mission, void)),
    control_mode(STABILIZE),
#if FRAME_CONFIG == HELI_FRAME  // helicopter constructor requires more arguments
    motors(g.rc_7, g.rc_8, g.heli_servo_1, g.heli_servo_2, g.heli_servo_3, g.heli_servo_4, MAIN_LOOP_RATE),
#elif FRAME_CONFIG == TRI_FRAME  // tri constructor requires additional rc_7 argument to allow tail servo reversing
    motors(MAIN_LOOP_RATE),
#elif FRAME_CONFIG == SINGLE_FRAME  // single constructor requires extra servos for flaps
    motors(g.single_servo_1, g.single_servo_2, g.single_servo_3, g.single_servo_4, MAIN_LOOP_RATE),
#elif FRAME_CONFIG == COAX_FRAME  // single constructor requires extra servos for flaps
    motors(g.single_servo_1, g.single_servo_2, MAIN_LOOP_RATE),
#else
    motors(MAIN_LOOP_RATE),
#endif
    scaleLongDown(1),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry(ahrs, battery),
#endif
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
    G_Dt(0.0025f),
    inertial_nav(ahrs),
#if FRAME_CONFIG == HELI_FRAME
    attitude_control(ahrs, aparm, motors, g.p_stabilize_roll, g.p_stabilize_pitch, g.p_stabilize_yaw,
                     g.pid_rate_roll, g.pid_rate_pitch, g.pid_rate_yaw),
#else
    attitude_control(ahrs, aparm, motors, g.p_stabilize_roll, g.p_stabilize_pitch, g.p_stabilize_yaw,
                     g.pid_rate_roll, g.pid_rate_pitch, g.pid_rate_yaw),
#endif
    pos_control(ahrs, inertial_nav, motors, attitude_control,
                g.p_alt_hold, g.p_vel_z, g.pid_accel_z,
                g.p_pos_xy, g.pi_vel_xy),
    wp_nav(inertial_nav, ahrs, pos_control, attitude_control),
    circle_nav(inertial_nav, ahrs, pos_control),
    ServoRelayEvents(relay),
#if CAMERA == ENABLED
    camera(&relay),
#endif
#if MOUNT == ENABLED
    camera_mount(ahrs, current_loc),
#endif
#if AC_FENCE == ENABLED
    fence(inertial_nav),
#endif
#if AC_RALLY == ENABLED
    rally(ahrs),
#endif
#if SPRAYER == ENABLED
    sprayer(&inertial_nav),
#endif
#if PARACHUTE == ENABLED
    parachute(relay),
#endif
#if AP_TERRAIN_AVAILABLE
    terrain(ahrs, mission, rally),
#endif
    param_loader(var_info)
{
}

Copter copter;
