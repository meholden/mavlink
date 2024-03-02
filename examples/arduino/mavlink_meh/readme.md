# Notes on making this example work #
by Mike Holden www.holdentechnology.com

Todo:
fix hack on enable custom modes in heartbeat code, need a function to set/unset this mode.  Update readme

## Installing

I put two folders into my arduino libraries folder
-- mavlink (you must delete the one from the library manager first if installed)
-- units (from bolderflight)

## Hacks and compile notes
Put the UART you want to use in the setup() function.  
Unfortunately you cannot use USB UARTs like the teensy has, or bluetooth on ESP32 only hardwareSerial stuff.

I had trouble with one of the templates and unrolled it myself
There may be a fix to keep it in a template but this works
* in `telemetry.cpp`
    * `WrapToPi `
    * `WrapTo2Pi `

I had to install and #include the `elapsedMillis` library explicitly for esp32 but not for teensy devices.  
This #include is in a few of the .h files where `#include "Arduino.h"` is found.

I still get a ton of compiler warnings about data structure packing/alignment

## What it does
The baseline example code sends just about everything from the update function!  You need to fill in all the data though.
This code fills in data using simulated numbers, to show how to get your flight code variables into the telemetry stream.

Parameters are uploaded and can be downloaded.  5 parameters in this example.

Heartbeat is sent automatically.  
* Modes enumerated in heartbeat.h, note STABALIZED spelling ugh.  
* Mission planner confused by these modes (?) but QGC is okay
* This example sets the custom mode that Mission planner likes, and there is a hack in 

Command long like for arm/disarm not supported, check mavlink.h for which long commands it handles.

Waypoint missions are supported and the demo code will print stuff when the mission is changed on the vehicle.
There appear to be msg handlers for telling ground you have reached a waypoint and willadvance to next waypoint 
and also to let ground tell you which waypoint to go to. Untested by MEH, see mission.cpp

### Breakdown of telemetry streams
Telemetry from the vehicle is broken into a few streams, which each send a few mavlink messages, which each send a few variables.  The streams and the messages are listed below.  To find out the variables, 
look in the mavlink library (search for the message pack function listed or go to https://mavlink.io/en/messages/common.html)

Code appears to wait for groundstation to request the stream.  Mission planner does, QGC maybe not?  Not entirely sure MEH.

* SRx_RAW_SENS
    * `mavlink_msg_scaled_imu_pack`(sys_id_, comp_id_, &msg_,
                                         sys_time_ms_,
                                         accel_x_mg_, accel_y_mg_, accel_z_mg_,
                                         gyro_x_mradps_, gyro_y_mradps_,
                                         gyro_z_mradps_, mag_x_mgauss_,
                                         mag_y_mgauss_, mag_z_mgauss_,
                                         temp_cc_);
    * `mavlink_msg_gps_raw_int_pack`(sys_id_, comp_id_, &msg_,
                                          sys_time_us_, fix_, lat_dege7_,
                                          lon_dege7_, alt_msl_mm_, eph_, epv_,
                                          vel_cmps_, track_cdeg_, num_sv_,
                                          alt_wgs84_mm_, h_acc_mm_, v_acc_mm_,
                                          vel_acc_mmps_, hdg_acc_dege5_,
                                          yaw_cdeg_);
    * `mavlink_msg_scaled_pressure_pack`(sys_id_, comp_id_, &msg_,
                                              sys_time_ms_,
                                              static_pres_hpa_,
                                              diff_pres_hpa_,
                                              static_temp_cc_,
                                              diff_temp_cc_);
* SRx_EXT_STAT
    * `mavlink_msg_sys_status_pack`(sys_id_, comp_id_, &msg_,
                                         sensors_present_, sensors_present_,
                                         sensors_healthy_, load_,
                                         voltage_battery_, current_battery_,
                                         battery_remaining_, drop_rate_comm_,
                                         errors_comm_, errors_count_[0],
                                         errors_count_[1], errors_count_[2],
                                         errors_count_[3],
                                         onboard_control_sensors_present_ext_,
                                         onboard_control_sensors_present_ext_,
                                         onboard_control_sensors_present_ext_);
    * `mavlink_msg_battery_status_pack`(sys_id_, comp_id_, &msg_,
                                             id_, battery_function_, type_,
                                             temp_, &volt_[0], current_,
                                             current_consumed_,
                                             energy_consumed_,
                                             battery_remaining_,
                                             time_remaining_,
                                             charge_state_, &volt_[10],
                                             battery_mode_, fault_bitmask_);
* SRx_RC_CHAN
    * `mavlink_msg_servo_output_raw_pack`(sys_id_, comp_id_, &msg_,
                                               sys_time_us_, port_,
                                               servo_raw_[0], servo_raw_[1],
                                               servo_raw_[2], servo_raw_[3],
                                               servo_raw_[4], servo_raw_[5],
                                               servo_raw_[6], servo_raw_[7],
                                               servo_raw_[8], servo_raw_[9],
                                               servo_raw_[10], servo_raw_[11],
                                               servo_raw_[12], servo_raw_[13],
                                               servo_raw_[14], servo_raw_[15]);
    * `mavlink_msg_rc_channels_pack`(sys_id_, comp_id_, &msg_,
                                          sys_time_ms_, chancount_,
                                          chan_[0], chan_[1],
                                          chan_[2], chan_[3],
                                          chan_[4], chan_[5],
                                          chan_[6], chan_[7],
                                          chan_[8], chan_[9],
                                          chan_[10], chan_[11],
                                          chan_[12], chan_[13],
                                          chan_[14], chan_[15],
                                          chan_[16], chan_[17], rssi_);
* SRx_POSITION
    * `mavlink_msg_local_position_ned_pack`(sys_id_, comp_id_, &msg_,
                                                 sys_time_ms_,
                                                 nav_north_pos_m_,
                                                 nav_east_pos_m_,
                                                 nav_down_pos_m_,
                                                 nav_north_vel_mps_,
                                                 nav_east_vel_mps_,
                                                 nav_down_vel_mps_);
    * `mavlink_msg_global_position_int_pack`(sys_id_, comp_id_, &msg_,
                                                  sys_time_ms_, lat_dege7_,
                                                  lon_dege7_, alt_msl_mm_,
                                                  alt_agl_mm_, vx_cmps_,
                                                  vy_cmps_, vz_cmps_,
                                                  hdg_cdeg_);
* SRx_EXTRA1
    * `mavlink_msg_attitude_pack`(sys_id_, comp_id_, &msg_,
                                       sys_time_ms_, nav_roll_rad_,
                                       nav_pitch_rad_, yaw_rad_,
                                       nav_gyro_x_radps_, nav_gyro_y_radps_,
                                       nav_gyro_z_radps_);
* SRx_EXTRA2
    * `mavlink_msg_vfr_hud_pack`(sys_id_, comp_id_, &msg_,
                                      nav_ias_mps_, nav_gnd_spd_mps_,
                                      hdg_deg_, throttle_, nav_alt_msl_m_,
                                      climb_mps_);
* SRx_EXTRA3
    * `mavlink_msg_wind_cov_pack`(sys_id_, comp_id_, &msg_, sys_time_us_,
                                       wind_x_mps_, wind_y_mps_, wind_z_mps_,
                                       wind_var_horz_mps_, wind_var_vert_mps_,
                                       wind_alt_m_, wind_horz_acc_mps_,
                                       wind_vert_acc_mps_);
    * `mavlink_msg_system_time_pack`(sys_id_, comp_id_, &msg_,
                                          unix_time_us_, sys_time_ms_);

## More estoteric notes
Note to compatibility testers:  if you connect to MP the code configures itself for that GCS and won't work well with QGC without a restart.  (search for "planner" for the flag)                                          
