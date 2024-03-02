
/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
*  Modified by Michael E. Holden www.holdentechnology.com for further testing
*
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.dd
*/

/* MEH Notes:
* need to install the mavlink folder into the arduino/libraries folder (and uninstall other mavlink library)
* also need the units folder from bolder into the libraries folder
* also need elapsedMillis (library manager can do it) 
* also need to put the correct serial port in the setup function
*/

#include <elapsedMillis.h>
//#include <units.h>
#include <mavlink.h>

/*
* Storage for mission items (i.e. flight plans), fence vertices, rally points,
* and temporary storage, which is used to hold items during upload from the
* GCS and before the upload is verified good.
*/
std::array<bfs::MissionItem, 250> mission;
std::array<bfs::MissionItem, 250> fence;
std::array<bfs::MissionItem, 5> rally;
std::array<bfs::MissionItem, 250> temp;

/*
* A MavLink object with 5 parameters that can be tuned in real-time from the GCS
* and up to 10 simultaneous UTM messages received
*/
bfs::MavLink<5, 10> mavlink;

void setup() {


  /* Starting serial to print results */
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("hello there from mavlink example");

  /* Configuring MavLink serial port and aircraft type */
  mavlink.hardware_serial(&Serial4);
  //mavlink.hardware_serial(&SerialUSB1);
  mavlink.aircraft_type(bfs::FIXED_WING);
  /* Passing info about where to store mission data */
  mavlink.mission(mission.data(), mission.size(), temp.data());
  mavlink.fence(fence.data(), fence.size());
  mavlink.rally(rally.data(), rally.size());

  // Initialize some fake parameters
  // set home location to WP(0) as MP pulls on arming https://diydrones.com/forum/topics/failed-to-update-home-location-1
  mavlink.num_mission_items(1);
  mission[0].x = 380144567; // deg*1e7 point pinole ish
  mission[0].y = -1223526763; // deg*1e7
  mission[0].z = 0;

  /* Starting communication on the serial port */
  mavlink.Begin(57600);
}

void loop() {
  /* Needed to send and receive MavLink data */
  mavlink.Update();

  // make some sine wave things to change the inputs perXs is X second period
  // omega = 2*pi/T  where T in ms
  float per5s = cos((double)millis()*0.0012566); // 5 second period
  float per15s = cos((double)millis()*0.000418879);
  float per30s = cos((double)millis()*0.0002094395);
  float per30sin = sin((double)millis()*0.0002094395);

  float roll = 0.7*per5s; // rad
  float pitch = 0.3*per15s; // rad
  float yaw = 6*per30s; // rad

  float dx = 0.02*per30s;
  float dy = 0.01*per30sin;

  float latitude, longitude, alt, rel_alt;
  float vx, vy, vz, hdg;
  latitude = (38.014398 + dy)/57.3; // rad
  longitude = (-122.371464 + dx)/57.3; // rad
  alt = 200*per30sin; // m
  rel_alt = 1; // m
  vx = vy = vz = 0; // 
//  hdg = 180e2; // cdeg


  // Mode (heartbeat)
  mavlink.throttle_enabled(true);  // armed
  mavlink.aircraft_state(bfs::ACTIVE);  // 
  mavlink.aircraft_mode(bfs::AUTO);  // reads as Stabilize + Guided on QGC
  mavlink.custom_mode(11); // for mission planner see https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html.  0=manual, 10-auto, 11=rtl

  // RAW_SENS
  //--mavlink_msg_scaled_imu_pack
  //  skipping for now, check readme + telemetry.cpp to see if you want those parameters

  //--mavlink_msg_gps_raw_int_pack
  mavlink.gnss_fix(GPS_FIX_TYPE_3D_FIX);
  mavlink.gnss_lat_rad(latitude);
  mavlink.gnss_lon_rad(longitude);
  mavlink.gnss_alt_msl_m(alt+500);
  mavlink.gnss_hdop(8);
  mavlink.gnss_vdop(1);
  mavlink.gnss_vel_acc_mps(0);
  
  //--mavlink_msg_scaled_pressure_packc:\Users\holde\OneDrive\Documents\Arduino\libraries\mavlink\src\telemetry.cpp
  //  skipping for now, check readme + telemetry.cpp to see if you want those parameters


  //  EXT_STAT
  //--mavlink_msg_sys_status_pack
  mavlink.gyro_installed(true);  // 
  mavlink.accel_installed(true);
  mavlink.mag_installed(true);
  mavlink.gyro_healthy(true);
  mavlink.accel_healthy(true);
  mavlink.mag_healthy(true);
  mavlink.battery_volt(11.4);
  mavlink.battery_current_ma(420);
  mavlink.battery_remaining_prcnt(92);

  //  RC_CHAN
  //--mavlink_msg_servo_output_raw_pack`(sys_id_, comp_id_, &msg_,
  std::array<float,16> pctsvo={0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
  pctsvo[0] = 0.5+0.5*per5s; // wiggle something
  mavlink.effector(pctsvo);

  //  mavlink_msg_rc_channels_pack
  std::array<float,16> rcin = {0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
  mavlink.inceptor(rcin);

  //  POSITION
  //--mavlink_msg_global_position_int_pack
  mavlink.nav_lat_rad(latitude);
  mavlink.nav_lon_rad(longitude);
  mavlink.nav_alt_agl_m(alt);
  mavlink.nav_alt_msl_m(alt+500);
  mavlink.nav_down_vel_mps(vz);
  mavlink.nav_east_vel_mps(vx);
  mavlink.nav_north_vel_mps(vy);
  
  //--mavlink_msg_local_position_ned_pack
  //  skipping for now, check readme + telemetry.cpp to see if you want those parameters


  // EXTRA1 set variables
  //--mavlink_msg_attitude_pack
  mavlink.nav_roll_rad(roll);
  mavlink.nav_pitch_rad(pitch);
  mavlink.nav_hdg_rad(yaw);
  mavlink.nav_gyro_x_radps(0);
  mavlink.nav_gyro_y_radps(0);
  mavlink.nav_gyro_z_radps(0);


  /* Check to see if the mission has been updated and print mission items */
  if (mavlink.mission_updated()) {
    Serial.println(mavlink.num_mission_items());
    for (std::size_t i = 0; i < mavlink.num_mission_items(); i++) {
      Serial.print(mission[i].x);
      Serial.print("\t");
      Serial.print(mission[i].y);
      Serial.print("\t");
      Serial.print(mission[i].z);
      Serial.print("\n");
    }
  }

  /* Check to see if the parameters have been updated and print */
  int up_p = mavlink.updated_param();
  if (up_p>0) {
    Serial.print(up_p);
    Serial.print(" changed to: ");
    Serial.print(mavlink.param(up_p));
    Serial.println("  got new parameters!");

  }

}
