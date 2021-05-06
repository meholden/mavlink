/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
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

#include "mavlink/mavlink.h"

std::array<bfs::MissionItem, 250> mission;
std::array<bfs::MissionItem, 250> fence;
std::array<bfs::MissionItem, 5> rally;
std::array<bfs::MissionItem, 250> temp;

bfs::MavLink<5> mavlink(&Serial4, bfs::AircraftType::FIXED_WING, mission.data(), mission.size(), fence.data(), fence.size(), rally.data(), rally.size(), temp.data());

int main() {
  Serial.begin(115200);
  while (!Serial) {}
  mavlink.Begin(57600);
  while (1) {
    mavlink.Update();
    if (mavlink.mission_updated()) {
      // Serial.println(mavlink.num_waypoints());
      // for (std::size_t i = 0; i < mavlink.num_waypoints(); i++) {
      //   Serial.print(mission[i].x);
      //   Serial.print("\t");
      //   Serial.print(mission[i].y);
      //   Serial.print("\t");
      //   Serial.print(mission[i].z);
      //   Serial.print("\t");
      //   Serial.print(temp[i].x);
      //   Serial.print("\t");
      //   Serial.print(temp[i].y);
      //   Serial.print("\t");
      //   Serial.println(temp[i].z);
      // }
    }
  }
}
