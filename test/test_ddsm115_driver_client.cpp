// Copyright 2026 Tatsukiyano
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include "ddsm115_ros2_driver/ddsm115_driver_client.hpp"

TEST(DDSM115DriverClientTest, calc_crc8_maxim_official_test_vectors)
{
  // Official test vectors from Waveshare Wiki
  // 1. ID setting (01) sync header packet (AA 55 53 01 00 00 00 00 00) -> Checksum: CB
  const uint8_t id_set_packet[] = {0xAA, 0x55, 0x53, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(ddsm115_ros2_driver::DDSM115DriverClient::calc_crc8_maxim(id_set_packet, 9), 0xCB);

  // 2. Velocity loop mode switch packet (01 A0 02 00 00 00 00 00 00) -> Checksum: E4
  const uint8_t mode_switch_packet[] = {0x01, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(
    ddsm115_ros2_driver::DDSM115DriverClient::calc_crc8_maxim(mode_switch_packet, 9), 0xE4);

  // 3. Velocity loop command 0 RPM (01 64 00 00 00 00 00 00 00) -> Checksum: 50
  const uint8_t velocity_zero_packet[] = {0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(
    ddsm115_ros2_driver::DDSM115DriverClient::calc_crc8_maxim(velocity_zero_packet, 9), 0x50);

  // 4. Velocity loop command 100 RPM (01 64 00 64 00 00 00 00 00) -> Checksum: 4F
  const uint8_t velocity_hundred_packet[] = {0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(
    ddsm115_ros2_driver::DDSM115DriverClient::calc_crc8_maxim(velocity_hundred_packet, 9), 0x4F);

  // 5. Current loop command 5000 (01 64 13 88 00 00 00 00 00) -> Checksum: A7
  const uint8_t current_5000_packet[] = {0x01, 0x64, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(
    ddsm115_ros2_driver::DDSM115DriverClient::calc_crc8_maxim(current_5000_packet, 9), 0xA7);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
