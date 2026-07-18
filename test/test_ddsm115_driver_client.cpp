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

using namespace ddsm115_ros2_driver;

class TestDDSM115DriverClient : public DDSM115DriverClient {
public:
  TestDDSM115DriverClient(FeedbackCallback fb, LogCallback lb)
  : DDSM115DriverClient(fb, lb) {}

  // Expose protected methods for testing
  using DDSM115DriverClient::process_feedback_packet;
};

TEST(DDSM115DriverClientTest, calc_crc8_maxim_official_test_vectors)
{
  const uint8_t id_set_packet[] = {0xAA, 0x55, 0x53, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(DDSM115DriverClient::calc_crc8_maxim(id_set_packet, 9), 0xCB);

  const uint8_t mode_switch_packet[] = {0x01, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(DDSM115DriverClient::calc_crc8_maxim(mode_switch_packet, 9), 0xE4);

  const uint8_t velocity_zero_packet[] = {0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(DDSM115DriverClient::calc_crc8_maxim(velocity_zero_packet, 9), 0x50);

  const uint8_t velocity_hundred_packet[] = {0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(DDSM115DriverClient::calc_crc8_maxim(velocity_hundred_packet, 9), 0x4F);
}

TEST(DDSM115DriverClientTest, decode_feedback_normal_packet)
{
  MotorFeedback parsed_fb;
  auto fb_cb = [&](const MotorFeedback& fb) { parsed_fb = fb; };
  TestDDSM115DriverClient client(fb_cb, nullptr);

  // Mock normal feedback packet from wiki (0x64 mode)
  // ID(01) Mode(02) Current(00 10 -> 16 = 0.0039A) Vel(00 32 -> 50rpm) Pos(27 10 -> 10000 -> 109.8deg) Err(00) CRC
  std::vector<uint8_t> packet = {
    0x01, 0x02, 
    0x00, 0x10, 
    0x00, 0x32, 
    0x27, 0x10, 
    0x00, 0x00
  };
  client.process_feedback_packet(packet);

  EXPECT_EQ(parsed_fb.motor_id, 1);
  EXPECT_EQ(parsed_fb.mode, ControlLoopModes::MODE_VELOCITY);
  EXPECT_NEAR(parsed_fb.current, 16.0 * (8.0 / 32767.0), 0.0001);
  EXPECT_DOUBLE_EQ(parsed_fb.velocity, 50.0);
  EXPECT_NEAR(parsed_fb.position, 10000.0 * (360.0 / 32767.0), 0.01);
  EXPECT_FALSE(parsed_fb.is_temperature_packet);
  EXPECT_EQ(parsed_fb.error_code, 0);
  EXPECT_FALSE(parsed_fb.sensor_fault);
  EXPECT_FALSE(parsed_fb.over_current);
}

TEST(DDSM115DriverClientTest, decode_feedback_error_bits)
{
  MotorFeedback parsed_fb;
  auto fb_cb = [&](const MotorFeedback& fb) { parsed_fb = fb; };
  TestDDSM115DriverClient client(fb_cb, nullptr);

  // Mock error packet
  // Error bitmask 0x1B = 0001 1011 (BIT0, BIT1, BIT3, BIT4)
  std::vector<uint8_t> packet = {
    0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x00
  };
  client.process_feedback_packet(packet);

  EXPECT_EQ(parsed_fb.error_code, 0x1B);
  EXPECT_TRUE(parsed_fb.sensor_fault);
  EXPECT_TRUE(parsed_fb.over_current);
  EXPECT_FALSE(parsed_fb.phase_over_current);
  EXPECT_TRUE(parsed_fb.stalling);
  EXPECT_TRUE(parsed_fb.troubleshooting);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
