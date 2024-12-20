// Copyright 2023 Personal Robotics Lab, University of Washington
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// Author: Ethan K. Gordon
// Based on https://github.com/personalrobotics/forque_sensor_hardware

#ifndef GEODUDE_HARDWARE_VENTIONAPI_H_
#define GEODUDE_HARDWARE_VENTIONAPI_H_

#include <sys/time.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Default Port Numbers
#define DEFAULT_TELNET_PORT 9999

// Other Configurations
#define DEFAULT_RECV_TIMEOUT_US 100000

namespace geodude_hardware
{
/*
Vention API is a Telnet interface that handles low-level send and recieve 
commands to and from the MachineMotionV2/Vention control machine over the Ethernet.
*/
class VentionAPI
{
public:
  VentionAPI(bool verbose = false);

  // Constants
  const static size_t ARM_LEFT = 2;
  const static size_t ARM_RIGHT = 1;

  // Disconnect/close all sockets.
  ~VentionAPI();

  // Connect to telnet socket on given hostname and port
  // Returns 0 on success, -1 on failure
  bool telnetConnect(
    std::string hostname, int port = DEFAULT_TELNET_PORT,
    suseconds_t recv_timeout = DEFAULT_RECV_TIMEOUT_US);
  bool telnetDisconnect();

  // Send a velocity command in mm/s (at max acceleration mm/s^2)
  // Axis is 1-indexed
  // Return false on error
  // If wait_for_ack, could block for up to recv_timeout
  bool sendVelocityCommand(
    size_t axis, double velocity_mm_s, double accel_mm_s2, bool wait_for_ack = false);

  // Get Encoder relative positions
  // Block until all positions are received (likely up to recv_timeout)
  // Return NaN on failure and print to strerr.
  double getPosition(size_t axis);

  std::vector<double> getPos(std::vector<size_t> axes);

  // Commands for lower-frequency control
  // See if motion is Completed Or Not
  // Blocks until response
  bool isMotionCompleted();

  // See if interface is ready or not.
  // Blocks until response
  bool isReady();

  // Non-blocking Home Command
  // Axes is 1-indexed
  bool homeAxes(std::vector<size_t> axes);

  // Sets the Relative Encoder current position to the specified value in mm
  // If wait_for_ack, could block for up to recv_timeout
  bool setPosition(std::vector<size_t> axes, double position_mm, bool wait_for_ack = true);

  // Mostly for internal use
  // Send command, return false (and print strerror to stderr) on failure.
  bool telnetSend(std::string command);
  // Receive response, optionally block
  // Call and ignore result to flush receive buffer.
  bool telnetRecv(std::string & response, bool blocking = false, bool print_error = true);

  bool immediateAbsoluteMove(
    std::vector<size_t> axes, std::vector<double> vention_commands_, bool wait_for_ack);

private:
  bool mVerbose;

  // networking socket
  int mTelnetSocket;

  // thread safety
  std::mutex mMutex;

};  // end class VentionAPI

// String Formatting Utility
template <typename... Args>
std::string string_format(const std::string & format, Args... args)
{
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;  // Extra space for '\0'
  if (size_s <= 0) {
    throw std::runtime_error("Error during formatting.");
  }
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
}

};  // namespace geodude_hardware

#endif