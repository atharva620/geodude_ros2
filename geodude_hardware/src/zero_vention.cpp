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

/*
zero_vention.cpp

This is a simple program that connects to the Vention Actuators and moves them
to zero position. The zero position is the top most position the actuators can
reach.
*/

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "geodude_hardware/vention_api.hpp"

using std::chrono::high_resolution_clock;
using namespace std::chrono_literals;
using namespace geodude_hardware;

int main(int argc, char ** argv)
{
  std::cout << "Zeroing Vention Actuators" << std::endl;
  VentionAPI vention(false);  // not verbose

  // Connect to Vention
  std::cout << "Initialize Telnet API...";
  auto start = high_resolution_clock::now();
  std::string ip_address = "192.168.1.152";
  bool result = vention.telnetConnect(ip_address);
  auto end = high_resolution_clock::now();
  if (!result) {
    std::cerr << "Could not connect to vention." << std::endl;
    return -1;
  }
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cout << "Done! (" << duration_ms.count() << "ms)" << std::endl;
  // Flush Receive Buffer
  std::cout << "Flushing Receive Buffer for startup response...";
  std::string ignore;
  start = high_resolution_clock::now();
  vention.telnetRecv(ignore, true);  // blocking
  end = high_resolution_clock::now();
  duration_ms = end - start;
  std::cout << "Done! (" << duration_ms.count() << "ms)" << std::endl;

  // Check if ready
  std::cout << "IsReady?...";
  start = high_resolution_clock::now();
  result = vention.isReady();
  end = high_resolution_clock::now();
  if (!result) {
    std::cerr << "No." << std::endl;
    return -1;
  }
  duration_ms = end - start;
  std::cout << "Yes! (" << duration_ms.count() << "ms)" << std::endl;

  std::string actuator(argv[argc - 1]);
  // Home Axes
  std::cout << "Command Home...";
  start = high_resolution_clock::now();

  if (actuator != "left" && actuator != "right") {
    // home both vention actuators if no argument is passed
    result = vention.homeAxes(std::vector<size_t>{VentionAPI::ARM_LEFT, VentionAPI::ARM_RIGHT});
  } else if (actuator == "left") {
    result = vention.homeAxes(std::vector<size_t>{VentionAPI::ARM_LEFT});
  } else if (actuator == "right") {
    result = vention.homeAxes(std::vector<size_t>{VentionAPI::ARM_RIGHT});
  }

  end = high_resolution_clock::now();
  if (!result) {
    std::cerr << "Error." << std::endl;
    return -1;
  }
  duration_ms = end - start;
  std::cout << "Done! (" << duration_ms.count() << "ms)" << std::endl;

  // Wait for MoveConnected
  std::cout << "Waiting for completion...";
  start = high_resolution_clock::now();
  while (!vention.isMotionCompleted()) {
    std::this_thread::sleep_for(1s);
  }
  end = high_resolution_clock::now();
  duration_ms = end - start;
  std::cout << "Done! (" << duration_ms.count() << "ms)" << std::endl;

  // Flush Receive Buffer
  std::cout << "Flushing Receive Buffer Twice (1 for each Home)...";
  start = high_resolution_clock::now();
  vention.telnetRecv(ignore, true, false);  // blocking
  vention.telnetRecv(ignore, true, false);  // blocking
  end = high_resolution_clock::now();
  duration_ms = end - start;
  std::cout << "Done! (" << duration_ms.count() << "ms)" << std::endl;

  // Disconnect from Vention
  std::cout << "Disconnect Telnet API...";
  start = high_resolution_clock::now();
  result = vention.telnetDisconnect();
  end = high_resolution_clock::now();
  if (!result) {
    std::cerr << "Error disconnecting." << std::endl;
    return -1;
  }
  duration_ms = end - start;
  std::cout << "Done! (" << duration_ms.count() << "ms)" << std::endl;
  return 0;
}
