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

#include <arpa/inet.h>  // For gethostbyaddr
#include <errno.h>
#include <netdb.h>  // gethostbyname
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <geodude_hardware/vention_api.hpp>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace geodude_hardware
{
static inline void verbosePrint(bool verbose, std::string str)
{
  if (verbose) {
    std::cout << "[VentionAPI] " << str << std::endl;
  }
}

static inline void errorPrint(std::string str) { std::cerr << "[VentionAPI] " << str << std::endl; }

VentionAPI::VentionAPI(bool verbose) : mVerbose(verbose), mTelnetSocket(-1)
{
  verbosePrint(verbose, "Init");
}

VentionAPI::~VentionAPI() { telnetDisconnect(); }

bool VentionAPI::telnetConnect(std::string ip_address, int port, suseconds_t recv_timeout)
{
  std::lock_guard<std::mutex> guard(mMutex);
  verbosePrint(mVerbose, "telnetConnect");

  if (mTelnetSocket >= 0) {
    verbosePrint(mVerbose, "Socket already exists.");
    return true;
  }

  /* 
  inet_aton is used to validate and convert the IP address from a string to a binary 
  format, which is then used to retrieve host information with gethostbyaddr
  */
  struct in_addr addr;
  if (inet_aton(ip_address.c_str(), &addr) == 0) {
    // Invalid IP address if inet_aton returns 0 or the conversion fails
    errorPrint(std::string("Invalid IP address: ") + ip_address);
    return false;
  }
  struct hostent * server = gethostbyaddr((const char *)&addr, sizeof(addr), AF_INET);
  if (server == NULL) {
    errorPrint(std::string("No such host by address: ") + ip_address);
    return false;
  }

  /*
  The following code initializes the sockaddr_in structure serv_addr with the server's address 
  and port, preparing it for use in a subsequent connect call to establish a 
  connection to the server.
  */

  // This structure is used to specify the address and port of the server.
  struct sockaddr_in serv_addr;

  /* 
  This line sets all bytes of the serv_addr structure to zero. 
  This ensures that there are no residual values in the structure that could cause unexpected behavior.
  */
  bzero((char *)&serv_addr, sizeof(serv_addr));

  /* 
  This sets the sin_family field of the serv_addr structure to AF_INET, 
  indicating that the address is an IPv4 address.
  */
  serv_addr.sin_family = AF_INET;

  /*
  htons stands for "host to network short" and converts a 16-bit number 
  from host byte order to network byte order (big-endian).
  */
  serv_addr.sin_port = htons(port);

  /*
  This copies the server's IP address from the server structure (obtained 
  from the gethostbyaddr function) to the sin_addr.s_addr field of the serv_addr structure.

  bcopy is a legacy function that copies a block of memory from one location to another. 
  It takes three arguments: a pointer to the source memory block, a pointer to the 
  destination memory block, and the number of bytes to copy.

  server->h_addr is a pointer to the server's IP address.

  server->h_length is the length of the IP address.
  */
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);

  // Initialize Socket
  verbosePrint(mVerbose, "Creating socket...");

  /* 
  This function creates a socket using the socket function with the parameters:
  AF_INET (IPv4), SOCK_STREAM (TCP), and protocol 0.
  */
  mTelnetSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (mTelnetSocket < 0) {
    errorPrint("Cannot init socket");
    return false;
  }

  // Set socket options
  struct timeval timeout;
  timeout.tv_sec = recv_timeout / 1000000;
  timeout.tv_usec = recv_timeout % 1000000;
  if (
    // Sets a receive timeout for the socket using the setsockopt function.
    setsockopt(mTelnetSocket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout)) <
    0) {
    errorPrint("Cannot set socket options");
    return false;
  }

  // Connect to the server
  verbosePrint(mVerbose, "Connecting to server...");
  // Attempts to connect to the server using the connect function with the serv_addr structure.
  if (connect(mTelnetSocket, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    errorPrint("Cannot connect to server");

    /*
    If the connection fails, it prints an error message, closes the socket, 
    sets mTelnetSocket to -1, and returns false.
    */
    close(mTelnetSocket);

    mTelnetSocket = -1;
    return false;
  }

  verbosePrint(mVerbose, "Connected to server");
  return true;
}

// Handles disconnecting from the Telnet server.
bool VentionAPI::telnetDisconnect()
{
  verbosePrint(mVerbose, "telnetDisconnect");
  std::lock_guard<std::mutex> guard(mMutex);

  try {
    if (mTelnetSocket >= 0) {
      ::shutdown(mTelnetSocket, SHUT_RDWR);
      close(mTelnetSocket);
      mTelnetSocket = -1;
    }
  } catch (...) {
    return false;
  }

  return true;
}

bool VentionAPI::telnetSend(std::string command)
{
  if (mTelnetSocket < 0) {
    return false;
  }

  // Check for and add carriage return
  if (command.substr(command.size() - 2) != "\r\n") {
    command += "\r\n";
  }

  try {
    verbosePrint(mVerbose, string_format("sending telnet command '%s'", command.c_str()));

    // size is big enough for all data packets
    char buffer[2048];

    int n;
    strncpy(buffer, command.c_str(), 2047);
    n = send(mTelnetSocket, buffer, strlen(buffer), MSG_NOSIGNAL | MSG_DONTWAIT);
    if (n < 0) {
      errorPrint(string_format("Error writing to telnet socket: %s", strerror(errno)));
      return false;
    } else {
      verbosePrint(mVerbose, string_format("socket write: sent %d bytes, ok.", n));
    }
  } catch (...) {
    return false;
  }

  return true;
}

bool VentionAPI::telnetRecv(std::string & response, bool blocking, bool print_error)
{
  response = "";
  if (mTelnetSocket < 0) {
    return false;
  }

  try {
    // size is big enough for all data packets
    char buffer[4096];

    // read from Vention
    bzero((char *)&buffer, 4096);

    // recv function is used to receive 4095 bytes of data from the Telnet socket.
    int n = recv(mTelnetSocket, buffer, 4095, (blocking ? 0 : MSG_DONTWAIT));
    if (n < 0) {
      if (print_error) {
        errorPrint(string_format("Error reading from telnet socket: %s", strerror(errno)));
      }
      response = "";
      return false;
    } else {
      response = std::string(buffer);
      verbosePrint(
        mVerbose, string_format("telnet socket read (%d bytes): %s", n, response.c_str()));
    }
  } catch (...) {
    return false;
  }

  return true;
}

// Vention socket API advertises this function as continuous movement, implying non-blocking.
bool VentionAPI::sendVelocityCommand(
  size_t axis, double velocity_mm_s, double accel_mm_s2, bool wait_for_ack)
{
  std::ostringstream command;
  if (mTelnetSocket < 0) {
    return false;
  }

  command << "SET im_conv_" << axis;
  command << " S" << std::fixed << std::setprecision(1) << velocity_mm_s;
  command << " A" << std::fixed << std::setprecision(1) << accel_mm_s2 << ";" << std::endl;

  // Sends the velocity command to the Vention actuator
  bool result = telnetSend(command.str());
  if (result && wait_for_ack) {
    std::string response;
    result = telnetRecv(response, true);  // blocking
    return result && (response == "Ack");
  }
  return result;
}

// Reads the current position of the Vention actuator
double VentionAPI::getPosition(size_t axis)
{
  double position = std::numeric_limits<double>::quiet_NaN();
  if (mTelnetSocket < 0) {
    return position;
  }

  std::ostringstream command;
  command << "GET im_get_controller_pos_axis_" << axis << ";" << std::endl;

  bool result = telnetSend(command.str());
  if (!result) {
    return position;
  }
  std::string response;
  result = telnetRecv(response, true);  // blocking
  if (!result) {
    return position;
  }
  std::istringstream respStream(response);
  while (!respStream.eof() && !respStream.fail()) {
    respStream.ignore(response.size(), '(');
    if (respStream.good()) respStream >> position;
  }
  return position;
}

bool VentionAPI::isMotionCompleted()
{
  if (mTelnetSocket < 0) {
    return false;
  }

  bool result = telnetSend("isMotionCompleted;");
  if (!result) {
    return false;
  }

  std::string response;
  result = telnetRecv(response, true, false);  // blocking, don't print error
  if (!result) {
    return false;
  }

  if (response.find("MachineMotion isMotionCompleted = true") != std::string::npos) {
    return true;
  }

  return false;
}

bool VentionAPI::isReady()
{
  if (mTelnetSocket < 0) {
    return false;
  }

  bool result = telnetSend("isReady;");
  if (!result) {
    return false;
  }

  std::string response;
  result = telnetRecv(response, true);  // blocking
  if (!result) {
    return false;
  }

  if (response.find("MachineMotion isReady = true") != std::string::npos) {
    return true;
  }

  return false;
}

bool VentionAPI::homeAxes(std::vector<size_t> axes)
{
  if (axes.size() < 1) {
    return false;
  }
  if (mTelnetSocket < 0) {
    return false;
  }

  std::ostringstream command;
  for (size_t axis : axes) {
    command << "im_home_axis_" << axis << ";";
  }
  command << std::endl;

  return telnetSend(command.str());
}

bool VentionAPI::setPosition(std::vector<size_t> axes, double position_mm, bool wait_for_ack)
{
  if (axes.size() < 1) {
    return false;
  }
  if (mTelnetSocket < 0) {
    return false;
  }

  std::ostringstream command;
  std::ostringstream expected;
  for (size_t axis : axes) {
    command << "SET im_set_controller_pos_axis_" << axis << "/";
    command << std::fixed << std::setprecision(1) << position_mm << "/;";
    expected << "Ack";
  }
  command << std::endl;

  bool result = telnetSend(command.str());
  if (result && wait_for_ack) {
    std::string response;
    result = telnetRecv(response, true);  // blocking
    return result && (response == expected.str());
  }
  return result;
}

std::vector<double> VentionAPI::getPos(std::vector<size_t> axes)
{
  std::vector<double> vention_states_;
  vention_states_.resize(2, std::numeric_limits<double>::quiet_NaN());
  if (mTelnetSocket < 0 || axes.size() < 1 || axes.size() > 2) {
    return vention_states_;
  }
  for (size_t axis : axes) {
    auto pos = getPosition(axis);
    vention_states_.push_back(pos);
  }
  return vention_states_;
}

bool VentionAPI::immediateAbsoluteMove(
  std::vector<size_t> axes, std::vector<double> vention_commands_, bool wait_for_ack)
{
  if (axes.size() < 1) {
    return false;
  }
  if (mTelnetSocket < 0) {
    return false;
  }

  std::ostringstream command;
  std::ostringstream expected;
  for (size_t axis : axes) {
    command << "SET im_move_abs_" << axis << "/";
    /* [axis-1] because the input vector is indexed from 0 where as 
    vention axis is indexed 1 for right actuator and 2 for left actuator*/ 
    command << std::fixed << std::setprecision(1) << vention_commands_[axis-1] << "/;";
    expected << "Ack";
  }
  command << std::endl;

  bool result = telnetSend(command.str());
  if (result && wait_for_ack) {
    std::string response;
    result = telnetRecv(response, true);  // blocking
    return result && (response == expected.str());
  }
  return result;
}

}  // End namespace geodude_hardware
