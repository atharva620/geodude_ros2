// Copyright 2024 Personal Robotics Lab, University of Washington
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

/* zero_barrett.cpp
 *
 * This is the simplest possible program that runs Barrett Technology's WAM Arm
 * using the libbarrett controls library. Use to set the WAM Zero.
 *
 * Before running this program, make sure the WAM is powered on and connected to
 * the Control PC. It can be either E-stopped or Idled.
 *
 */

#include <barrett/exception.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
  // Give us pretty stack-traces when things die
  ::barrett::installExceptionHandler();

  // Get "left" or "right" arm
  std::string arm(argv[argc - 1]);
  if (arm != "left" && arm != "right") {
    std::cout << "Usage: zero_barrett <left/right>" << std::endl;
    return -1;
  }

  // may throw ament_index_cpp::PackageNotFoundError exception
  std::string wam_config_file = ament_index_cpp::get_package_share_directory("geodude_hardware") +
                                "/config/" + arm + "_wam7w/default.conf";
  std::cout << "Loading Config File: " << wam_config_file << std::endl;

  ::barrett::ProductManager pm(wam_config_file.c_str());
  pm.waitForWam(true);  // true == prompt to zero
  pm.wakeAllPucks();
  pm.getWam7(false);  // false == no shift activate

  // Wait for the user to press Shift-idle
  pm.getSafetyModule()->waitForMode(::barrett::SafetyModule::IDLE);
  return 0;
}