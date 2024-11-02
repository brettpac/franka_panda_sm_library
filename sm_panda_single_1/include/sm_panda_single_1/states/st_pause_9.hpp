// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 *****************************************************************************************************************/

#pragma once

#include <sensor_msgs/msg/joint_state.h>
#include <smacc2/client_behaviors/cb_sleep_for.hpp>

namespace sm_panda_single_1
{
// SMACC2 classes
using smacc2::EvStateRequestFinish;
using smacc2::Transition;
using smacc2::default_transition_tags::SUCCESS;
using namespace smacc2;
using namespace cl_moveit2z;
using smacc2::client_behaviors::CbWaitTopicMessage;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using namespace cl_keyboard;

// STATE DECLARATION
struct StPause9 : smacc2::SmaccState<StPause9, SmPandaSingle1>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef boost::mpl::list<
    Transition<EvCbSuccess<CbSleepFor, OrArm>, StUndoLastTrajectory, SUCCESS>,
    
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StUndoLastTrajectory, NEXT>  
  

    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // configure_orthogonal<OrArm, CbWaitTopicMessage<sensor_msgs::msg::JointState>>("/joint_states");
    configure_orthogonal<OrArm, CbSleepFor>(15s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  };

  void runtimeConfigure() {}
};
}  // namespace sm_panda_single_1
