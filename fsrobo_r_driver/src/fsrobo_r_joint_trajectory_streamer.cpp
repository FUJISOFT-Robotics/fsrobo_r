/*********************************************************************
* FSRobo-R Package BSDL
* ---------
* Copyright (C) 2019 FUJISOFT. All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "fsrobo_r_driver/fsrobo_r_joint_trajectory_streamer.h"
#include <sstream>

namespace fsrobo_r_driver
{
namespace joint_trajectory_streamer
{
bool FSRoboRJointTrajectoryStreamer::init(SmplMsgConnection *connection, const std::vector<std::string> &joint_names,
                                   const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("FSRoboRJointTrajectoryStreamer: init");

  rtn &= JointTrajectoryInterface::init(connection, joint_names, velocity_limits);

  this->mutex_.lock();
  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;
  this->streaming_thread_ =
      new boost::thread(boost::bind(&FSRoboRJointTrajectoryStreamer::streamingThread, this));
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();


  io_ctrl_.init(connection);
  this->srv_set_io = this->node_.advertiseService("set_io", &FSRoboRJointTrajectoryStreamer::setIOCB, this);

  robot_program_executor_.init(connection);
  this->srv_execute_robot_program = this->node_.advertiseService("execute_robot_program", &FSRoboRJointTrajectoryStreamer::executeRobotProgramCB, this);

  robot_configurator_.init(connection);
  this->srv_set_posture = this->node_.advertiseService("set_posture", &FSRoboRJointTrajectoryStreamer::setPostureCB, this);
  this->srv_get_posture = this->node_.advertiseService("get_posture", &FSRoboRJointTrajectoryStreamer::getPostureCB, this);
  this->srv_set_tool_offset = this->node_.advertiseService("set_tool_offset", &FSRoboRJointTrajectoryStreamer::setToolOffsetCB, this);

  return rtn;
}
FSRoboRJointTrajectoryStreamer::~FSRoboRJointTrajectoryStreamer()
{
  delete this->streaming_thread_;
}

void FSRoboRJointTrajectoryStreamer::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

    this->mutex_.lock();
    trajectoryStop();
    this->mutex_.unlock();
    return;
  }

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, send abort command anyway.");
    this->mutex_.lock();
    trajectoryStop();
    this->mutex_.unlock();
    return;
  }

  // calc new trajectory
  std::vector<JointTrajPtMessage> new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

bool FSRoboRJointTrajectoryStreamer::send_to_robot(const std::vector<JointTrajPtMessage> &messages)
{
  ROS_INFO("Loading trajectory, setting state to streaming");
  this->mutex_.lock();
  {
    ROS_INFO("Executing trajectory of size: %d", (int)messages.size());
    this->current_traj_ = messages;
    this->current_point_ = 0;
    this->state_ = TransferStates::STREAMING;
    this->streaming_start_ = ros::Time::now();
  }
  this->mutex_.unlock();

  return true;
}

bool FSRoboRJointTrajectoryStreamer::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector<JointTrajPtMessage> *msgs)
{
  // use base function to transform points
  if (!JointTrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", (int)msgs->size(), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}

void FSRoboRJointTrajectoryStreamer::streamingThread()
{
  JointTrajPtMessage jtpMsg;
  int connectRetryCount = 1;

  ROS_INFO("Starting joint trajectory streamer thread");
  while (ros::ok())
  {
    ros::Duration(0.001).sleep();

    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      ROS_INFO("Connecting to robot motion server");
      this->connection_->makeConnect();
      ros::Duration(0.250).sleep(); // wait for connection

      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        ROS_ERROR("Timeout connecting to robot controller.  Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }

    this->mutex_.lock();

    SimpleMessage msg, reply;

    switch (this->state_)
    {
    case TransferStates::IDLE:
      //ros::Duration(0.250).sleep(); //  slower loop while waiting for new trajectory
      break;

    case TransferStates::STREAMING:
      if (this->current_point_ >= (int)this->current_traj_.size())
      {
        ROS_INFO("Trajectory streaming complete, setting state to IDLE");
        this->state_ = TransferStates::IDLE;
        break;
      }

      if (this->current_point_ == 0 && (int)this->current_traj_.size() > 1)
      {
        ROS_INFO("First trajectory remove");
        this->current_point_++;
        break;
      }

      if (!this->connection_->isConnected())
      {
        ROS_DEBUG("Robot disconnected.  Attempting reconnect...");
        connectRetryCount = 5;
        break;
      }

      // check bufer filling
      {
        SimpleMessage req, res;
        fsrobo_r_driver::simple_message::sys_stat::SysStat stat;
        fsrobo_r_driver::simple_message::sys_stat_reply::SysStatReply stat_reply;
        stat.init(fsrobo_r_driver::simple_message::sys_stat::SysStatType::MOTION_REQUEST, 0);
        fsrobo_r_driver::simple_message::sys_stat_message::SysStatMessage stat_msg;
        fsrobo_r_driver::simple_message::sys_stat_reply_message::SysStatReplyMessage stat_reply_msg;
        stat_msg.init(stat);
        stat_msg.toRequest(req);
        if(!this->connection_->sendAndReceiveMsg(req, res)) {
          ROS_WARN("Failed to send SysStat Request message");
          break;
        }
        stat_reply_msg.init(res);

        stat_reply.copyFrom(stat_reply_msg.reply_);
        // don't send move command if buffer is full.
        if (stat_reply.getResult() >= 4) {
          ROS_WARN("using buffer size: %d", stat_reply.getResult());
          break;
        }

      }

      jtpMsg = this->current_traj_[this->current_point_];
      jtpMsg.toRequest(msg);
      {
        ros::Duration x(jtpMsg.point_.getDuration());
      }

      ROS_DEBUG("Sending joint trajectory point");
      if (this->connection_->sendAndReceiveMsg(msg, reply, false))
      {
        ROS_INFO("Point[%d of %d] sent to controller",
                 this->current_point_, (int)this->current_traj_.size());
        this->current_point_++;
      }
      else
        ROS_WARN("Failed sent joint point, will try again");

      break;
    default:
      ROS_ERROR("Joint trajectory streamer: unknown state");
      this->state_ = TransferStates::IDLE;
      break;
    }

    this->mutex_.unlock();
  }

  ROS_WARN("Exiting trajectory streamer thread");
}

void FSRoboRJointTrajectoryStreamer::trajectoryStop()
{
  JointTrajectoryInterface::trajectoryStop();

  ROS_DEBUG("Stop command sent, entering idle mode");
  this->state_ = TransferStates::IDLE;
}


bool FSRoboRJointTrajectoryStreamer::setIOCB(fsrobo_r_msgs::SetIO::Request &req, fsrobo_r_msgs::SetIO::Response &res)
{
  ROS_WARN("SetIO!");

  industrial::shared_types::shared_int io_val = -1;

  std::stringstream ss;
  std::vector<industrial::shared_types::shared_int> data;
  for (int x : req.data)
  {
    data.push_back(x);
    ss << x;
    ss << " ";
  }
  ROS_WARN("%s", ss.str().c_str());

  this->mutex_.lock();
  bool result = io_ctrl_.setIO(req.fun, req.address, data);
  this->mutex_.unlock();

  res.success = result;

  if (!result)
  {
    ROS_ERROR("Writing IO element %d failed", req.address);
    return false;
  }

  return true;
}

bool FSRoboRJointTrajectoryStreamer::executeRobotProgramCB(fsrobo_r_msgs::ExecuteRobotProgram::Request &req, fsrobo_r_msgs::ExecuteRobotProgram::Response &res)
{
  ROS_WARN("ExecuteRobotProgram!");

  ROS_WARN("%s", req.name.c_str());
  bool exec_result;

  this->mutex_.lock();
  bool result = robot_program_executor_.execute(req.name, req.param, exec_result);
  this->mutex_.unlock();

  res.result = exec_result;

  if (!result)
  {
    ROS_ERROR("Executing robot program %s failed", req.name.c_str());
    return false;
  }

  return true;
}

bool FSRoboRJointTrajectoryStreamer::setPostureCB(fsrobo_r_msgs::SetPosture::Request &req, fsrobo_r_msgs::SetPosture::Response &res)
{
  ROS_WARN("SetPosture!");

  bool exec_result;

  this->mutex_.lock();
  bool send_result = robot_configurator_.setPosture(req.posture, exec_result);
  this->mutex_.unlock();

  bool result = send_result && exec_result;

  if (!result)
  {
    ROS_ERROR("Setting Posture failed");
    return false;
  }

  return true;
}

bool FSRoboRJointTrajectoryStreamer::getPostureCB(fsrobo_r_msgs::GetPosture::Request &req, fsrobo_r_msgs::GetPosture::Response &res)
{
  ROS_WARN("GetPosture!");

  bool exec_result;
  industrial::shared_types::shared_int posture;

  this->mutex_.lock();
  bool send_result = robot_configurator_.getPosture(posture, exec_result);
  this->mutex_.unlock();

  res.posture = posture;
  bool result = send_result && exec_result;

  if (!result)
  {
    ROS_ERROR("Getting posture failed");
    return false;
  }

  return true;
}

bool FSRoboRJointTrajectoryStreamer::setToolOffsetCB(fsrobo_r_msgs::SetToolOffset::Request &req, fsrobo_r_msgs::SetToolOffset::Response &res)
{
  ROS_WARN("SetToolOffset!");

  bool exec_result;

  this->mutex_.lock();
  bool send_result = robot_configurator_.setToolOffset(req.origin.x, req.origin.y, req.origin.z, req.rotation.z, req.rotation.y, req.rotation.x, exec_result);
  this->mutex_.unlock();

  bool result = send_result && exec_result;

  if (!result)
  {
    ROS_ERROR("Setting tool offset failed");
    return false;
  }

  return true;
}

} // namespace joint_trajectory_streamer
} // namespace fsrobo_r_driver