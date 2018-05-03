/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /kobuki_safety_controller/include/kobuki_safety_controller/safety_controller.hpp
 *
 * @brief Kobuki-specific safety controller
 *
 * This controller uses Kobuki's bumper, cliff and wheel drop sensors to ensure safe operation.
 *
 * @author Marcus Liebhardt, Yujin Robot
 *
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SAFETY_CONTROLLER_HPP_
#define SAFETY_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ros/ros.h>
#include <haros/haros.h>
#include <yocs_controllers/default_controller.hpp>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>

namespace kobuki
{

/**
 * @ brief Keeps track of safety-related events and commands Kobuki to move accordingly
 *
 * The SafetyController keeps track of bumper, cliff and wheel drop events. In case of the first two,
 * Kobuki is commanded to move back. In the latter case, Kobuki is stopped. All commands stop when the
 * event condition disappears. In the case of lateral bump/cliff, robot also spins a bit, what makes
 * easier to escape from the risk.
 *
 * This controller can be enabled/disabled.
 * The safety states (bumper pressed etc.) can be reset. WARNING: Dangerous!
 */
class SafetyController : public yocs::Controller
{
public:
  SafetyController(ros::NodeHandle& nh, std::string& name) :
    Controller(),
    nh_(nh),
    name_(name),
    wheel_left_dropped_(false),
    wheel_right_dropped_(false),
    bumper_left_pressed_(false),
    bumper_center_pressed_(false),
    bumper_right_pressed_(false),
    cliff_left_detected_(false),
    cliff_center_detected_(false),
    cliff_right_detected_(false), 
    last_event_time_(ros::Time(0)),
    msg_(new geometry_msgs::Twist()){};
  ~SafetyController(){};

  /**
   * Set-up necessary publishers/subscribers and variables
   * @return true, if successful
   */
  bool init()
  {
    //how long to keep sending messages after a bump, cliff, or wheel drop stops
    double time_to_extend_bump_cliff_events;
    nh_.param("time_to_extend_bump_cliff_events", time_to_extend_bump_cliff_events, 0.0);
    time_to_extend_bump_cliff_events_ = ros::Duration(time_to_extend_bump_cliff_events);
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &SafetyController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &SafetyController::disableCB, this);
    bumper_event_subscriber_ = nh_.subscribe("events/bumper", 10, &SafetyController::bumperEventCB, this);
    cliff_event_subscriber_  = nh_.subscribe("events/cliff",  10, &SafetyController::cliffEventCB, this);
    wheel_event_subscriber_  = nh_.subscribe("events/wheel_drop", 10, &SafetyController::wheelEventCB, this);
    reset_safety_states_subscriber_ = nh_.subscribe("reset", 10, &SafetyController::resetSafetyStatesCB, this);
    velocity_command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel", 10);
    return true;
  };

  /**
   * @ brief Checks safety states and publishes velocity commands when necessary
   */
  void spin();

private:
  ros::NodeHandle nh_;
  std::string name_;
  haros::Subscriber<std_msgs::Empty> enable_controller_subscriber_,
                                     disable_controller_subscriber_;
  haros::Subscriber<kobuki_msgs::BumperEvent> bumper_event_subscriber_;
  haros::Subscriber<kobuki_msgs::CliffEvent> cliff_event_subscriber_;
  haros::Subscriber<kobuki_msgs::WheelDropEvent> wheel_event_subscriber_;
  haros::Subscriber<std_msgs::Empty> reset_safety_states_subscriber_;
  haros::Publisher<geometry_msgs::Twist> velocity_command_publisher_;
  bool wheel_left_dropped_, wheel_right_dropped_;
  bool bumper_left_pressed_, bumper_center_pressed_, bumper_right_pressed_;
  bool cliff_left_detected_, cliff_center_detected_, cliff_right_detected_;
  ros::Duration time_to_extend_bump_cliff_events_;
  ros::Time last_event_time_;

  geometry_msgs::TwistPtr msg_; // velocity command

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void enableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief ROS logging output for disabling the controller
   * @param msg incoming topic message
   */
  void disableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief Keeps track of bumps
   * @param msg incoming topic message
   */
  void bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg);

  /**
   * @brief Keeps track of cliff detection
   * @param msg incoming topic message
   */
  void cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg);

  /**
   * @brief Keeps track of the wheel states
   * @param msg incoming topic message
   */
  void wheelEventCB(const kobuki_msgs::WheelDropEventConstPtr msg);

  /**
   * @brief Callback for resetting safety variables
   *
   * Allows resetting bumper, cliff and wheel drop states.
   * DANGEROUS!
   *
   * @param msg incoming topic message
   */
  void resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg);

  haros::MessageEvent<kobuki_msgs::WheelDropEvent> lastWheelDrop();
};


void SafetyController::enableCB(const std_msgs::EmptyConstPtr msg)
{
  // messages are empty and independent from previous ones or internal state
  ROS_ASSERT(true);
  if (this->enable())
  {
    ROS_INFO_STREAM("Controller has been enabled. [" << name_ << "]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"]");
  }
};

void SafetyController::disableCB(const std_msgs::EmptyConstPtr msg)
{
  // messages are empty and independent from previous ones or internal state
  ROS_ASSERT(true);
  if (this->disable())
  {
    ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
};

void SafetyController::cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg)
{
  ROS_ASSERT(msg->state == kobuki_msgs::CliffEvent::CLIFF
          || msg->state == kobuki_msgs::CliffEvent::FLOOR);
  ROS_ASSERT(msg->sensor == kobuki_msgs::CliffEvent::LEFT
          || msg->sensor == kobuki_msgs::CliffEvent::CENTER
          || msg->sensor == kobuki_msgs::CliffEvent::RIGHT);
  if (msg->state == kobuki_msgs::CliffEvent::CLIFF)
  {
    last_event_time_ = ros::Time::now();
    ROS_DEBUG_STREAM("Cliff detected. Moving backwards. [" << name_ << "]");
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:
        cliff_event_subscriber_.bookmark("left");
        cliff_left_detected_ = true;
        break;
      case kobuki_msgs::CliffEvent::CENTER:
        cliff_event_subscriber_.bookmark("center");
        cliff_center_detected_ = true;
        break;
      case kobuki_msgs::CliffEvent::RIGHT:
        cliff_event_subscriber_.bookmark("right");
        cliff_right_detected_ = true;
        break;
    }
  }
  else // kobuki_msgs::CliffEvent::FLOOR
  {
    ROS_DEBUG_STREAM("Not detecting any cliffs. Resuming normal operation. [" << name_ << "]");
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:
        cliff_event_subscriber_.bookmark("left");
        cliff_left_detected_ = false;
        break;
      case kobuki_msgs::CliffEvent::CENTER:
        cliff_event_subscriber_.bookmark("center");
        cliff_center_detected_ = false;
        break;
      case kobuki_msgs::CliffEvent::RIGHT:
        cliff_event_subscriber_.bookmark("right");
        cliff_right_detected_  = false;
        break;
    }
  }
};

void SafetyController::bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg)
{
  ROS_ASSERT(msg->state == kobuki_msgs::BumperEvent::PRESSED
          || msg->state == kobuki_msgs::BumperEvent::RELEASED);
  ROS_ASSERT(msg->bumper == kobuki_msgs::BumperEvent::LEFT
          || msg->bumper == kobuki_msgs::BumperEvent::CENTER
          || msg->bumper == kobuki_msgs::BumperEvent::RIGHT);
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    last_event_time_ = ros::Time::now();
    ROS_DEBUG_STREAM("Bumper pressed. Moving backwards. [" << name_ << "]");
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
        bumper_event_subscriber_.bookmark("left");
        bumper_left_pressed_ = true;
        break;
      case kobuki_msgs::BumperEvent::CENTER:
        bumper_event_subscriber_.bookmark("center");
        bumper_center_pressed_ = true;
        break;
      case kobuki_msgs::BumperEvent::RIGHT:
        bumper_event_subscriber_.bookmark("right");
        bumper_right_pressed_ = true;
        break;
    }
  }
  else // kobuki_msgs::BumperEvent::RELEASED
  {
    ROS_DEBUG_STREAM("No bumper pressed. Resuming normal operation. [" << name_ << "]");
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
        bumper_event_subscriber_.bookmark("left");
        bumper_left_pressed_ = false;
        break;
      case kobuki_msgs::BumperEvent::CENTER:
        bumper_event_subscriber_.bookmark("center");
        bumper_center_pressed_ = false;
        break;
      case kobuki_msgs::BumperEvent::RIGHT:
        bumper_event_subscriber_.bookmark("right");
        bumper_right_pressed_ = false;
        break;
    }
  }
};

void SafetyController::wheelEventCB(const kobuki_msgs::WheelDropEventConstPtr msg)
{
  ROS_ASSERT(msg->state == kobuki_msgs::WheelDropEvent::DROPPED
          || msg->state == kobuki_msgs::WheelDropEvent::RAISED);
  ROS_ASSERT(msg->wheel == kobuki_msgs::WheelDropEvent::LEFT
          || msg->wheel == kobuki_msgs::WheelDropEvent::RIGHT);
  if (msg->state == kobuki_msgs::WheelDropEvent::DROPPED)
  {
    // need to keep track of both wheels separately
    if (msg->wheel == kobuki_msgs::WheelDropEvent::LEFT)
    {
      ROS_DEBUG_STREAM("Left wheel dropped. [" << name_ << "]");
      wheel_event_subscriber_.bookmark("left");
      wheel_left_dropped_ = true;
    }
    else // kobuki_msgs::WheelDropEvent::RIGHT
    {
      ROS_DEBUG_STREAM("Right wheel dropped. [" << name_ << "]");
      wheel_event_subscriber_.bookmark("right");
      wheel_right_dropped_ = true;
    }
  }
  else // kobuki_msgs::WheelDropEvent::RAISED
  {
    // need to keep track of both wheels separately
    if (msg->wheel == kobuki_msgs::WheelDropEvent::LEFT)
    {
      ROS_DEBUG_STREAM("Left wheel raised. [" << name_ << "]");
      wheel_event_subscriber_.bookmark("left");
      wheel_left_dropped_ = false;
    }
    else // kobuki_msgs::WheelDropEvent::RIGHT
    {
      ROS_DEBUG_STREAM("Right wheel raised. [" << name_ << "]");
      wheel_event_subscriber_.bookmark("right");
      wheel_right_dropped_ = false;
    }
    if (!wheel_left_dropped_ && !wheel_right_dropped_)
    {
      ROS_DEBUG_STREAM("Both wheels raised. Resuming normal operation. [" << name_ << "]");
    }
  }
};

void SafetyController::resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg)
{
  // messages are empty and completely independent from previous ones
  ROS_ASSERT(true);
  wheel_left_dropped_    = false;
  wheel_right_dropped_   = false;
  bumper_left_pressed_   = false;
  bumper_center_pressed_ = false;
  bumper_right_pressed_  = false;
  cliff_left_detected_   = false;
  cliff_center_detected_ = false;
  cliff_right_detected_  = false;
  ROS_WARN_STREAM("All safety states have been reset to false. [" << name_ << "]");
}

void SafetyController::spin()
{
  if (this->getState())
  {
    //--------------------------------------------------------------------------
    // all of this is just to write shorter properties
    typedef haros::MessageEvent<kobuki_msgs::BumperEvent> Bumper;
    typedef haros::MessageEvent<kobuki_msgs::CliffEvent> Cliff;
    typedef haros::MessageEvent<kobuki_msgs::WheelDropEvent> Wheel;
    const uint8_t PRESSED = kobuki_msgs::BumperEvent::PRESSED;
    const uint8_t CLIFF = kobuki_msgs::CliffEvent::CLIFF;
    const uint8_t DROPPED = kobuki_msgs::WheelDropEvent::DROPPED;
    //--------------------------------------------------------------------------

    if (wheel_left_dropped_ || wheel_right_dropped_)
    {
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = 0.0;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.0;
      ROS_ASSERT(lastWheelDrop());  // a wheel drop exists
      velocity_command_publisher_.publish(msg_);
    }
    else if (bumper_center_pressed_ || cliff_center_detected_)
    {
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.0;
      {
        ROS_ASSERT(!lastWheelDrop()); // wheels are not dropped
        Bumper last_bumper = bumper_event_subscriber_.lastReceive("center");
        Cliff last_cliff = cliff_event_subscriber_.lastReceive("center");
        ROS_ASSERT((last_bumper && last_bumper.msg->state == PRESSED)
                || (last_cliff && last_cliff.msg->state == CLIFF));
      }
      velocity_command_publisher_.publish(msg_);
    }
    else if (bumper_left_pressed_ || cliff_left_detected_)
    {
      // left bump/cliff; also spin a bit to the right to make escape easier
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = -0.4;
      {
        ROS_ASSERT(!lastWheelDrop()); // wheels are not dropped
        Bumper center_bumper = bumper_event_subscriber_.lastReceive("center");
        Cliff center_cliff = cliff_event_subscriber_.lastReceive("center");
        ROS_ASSERT(!center_bumper || center_bumper.msg->state != PRESSED);
        ROS_ASSERT(!center_cliff || center_cliff.msg->state != CLIFF);
        Bumper left_bumper = bumper_event_subscriber_.lastReceive("left");
        Cliff left_cliff = cliff_event_subscriber_.lastReceive("left");
        ROS_ASSERT((left_bumper && left_bumper.msg->state == PRESSED)
                || (left_cliff && left_cliff.msg->state == CLIFF));
      }
      velocity_command_publisher_.publish(msg_);
    }
    else if (bumper_right_pressed_ || cliff_right_detected_)
    {
      // right bump/cliff; also spin a bit to the left to make escape easier
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.4;
      {
        ROS_ASSERT(!lastWheelDrop()); // wheels are not dropped
        Bumper center_bumper = bumper_event_subscriber_.lastReceive("center");
        Cliff center_cliff = cliff_event_subscriber_.lastReceive("center");
        ROS_ASSERT(!center_bumper || center_bumper.msg->state != PRESSED);
        ROS_ASSERT(!center_cliff || center_cliff.msg->state != CLIFF);
        Bumper left_bumper = bumper_event_subscriber_.lastReceive("left");
        Cliff left_cliff = cliff_event_subscriber_.lastReceive("left");
        ROS_ASSERT(!left_bumper || left_bumper.msg->state != PRESSED);
        ROS_ASSERT(!left_cliff || left_cliff.msg->state != CLIFF);
        Bumper right_bumper = bumper_event_subscriber_.lastReceive("right");
        Cliff right_cliff = cliff_event_subscriber_.lastReceive("right");
        ROS_ASSERT((right_bumper && right_bumper.msg->state == PRESSED)
                || (right_cliff && right_cliff.msg->state == CLIFF));
      }
      velocity_command_publisher_.publish(msg_);
    }
    //if we want to extend the safety state and we're within the time, just keep sending msg_
    else if (time_to_extend_bump_cliff_events_ > ros::Duration(1e-10) && 
        ros::Time::now() - last_event_time_ < time_to_extend_bump_cliff_events_) {
      {
        ROS_ASSERT(!lastWheelDrop()); // wheels are not dropped
        Bumper center_bumper = bumper_event_subscriber_.lastReceive("center");
        Cliff center_cliff = cliff_event_subscriber_.lastReceive("center");
        ROS_ASSERT(!center_bumper || center_bumper.msg->state != PRESSED);
        ROS_ASSERT(!center_cliff || center_cliff.msg->state != CLIFF);
        Bumper left_bumper = bumper_event_subscriber_.lastReceive("left");
        Cliff left_cliff = cliff_event_subscriber_.lastReceive("left");
        ROS_ASSERT(!left_bumper || left_bumper.msg->state != PRESSED);
        ROS_ASSERT(!left_cliff || left_cliff.msg->state != CLIFF);
        Bumper right_bumper = bumper_event_subscriber_.lastReceive("right");
        Cliff right_cliff = cliff_event_subscriber_.lastReceive("right");
        ROS_ASSERT(!right_bumper || right_bumper.msg->state != PRESSED);
        ROS_ASSERT(!right_cliff || right_cliff.msg->state != CLIFF);
      }
      velocity_command_publisher_.publish(msg_);
    }
  }
}

haros::MessageEvent<kobuki_msgs::WheelDropEvent> SafetyController::lastWheelDrop()
{
  const haros::MessageEvent<kobuki_msgs::WheelDropEvent> left =
      wheel_event_subscriber_.lastReceive("left");
  const haros::MessageEvent<kobuki_msgs::WheelDropEvent> right =
      wheel_event_subscriber_.lastReceive("right");
  const bool left_drop =
      left && left.msg->state == kobuki_msgs::WheelDropEvent::DROPPED;
  const bool right_drop =
      right && right.msg->state == kobuki_msgs::WheelDropEvent::DROPPED;
  if (left_drop && right_drop)
    { return left > right ? left : right; }
  if (left_drop)
    { return left; }
  if (right_drop)
    { return right; }
  return haros::MessageEvent<kobuki_msgs::WheelDropEvent>();
}

} // namespace kobuki

#endif /* SAFETY_CONTROLLER_HPP_ */
