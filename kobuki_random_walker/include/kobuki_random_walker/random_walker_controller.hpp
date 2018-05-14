/**
 * @file /kobuki_controller_tutorial/src/nodelet.cpp
 *
 * @brief A controller implementing a simple random walker algorithm
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_random_walker/LICENSE
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef RANDOM_WALKER_CONTROLLER_HPP_
#define RANDOM_WALKER_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <ecl/threads/mutex.hpp>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <ros/ros.h>
#include <haros/haros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>

namespace kobuki
{

#ifndef KOBUKI_MESSAGE_PREDICATES_
#define KOBUKI_MESSAGE_PREDICATES_
bool isLeftBumper(const kobuki_msgs::BumperEvent& msg)
{
  return msg.bumper == kobuki_msgs::BumperEvent::LEFT;
}

bool isCenterBumper(const kobuki_msgs::BumperEvent& msg)
{
  return msg.bumper == kobuki_msgs::BumperEvent::CENTER;
}

bool isRightBumper(const kobuki_msgs::BumperEvent& msg)
{
  return msg.bumper == kobuki_msgs::BumperEvent::RIGHT;
}

bool isLeftCliff(const kobuki_msgs::CliffEvent& msg)
{
  return msg.sensor == kobuki_msgs::CliffEvent::LEFT;
}

bool isCenterCliff(const kobuki_msgs::CliffEvent& msg)
{
  return msg.sensor == kobuki_msgs::CliffEvent::CENTER;
}

bool isRightCliff(const kobuki_msgs::CliffEvent& msg)
{
  return msg.sensor == kobuki_msgs::CliffEvent::RIGHT;
}

bool isLeftWheel(const kobuki_msgs::WheelDropEvent& msg)
{
  return msg.wheel == kobuki_msgs::WheelDropEvent::LEFT;
}

bool isRightWheel(const kobuki_msgs::WheelDropEvent& msg)
{
  return msg.wheel == kobuki_msgs::WheelDropEvent::RIGHT;
}
#endif

/**
 * @ brief A controller implementing a simple random walker algorithm
 *
 * Controller moves the robot around, changing direction whenever a bumper or cliff event occurs
 * For changing direction random angles are used.
 */
class RandomWalkerController : public yocs::Controller
{
public:
  RandomWalkerController(ros::NodeHandle& nh_priv, std::string& name) : Controller(),
                                                                           nh_priv_(nh_priv),
                                                                           name_(name),
                                                                           change_direction_(false),
                                                                           stop_(false),
                                                                           bumper_left_pressed_(false),
                                                                           bumper_center_pressed_(false),
                                                                           bumper_right_pressed_(false),
                                                                           cliff_left_detected_(false),
                                                                           cliff_center_detected_(false),
                                                                           cliff_right_detected_(false),
                                                                           led_bumper_on_(false),
                                                                           led_cliff_on_(false),
                                                                           led_wheel_drop_on_(false),
                                                                           turning_(false),
                                                                           turning_direction_(1)
                                                                           {};
  ~RandomWalkerController(){};

  /**
   * Set-up necessary publishers/subscribers and initialise time
   * @return true, if successful
   */
  bool init()
  {
    enable_controller_subscriber_ = haros::Subscriber<std_msgs::Empty>(nh_priv_, "enable", 10, &RandomWalkerController::enableCB, this);
    disable_controller_subscriber_ = haros::Subscriber<std_msgs::Empty>(nh_priv_, "disable", 10, &RandomWalkerController::disableCB, this);

    bumper_event_subscriber_ = haros::Subscriber<kobuki_msgs::BumperEvent>(nh_priv_, "events/bumper", 10, &RandomWalkerController::bumperEventCB, this);
    bumper_event_subscriber_.recordIf(isLeftBumper, "left");
    bumper_event_subscriber_.recordIf(isCenterBumper, "center");
    bumper_event_subscriber_.recordIf(isRightBumper, "right");

    cliff_event_subscriber_ = haros::Subscriber<kobuki_msgs::CliffEvent>(nh_priv_, "events/cliff", 10, &RandomWalkerController::cliffEventCB, this);
    cliff_event_subscriber_.recordIf(isLeftCliff, "left");
    cliff_event_subscriber_.recordIf(isCenterCliff, "center");
    cliff_event_subscriber_.recordIf(isRightCliff, "right");

    wheel_drop_event_subscriber_ = haros::Subscriber<kobuki_msgs::WheelDropEvent>(nh_priv_, "events/wheel_drop", 10, &RandomWalkerController::wheelDropEventCB, this);
    wheel_drop_event_subscriber_.recordIf(isLeftWheel, "left");
    wheel_drop_event_subscriber_.recordIf(isRightWheel, "right");

    cmd_vel_publisher_ = haros::Publisher<geometry_msgs::Twist>(nh_priv_, "commands/velocity", 10);
    led1_publisher_ = haros::Publisher<kobuki_msgs::Led>(nh_priv_, "commands/led1", 10);
    led2_publisher_ = haros::Publisher<kobuki_msgs::Led>(nh_priv_, "commands/led2", 10);

    nh_priv_.param("linear_velocity", vel_lin_, 0.5);
    nh_priv_.param("angular_velocity", vel_ang_, 0.1);
    ROS_INFO_STREAM("Velocity parameters: linear velocity = " << vel_lin_
                    << ", angular velocity = " << vel_ang_ << " [" << name_ <<"]");
    std::srand(std::time(0));

    this->enable(); // enable controller

    return true;
  };


  /**
   * @brief Publishes velocity commands and triggers the LEDs
   */
  void spin();

private:
  /// Mutex for shared data protection
  ecl::Mutex mutex_;
  /// Private ROS handle
  ros::NodeHandle nh_priv_;
  /// Node(let) name
  std::string name_;
  /// Subscribers
  haros::Subscriber<std_msgs::Empty> enable_controller_subscriber_,
                                     disable_controller_subscriber_;
  /// Subscribers
  haros::Subscriber<kobuki_msgs::BumperEvent> bumper_event_subscriber_;
  haros::Subscriber<kobuki_msgs::CliffEvent> cliff_event_subscriber_;
  haros::Subscriber<kobuki_msgs::WheelDropEvent> wheel_drop_event_subscriber_;
  /// Publishers
  haros::Publisher<geometry_msgs::Twist> cmd_vel_publisher_;
  haros::Publisher<kobuki_msgs::Led> led1_publisher_, led2_publisher_;
  /// Flag for changing direction
  bool change_direction_;
  /// Flag for stopping
  bool stop_;
  /// Flag for left bumper's state
  bool bumper_left_pressed_;
  /// Flag for center bumper's state
  bool bumper_center_pressed_;
  /// Flag for right bumper's state
  bool bumper_right_pressed_;
  /// Flag for left cliff sensor's state
  bool cliff_left_detected_;
  /// Flag for center cliff sensor's state
  bool cliff_center_detected_;
  /// Flag for right cliff sensor's state
  bool cliff_right_detected_;
  /// Flag for left wheel drop sensor's state
  bool wheel_drop_left_detected_;
  /// Flag for right wheel drop sensor's state
  bool wheel_drop_right_detected_;
  /// Flag for bumper LED's state
  bool led_bumper_on_;
  /// Flag for cliff sensor LED's state
  bool led_cliff_on_;
  /// Flag for wheel drop sensor LED's state
  bool led_wheel_drop_on_;
  /// Linear velocity for moving straight
  double vel_lin_;
  /// Angular velocity for rotating
  double vel_ang_;
  /// Randomly chosen turning duration
  ros::Duration turning_duration_;
  /// Randomly chosen turning direction
  int turning_direction_;
  /// Start time of turning
  ros::Time turning_start_;
  /// Flag for turning state
  bool turning_;

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
   * @brief Trigger direction change and LED blink, when a bumper is pressed
   * @param msg bumper event
   */
  void bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg);

  /**
   * @brief Trigger direction change and LED blink, when a cliff is detected
   * @param msg cliff event
   */
  void cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg);

  /**
   * @brief Trigger stopping and LED blink, when a wheel drop is detected
   * @param msg wheel drop event
   */
  void wheelDropEventCB(const kobuki_msgs::WheelDropEventConstPtr msg);

  bool hasPressedBumper() const;

  bool hasDetectedCliff() const;

  bool hasDroppedWheel() const;
};

void RandomWalkerController::enableCB(const std_msgs::EmptyConstPtr msg)
{
  ROS_ASSERT(true); // nothing relevant to say
  mutex_.lock();
  if (this->enable())
  {
    ROS_INFO_STREAM("Controller has been enabled. [" << name_ << "]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"]");
  }
  enable_controller_subscriber_.updateHistory();
  mutex_.unlock();
};

void RandomWalkerController::disableCB(const std_msgs::EmptyConstPtr msg)
{
  ROS_ASSERT(true); // nothing relevant to say
  mutex_.lock();
  if (this->disable())
  {
    ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
  disable_controller_subscriber_.updateHistory();
  mutex_.unlock();
};

void RandomWalkerController::bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg)
{
  ROS_ASSERT(msg->state == kobuki_msgs::BumperEvent::PRESSED
          || msg->state == kobuki_msgs::BumperEvent::RELEASED);
  ROS_ASSERT(msg->bumper == kobuki_msgs::BumperEvent::LEFT
          || msg->bumper == kobuki_msgs::BumperEvent::CENTER
          || msg->bumper == kobuki_msgs::BumperEvent::RIGHT);
  mutex_.lock();
  if (this->getState()) // check, if the controller is active
  {
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
      switch (msg->bumper)
      {
        case kobuki_msgs::BumperEvent::LEFT:
          if (!bumper_left_pressed_)
          {
            bumper_left_pressed_ = true;
            change_direction_ = true;
          }
          break;
        case kobuki_msgs::BumperEvent::CENTER:
          if (!bumper_center_pressed_)
          {
            bumper_center_pressed_ = true;
            change_direction_ = true;
          }
          break;
        case kobuki_msgs::BumperEvent::RIGHT:
          if (!bumper_right_pressed_)
          {
            bumper_right_pressed_ = true;
            change_direction_ = true;
          }
          break;
      }
    }
    else // kobuki_msgs::BumperEvent::RELEASED
    {
      switch (msg->bumper)
      {
        case kobuki_msgs::BumperEvent::LEFT:
          bumper_left_pressed_ = false;
          break;
        case kobuki_msgs::BumperEvent::CENTER:
          bumper_center_pressed_ = false;
          break;
        case kobuki_msgs::BumperEvent::RIGHT:
          bumper_right_pressed_ = false;
          break;
      }
    }
    if (!led_bumper_on_ && (bumper_left_pressed_ || bumper_center_pressed_ || bumper_right_pressed_))
    {
      kobuki_msgs::LedPtr led_msg_ptr;
      led_msg_ptr.reset(new kobuki_msgs::Led());
      led_msg_ptr->value = kobuki_msgs::Led::ORANGE;
      {
        // This message is a bumper PRESSED, or the most recent message
        // for some of the bumpers was PRESSED.
        ROS_ASSERT(msg->state == kobuki_msgs::BumperEvent::PRESSED || hasPressedBumper());
      }
      led1_publisher_.publish(led_msg_ptr);
      led_bumper_on_ = true;
    }
    else if (led_bumper_on_ && (!bumper_left_pressed_ && !bumper_center_pressed_ && !bumper_right_pressed_))
    {
      kobuki_msgs::LedPtr led_msg_ptr;
      led_msg_ptr.reset(new kobuki_msgs::Led());
      led_msg_ptr->value = kobuki_msgs::Led::BLACK;
      {
        // This message is a bumper RELEASED and the most recent message
        // for the other bumpers was also RELEASED.
        ROS_ASSERT(msg->state == kobuki_msgs::BumperEvent::RELEASED);
        ROS_ASSERT(!hasPressedBumper());
      }
      led1_publisher_.publish(led_msg_ptr);
      led_bumper_on_ = false;
    }
    if (change_direction_)
    {
      ROS_INFO_STREAM("Bumper pressed. Changing direction. [" << name_ << "]");
    }
  }
  bumper_event_subscriber_.updateHistory();
  mutex_.unlock();
};

void RandomWalkerController::cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg)
{
  ROS_ASSERT(msg->state == kobuki_msgs::CliffEvent::CLIFF
          || msg->state == kobuki_msgs::CliffEvent::FLOOR);
  ROS_ASSERT(msg->sensor == kobuki_msgs::CliffEvent::LEFT
          || msg->sensor == kobuki_msgs::CliffEvent::CENTER
          || msg->sensor == kobuki_msgs::CliffEvent::RIGHT);
  mutex_.lock();
  if (msg->state == kobuki_msgs::CliffEvent::CLIFF)
  {
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:
        if (!cliff_left_detected_)
        {
          cliff_left_detected_ = true;
          change_direction_ = true;
        }
        break;
      case kobuki_msgs::CliffEvent::CENTER:
        if (!cliff_center_detected_)
        {
          cliff_center_detected_ = true;
          change_direction_ = true;
        }
        break;
      case kobuki_msgs::CliffEvent::RIGHT:
        if (!cliff_right_detected_)
        {
          cliff_right_detected_ = true;
          change_direction_ = true;
        }
        break;
    }
  }
  else // kobuki_msgs::BumperEvent::FLOOR
  {
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:
        cliff_left_detected_ = false;
        break;
      case kobuki_msgs::CliffEvent::CENTER:
        cliff_center_detected_ = false;
        break;
      case kobuki_msgs::CliffEvent::RIGHT:
        cliff_right_detected_ = false;
        break;
    }
  }
  if (!led_cliff_on_ && (cliff_left_detected_ || cliff_center_detected_ || cliff_right_detected_))
  {
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());
    led_msg_ptr->value = kobuki_msgs::Led::ORANGE;
    {
      // This message is a CLIFF, or the most recent message for some
      // of the sensors was a CLIFF.
      ROS_ASSERT(msg->state == kobuki_msgs::CliffEvent::CLIFF || hasDetectedCliff());
    }
    led2_publisher_.publish(led_msg_ptr);
    led_cliff_on_ = true;
  }
  else if (led_cliff_on_ && (!cliff_left_detected_ && !cliff_center_detected_ && !cliff_right_detected_))
  {
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());
    led_msg_ptr->value = kobuki_msgs::Led::BLACK;
    {
      // This message is a FLOOR and the most recent message
      // for the other sensors was also FLOOR.
      ROS_ASSERT(msg->state == kobuki_msgs::CliffEvent::FLOOR);
      ROS_ASSERT(!hasDetectedCliff());
    }
    led2_publisher_.publish(led_msg_ptr);
    led_cliff_on_ = false;
  }
  if (change_direction_)
  {
    ROS_INFO_STREAM("Cliff detected. Changing direction. [" << name_ << "]");
  }
  cliff_event_subscriber_.updateHistory();
  mutex_.unlock();
};

void RandomWalkerController::wheelDropEventCB(const kobuki_msgs::WheelDropEventConstPtr msg)
{
  ROS_ASSERT(msg->state == kobuki_msgs::WheelDropEvent::DROPPED
          || msg->state == kobuki_msgs::WheelDropEvent::RAISED);
  ROS_ASSERT(msg->wheel == kobuki_msgs::WheelDropEvent::LEFT
          || msg->wheel == kobuki_msgs::WheelDropEvent::RIGHT);
  mutex_.lock();
  if (msg->state == kobuki_msgs::WheelDropEvent::DROPPED)
  {
    switch (msg->wheel)
    {
      case kobuki_msgs::WheelDropEvent::LEFT:
        if (!wheel_drop_left_detected_)
        {
          wheel_drop_left_detected_ = true;
        }
        break;
      case kobuki_msgs::WheelDropEvent::RIGHT:
        if (!wheel_drop_right_detected_)
        {
          wheel_drop_right_detected_ = true;
        }
        break;
    }
  }
  else // kobuki_msgs::WheelDropEvent::RAISED
  {
    switch (msg->wheel)
    {
      case kobuki_msgs::WheelDropEvent::LEFT:
        wheel_drop_left_detected_ = false;
        break;
      case kobuki_msgs::WheelDropEvent::RIGHT:
        wheel_drop_right_detected_ = false;
        break;
    }
  }
  if (!led_wheel_drop_on_ && (wheel_drop_left_detected_ || wheel_drop_right_detected_))
  {
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());
    led_msg_ptr->value = kobuki_msgs::Led::RED;
    {
      // This message is a wheel DROPPED, or the most recent message for some
      // of the wheels was a DROPPED.
      ROS_ASSERT(msg->state == kobuki_msgs::WheelDropEvent::DROPPED || hasDroppedWheel());
    }
    led1_publisher_.publish(led_msg_ptr);
    led2_publisher_.publish(led_msg_ptr);
    stop_ = true;
    led_wheel_drop_on_ = true;
  }
  else if (led_wheel_drop_on_ && (!wheel_drop_left_detected_ && !wheel_drop_right_detected_))
  {
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());
    led_msg_ptr->value = kobuki_msgs::Led::BLACK;
    {
      // This message is a wheel RAISED and the most recent message
      // for the other wheel was also RAISED.
      ROS_ASSERT(msg->state == kobuki_msgs::WheelDropEvent::RAISED);
      ROS_ASSERT(!hasDroppedWheel());
    }
    led1_publisher_.publish(led_msg_ptr);
    led2_publisher_.publish(led_msg_ptr);
    stop_ = false;
    led_wheel_drop_on_ = false;
  }
  if (change_direction_)
  {
    ROS_INFO_STREAM("Wheel(s) dropped. Stopping. [" << name_ << "]");
  }
  wheel_drop_event_subscriber_.updateHistory();
  mutex_.unlock();
};

void RandomWalkerController::spin()
{
  mutex_.lock();
  if (this->getState()) // check, if the controller is active
  {
    // Velocity commands
    geometry_msgs::TwistPtr cmd_vel_msg_ptr;
    cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

    if (stop_)
    {
      {
        // Message is initialised to zeroes.
        ROS_ASSERT(cmd_vel_msg_ptr->linear.x == 0.0);
        ROS_ASSERT(cmd_vel_msg_ptr->angular.z == 0.0);
        // The most recent message for some of the wheels was a wheel DROPPED.
        ROS_ASSERT(hasDroppedWheel());
      }
      cmd_vel_publisher_.publish(cmd_vel_msg_ptr); // will be all zero when initialised
      return;
    }

    if (change_direction_)
    {
      change_direction_ = false;
      // calculate a random turning angle (-180 ... +180) based on the set angular velocity
      // time for turning 180 degrees in seconds = M_PI / angular velocity
      turning_duration_ = ros::Duration(((double)std::rand() / (double)RAND_MAX) * (M_PI / vel_ang_));
      // randomly chosen turning direction
      if (((double)std::rand() / (double)RAND_MAX) >= 0.5)
      {
        turning_direction_ = 1;
      }
      else
      {
        turning_direction_ = -1;
      }
      turning_start_ = ros::Time::now();
      turning_ = true;
      ROS_INFO_STREAM("Will rotate " << turning_direction_ * turning_duration_.toSec() * vel_ang_ / M_PI * 180
                      << " degrees. [" << name_ << "]");
    }

    if (turning_)
    {
      if ((ros::Time::now() - turning_start_) < turning_duration_)
      {
        cmd_vel_msg_ptr->angular.z = turning_direction_ * vel_ang_;
        {
          // The most recent message for some of the bumpers was PRESSED.
          ROS_ASSERT(hasPressedBumper());
          // The most recent message for some of the sensors was a CLIFF.
          ROS_ASSERT(hasDetectedCliff());
        }
        cmd_vel_publisher_.publish(cmd_vel_msg_ptr);
      }
      else
      {
        turning_ = false;
      }
    }
    else
    {
      cmd_vel_msg_ptr->linear.x = vel_lin_;
      {
        // Either all sensors are ok or it already finished turning.
        ROS_ASSERT((!hasPressedBumper() && !hasDetectedCliff())
            || (ros::Time::now() - turning_start_) >= turning_duration_);
      }
      cmd_vel_publisher_.publish(cmd_vel_msg_ptr);
    }
  }
  mutex_.unlock();
}

bool RandomWalkerController::hasPressedBumper() const
{
  const haros::MessageEvent<kobuki_msgs::BumperEvent>
      left = bumper_event_subscriber_.lastReceive("left"),
      center = bumper_event_subscriber_.lastReceive("center"),
      right = bumper_event_subscriber_.lastReceive("right");
  return (left && left.msg->state == kobuki_msgs::BumperEvent::PRESSED)
      || (center && center.msg->state == kobuki_msgs::BumperEvent::PRESSED)
      || (right && right.msg->state == kobuki_msgs::BumperEvent::PRESSED);
}

bool RandomWalkerController::hasDetectedCliff() const
{
  const haros::MessageEvent<kobuki_msgs::CliffEvent>
      left = cliff_event_subscriber_.lastReceive("left"),
      center = cliff_event_subscriber_.lastReceive("center"),
      right = cliff_event_subscriber_.lastReceive("right");
  return (left && left.msg->state == kobuki_msgs::CliffEvent::CLIFF)
      || (center && center.msg->state == kobuki_msgs::CliffEvent::CLIFF)
      || (right && right.msg->state == kobuki_msgs::CliffEvent::CLIFF);
}

bool RandomWalkerController::hasDroppedWheel() const
{
  const haros::MessageEvent<kobuki_msgs::WheelDropEvent>
      left = wheel_drop_event_subscriber_.lastReceive("left"),
      right = wheel_drop_event_subscriber_.lastReceive("right");
  return (left && left.msg->state == kobuki_msgs::WheelDropEvent::DROPPED)
      || (right && right.msg->state == kobuki_msgs::WheelDropEvent::DROPPED);
}

} // namespace kobuki
#endif /* RANDOM_WALKER_CONTROLLER_HPP_ */
