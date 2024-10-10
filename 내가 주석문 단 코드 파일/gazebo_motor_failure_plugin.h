/*
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 * Copyright 2017 Siddharth Patel, NTU Singapore
 * Copyright 2022 SungTae Moon, KOREATECH Korea
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_motor_model.h"

#include <stdio.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "common.h"

// ROS Topic subscriber
#include <thread>
// 내가 추가 -----------------------------------------------------------

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Int32.h>

// 내가 추가 ------------------------------------------------------------
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/int32.hpp>

namespace gazebo {  //가장 큰 바운더리

// Default values
static const std::string kDefaultROSMotorNumSubTopic = "/motor_failure/motor_number";
static const std::string kDefaultROSMotorNumSubTopic1 = "/motor_failure/motor_number1";
static const std::string kDefaultROSMotorNumSubTopic2 = "/motor_failure/motor_number2";
static const std::string kDefaultROSMotorNumSubTopic3 = "/motor_failure/motor_number3";
//ROS 에서 (/motor_failure/motor_number)를 sub하고,

//여기는 gazebo_motor_model.h랑 다르게 kDefaultMotorFailureNumSubTopic가 아니라 PubTopic 다
static const std::string kDefaultMotorFailureNumPubTopic = "/gazebo/motor_failure_num";
static const std::string kDefaultMotorFailureNumPubTopic1 = "/gazebo/motor_failure_num1";
static const std::string kDefaultMotorFailureNumPubTopic2 = "/gazebo/motor_failure_num2";
static const std::string kDefaultMotorFailureNumPubTopic3 = "/gazebo/motor_failure_num3";
// Gazebo 에서 (/gazebo/motor_failure_num)를 pub합니다.

//GazeboMotorFailure 클래스: ModelPlugin을 상속받아 정의되며, Gazebo 모델 플러그인으로 동작합니다.
class GazeboMotorFailure : public ModelPlugin {   // 그다음 GazeboMotorFailure 라는 바운더리
 public:
  GazeboMotorFailure();

  // void motorFailNumCallBack(const std_msgs::msg::Int32::SharedPtr msg);
  void motorFailNumCallBack(const std_msgs::Int32::ConstPtr& msg);
  void motorFailNumCallBack1(const std_msgs::Int32::ConstPtr& msg);
  void motorFailNumCallBack2(const std_msgs::Int32::ConstPtr& msg);
  void motorFailNumCallBack3(const std_msgs::Int32::ConstPtr& msg);
  //motorFailNumCallBack 함수는 ROS 토픽에서 모터 고장 번호를 받을 때 호출되는 콜백 함수입니다.


  virtual ~GazeboMotorFailure();

 protected:

  /// \brief Create subscription to ROS topic that triggers the motor failure
  /// \details Inits a rosnode in case there's not one and subscribes to ROS_motor_num_sub_topic_
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
//Load 함수는 Gazebo 플러그인이 모델에 로드될 때 호출되어, 필요한 초기 설정을 수행합니다. 여기에는 ROS 노드 초기화와 ROS 토픽 구독 설정이 포함됩니다.
  /// \brief Updates and publishes the motor_Failure_Number_ to motor_failure_num_pub_topic_
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);
//OnUpdate 함수는 시뮬레이션 동안 주기적으로 호출되어, 모터 고장 번호를 Gazebo 토픽으로 발행합니다.
 private:

  //내가 추가한거 -----------------------------------------------------------------
  void QueueThread();
  //내가 추가한거 -----------------------------------------------------------------

  event::ConnectionPtr updateConnection_;

  std::string ROS_motor_num_sub_topic_; // 구독할 토픽의 이름
  std::string ROS_motor_num_sub_topic_1;
  std::string ROS_motor_num_sub_topic_2;
  std::string ROS_motor_num_sub_topic_3;
  std::string motor_failure_num_pub_topic_;
  std::string motor_failure_num_pub_topic_1;
  std::string motor_failure_num_pub_topic_2;
  std::string motor_failure_num_pub_topic_3;
  std::string namespace_;

  transport::NodePtr node_handle_;   //이걸 사용하면 gazebo에 노드 생성
  transport::PublisherPtr motor_failure_pub_; /*!< Publish the motor_Failure_num to gazebo topic motor_failure_num_pub_topic_ */
  transport::PublisherPtr motor_failure_pub_1;
  transport::PublisherPtr motor_failure_pub_2;
  transport::PublisherPtr motor_failure_pub_3;
//ROS와 Gazebo 통신을 위한 변수들(ROS_motor_num_sub_topic_, motor_failure_num_pub_topic_, namespace_, node_handle_, motor_failure_pub_)
  boost::thread callback_queue_thread_;

  msgs::Int motor_failure_msg_;
  msgs::Int motor_failure_msg_1;
  msgs::Int motor_failure_msg_2;
  msgs::Int motor_failure_msg_3;
  int32_t motor_Failure_Number_;
  int32_t motor_Failure_Number_1;
  int32_t motor_Failure_Number_2;
  int32_t motor_Failure_Number_3;
//motor_Failure_Number_는 현재 모터 고장 번호를 저장합니다.
  //내가 추가한거 -----------------------------------------------------------------

  ros::NodeHandle* rosNode; // ROS1 노드 핸들, 이걸 사용하면 ros에 노드 생성
  ros::Subscriber rosSub; // ROS1 구독자
  ros::Subscriber rosSub1;
  ros::Subscriber rosSub2;
  ros::Subscriber rosSub3;
  ros::CallbackQueue rosQueue;

  std::thread rosQueueThread;

  //내가 추가한거 -----------------------------------------------------------------

  // ROS2 communication
  // rclcpp::Node::SharedPtr ros_node_;
  // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription;
//ROS2 노드(ros_node_), ROS2 구독자(subscription)를 포함합니다.
};
}
