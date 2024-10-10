/*
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 * Copyright 2017 Siddharth Patel, NTU Singapore
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

#include <gazebo_motor_failure_plugin.h>

namespace gazebo {

GazeboMotorFailure::GazeboMotorFailure() :
    ModelPlugin(),
    ROS_motor_num_sub_topic_(kDefaultROSMotorNumSubTopic),
    ROS_motor_num_sub_topic_1(kDefaultROSMotorNumSubTopic1),
    ROS_motor_num_sub_topic_2(kDefaultROSMotorNumSubTopic2),
    ROS_motor_num_sub_topic_3(kDefaultROSMotorNumSubTopic3),
    motor_failure_num_pub_topic_(kDefaultMotorFailureNumPubTopic),
    motor_failure_num_pub_topic_1(kDefaultMotorFailureNumPubTopic1),
    motor_failure_num_pub_topic_2(kDefaultMotorFailureNumPubTopic2),
    motor_failure_num_pub_topic_3(kDefaultMotorFailureNumPubTopic3)
{ }

GazeboMotorFailure::~GazeboMotorFailure() {
  this->updateConnection_.reset();
}

void GazeboMotorFailure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  // 이거 지우면 안됨. 초기화 값을 설정해줘야 처음 실행했을때 랜덤으로 값이 안들어간다
    motor_Failure_Number_ = 0;
    motor_Failure_Number_1 = 10;
    motor_Failure_Number_2 = 20;
    motor_Failure_Number_3 = 30;
  //

  this->namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  ////Gazebo의 transport 시스템은 시뮬레이션 내의 다양한 컴포넌트 간에 메시지 기반 통신을 가능하게 하는 매커니즘이다
  /*
   이는 Gazebo 내에서의 데이터 교환과 이벤트 처리를 위한 중요한 구조적 요소입니다.
   따라서 node_handle_은 Gazebo 환경에 특화된 노드로, Gazebo 시뮬레이션 밖에서는 사용되지 않습니다.
  */
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_); ////Gazebo 및 ROS 통신을 위한 노드 핸들

  motor_failure_pub_ = node_handle_->Advertise<msgs::Int>("/gazebo/motor_failure_num", 1);
  motor_failure_pub_1 = node_handle_->Advertise<msgs::Int>("/gazebo/motor_failure_num1", 1);
  motor_failure_pub_2 = node_handle_->Advertise<msgs::Int>("/gazebo/motor_failure_num2", 1);
  motor_failure_pub_3 = node_handle_->Advertise<msgs::Int>("/gazebo/motor_failure_num3", 1);
  //motor_failure_pub_ 라는 퍼블리셔가 생기고 <msgs::Int>는 구독할 메시지의 타입을 가지는 motor_failure_num_pub_topic_ 을 pub 한다.
  //여기서 의문점 : <msgs::Int>는 구독할 메시지의 타입을 pub 한다고 했다. msgs::Int는 정수형 데이터를 담을 수 있는 메시지 타입으로, 여기서는 모터의 고장 번호 등 정수형 데이터를 발행하는 데 사용
  //여기서 의문점 : 그럼 motor_failure_num_pub_topic_ 은 msgs::Int 타입인데  if (_sdf->HasElement("MotorFailureNumPubTopic")) 여기를 보면 std::string 이라는데?

  //신기하다
  // motor_failure_pub_ 가 pub하는 토픽인 motor_failure_num_pub_topic_에 motor_failure_msg_ 라는 msg객체의 데이터를 담을수 있다??

  if (_sdf->HasElement("ROSMotorNumSubTopic")) {    //sdf 파일에 지금 이 줄이 없다 ROSMotorNumSubTopic ******************************
    this->ROS_motor_num_sub_topic_ = _sdf->GetElement("ROSMotorNumSubTopic")->Get<std::string>();
  }
  else
  {
      ROS_WARN("No ROSMotorNumSubTopic specified. Defaulting to '/motor_failure/motor_number'.");
      ROS_motor_num_sub_topic_ = "/motor_failure/motor_number";
  }

  if (_sdf->HasElement("ROSMotorNumSubTopic1")) {    //sdf 파일에 지금 이 줄이 없다 ROSMotorNumSubTopic ******************************
    this->ROS_motor_num_sub_topic_1 = _sdf->GetElement("ROSMotorNumSubTopic1")->Get<std::string>();
  }
  else
  {
      ROS_WARN("No ROSMotorNumSubTopic1 specified. Defaulting to '/motor_failure/motor_number1'.");
      ROS_motor_num_sub_topic_1 = "/motor_failure/motor_number1";
  }

  if (_sdf->HasElement("ROSMotorNumSubTopic2")) {    //sdf 파일에 지금 이 줄이 없다 ROSMotorNumSubTopic ******************************
    this->ROS_motor_num_sub_topic_2 = _sdf->GetElement("ROSMotorNumSubTopic2")->Get<std::string>();
  }
  else
  {
      ROS_WARN("No ROSMotorNumSubTopic2 specified. Defaulting to '/motor_failure/motor_number2'.");
      ROS_motor_num_sub_topic_2 = "/motor_failure/motor_number2";
  }

  if (_sdf->HasElement("ROSMotorNumSubTopic3")) {    //sdf 파일에 지금 이 줄이 없다 ROSMotorNumSubTopic ******************************
    this->ROS_motor_num_sub_topic_3 = _sdf->GetElement("ROSMotorNumSubTopic3")->Get<std::string>();
  }
  else
  {
      ROS_WARN("No ROSMotorNumSubTopic3 specified. Defaulting to '/motor_failure/motor_number3'.");
      ROS_motor_num_sub_topic_3 = "/motor_failure/motor_number3";
  }


  //HasElement("MotorFailureNumPubTopic"): _sdf 객체의 HasElement 메서드는 주어진 이름("MotorFailureNumPubTopic")의 요소가 SDF 파일 내에 존재하는지를 검사합니다.
  //이 메서드는 불리언(boolean) 값을 반환하며, 해당 요소가 있을 경우 true, 없을 경우 false를 반환합니다.
  //Get<std::string>() 메서드를 호출하여 그 값을 문자열로 추출
  if (_sdf->HasElement("MotorFailureNumPubTopic")) {   //sdf 파일에 지금 이 줄이 없다 MotorFailureNumPubTopic ******************************
    this->motor_failure_num_pub_topic_ = _sdf->GetElement("MotorFailureNumPubTopic")->Get<std::string>();
  }
  // else
  // {
  //   ROS_WARN("MotorFailure", "MotorFailure plugin missing <MotorFailureNumPubTopic>, cannot proceed");
  //   motor_failure_num_pub_topic_ = "/gazebo/motor_failure_num";

  // }
  if (_sdf->HasElement("MotorFailureNumPubTopic1")) {   //sdf 파일에 지금 이 줄이 없다 MotorFailureNumPubTopic ******************************
    this->motor_failure_num_pub_topic_1 = _sdf->GetElement("MotorFailureNumPubTopic1")->Get<std::string>();
  }

  if (_sdf->HasElement("MotorFailureNumPubTopic2")) {   //sdf 파일에 지금 이 줄이 없다 MotorFailureNumPubTopic ******************************
    this->motor_failure_num_pub_topic_2 = _sdf->GetElement("MotorFailureNumPubTopic2")->Get<std::string>();
  }

  if (_sdf->HasElement("MotorFailureNumPubTopic3")) {   //sdf 파일에 지금 이 줄이 없다 MotorFailureNumPubTopic ******************************
    this->motor_failure_num_pub_topic_3 = _sdf->GetElement("MotorFailureNumPubTopic3")->Get<std::string>();
  }

  //내가 추가한거--------------------------------------------------------------------------------------------
  // ROS Topic subscriber
  // Initialize ROS, if it has not already bee initialized.
  if (!ros::isInitialized())  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_ros_sub", ros::init_options::NoSigintHandler);

  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  this->rosNode = new ros::NodeHandle(this->namespace_);
  //this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int32>("/motor_failure/motor_number", 1, boost::bind(&GazeboMotorFailure::motorFailNumCallBack, this, _1), ros::VoidPtr(), &this->rosQueue);
  ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Int32>("/motor_failure/motor_number1", 1, boost::bind(&GazeboMotorFailure::motorFailNumCallBack1, this, _1), ros::VoidPtr(), &this->rosQueue);
  ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Int32>("/motor_failure/motor_number2", 1, boost::bind(&GazeboMotorFailure::motorFailNumCallBack2, this, _1), ros::VoidPtr(), &this->rosQueue);
  ros::SubscribeOptions so3 = ros::SubscribeOptions::create<std_msgs::Int32>("/motor_failure/motor_number3", 1, boost::bind(&GazeboMotorFailure::motorFailNumCallBack3, this, _1), ros::VoidPtr(), &this->rosQueue);


  this->rosSub = this->rosNode->subscribe(so);
  this->rosSub1 = this->rosNode->subscribe(so1);
  this->rosSub2 = this->rosNode->subscribe(so2);
  this->rosSub3 = this->rosNode->subscribe(so3);
  //so는 구독 옵션을 담고 있는 객체
  //rosSub 가 node 이다. rosSub는 so라는 옵션을 사용한다.


  this->rosQueueThread = std::thread(std::bind(&GazeboMotorFailure::QueueThread, this));


  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_ << std::endl;
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_1 << std::endl;
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_2 << std::endl;
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_3 << std::endl;

  //내가 추가한거--------------------------------------------------------------------------------------------


  //--------------------------------------------------------------------------------------------
  /*
  // ROS2 Topic subscriber
  // Initialize ROS2, if it has not already been initialized.
  if (!rclcpp::is_initialized()) {
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);
  }

  // Create our ROS2 node. This acts in a similar manner to the Gazebo node
  this->ros_node_ = rclcpp::Node::make_shared("motor_failure");

  // Create a named topic, and subscribe to it.
  subscription = this->ros_node_->create_subscription<std_msgs::msg::Int32>(
		  this->ROS_motor_num_sub_topic_, 10,
		  boost::bind(&GazeboMotorFailure::motorFailNumCallBack, this, boost::placeholders::_1));
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_ << std::endl;
  */
  //---------------------------------------------------------------------------------------

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboMotorFailure::OnUpdate, this, _1));
}

//OnUpdate 함수는 시뮬레이션 동안 주기적으로 호출되어, 모터 고장 번호를 Gazebo 토픽으로 발행합니다.
void GazeboMotorFailure::OnUpdate(const common::UpdateInfo &info) {
    this->motor_failure_msg_.set_data(motor_Failure_Number_);
    this->motor_failure_msg_1.set_data(motor_Failure_Number_1);
    this->motor_failure_msg_2.set_data(motor_Failure_Number_2);
    this->motor_failure_msg_3.set_data(motor_Failure_Number_3);
    //motor_Failure_Number_ 는 gazebo_motor_model 에서 왔다. 정확하게 하면 MotorFailureCallback 콜백함수 지나서 왔다
    this->motor_failure_pub_->Publish(motor_failure_msg_);
    this->motor_failure_pub_1->Publish(motor_failure_msg_1);
    this->motor_failure_pub_2->Publish(motor_failure_msg_2);
    this->motor_failure_pub_3->Publish(motor_failure_msg_3);
    //rclcpp::spin_some(this->ros_node_);
    ros::spinOnce();  //내가 수정한거---------------------------------------------------
}

// void GazeboMotorFailure::motorFailNumCallBack(const std_msgs::msg::Int32::SharedPtr msg) {
void GazeboMotorFailure::motorFailNumCallBack(const std_msgs::Int32::ConstPtr& msg) {
  this->motor_Failure_Number_ = msg->data;//std_msgs::msg::Int32::SharedPtr msg 에서 data 라는 정보를 motor_Failure_Number_에 넣어라
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to " << ROS_motor_num_sub_topic_ << std::endl;
}

void GazeboMotorFailure::motorFailNumCallBack1(const std_msgs::Int32::ConstPtr& msg) {
  this->motor_Failure_Number_1 = msg->data;//std_msgs::msg::Int32::SharedPtr msg 에서 data 라는 정보를 motor_Failure_Number_에 넣어라
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to " << ROS_motor_num_sub_topic_1 << std::endl;
}

void GazeboMotorFailure::motorFailNumCallBack2(const std_msgs::Int32::ConstPtr& msg) {
  this->motor_Failure_Number_2 = msg->data;//std_msgs::msg::Int32::SharedPtr msg 에서 data 라는 정보를 motor_Failure_Number_에 넣어라
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to " << ROS_motor_num_sub_topic_2 << std::endl;
}

void GazeboMotorFailure::motorFailNumCallBack3(const std_msgs::Int32::ConstPtr& msg) {
  this->motor_Failure_Number_3 = msg->data;//std_msgs::msg::Int32::SharedPtr msg 에서 data 라는 정보를 motor_Failure_Number_에 넣어라
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to " << ROS_motor_num_sub_topic_3 << std::endl;
}

//내가 추가한거 -------------------------------------------------------------------------------------------------
void GazeboMotorFailure::QueueThread() {
  static const double timeout = 0.01;

  while (this->rosNode->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
// void GazeboMotorFailure::QueueThread1() {
//   static const double timeout = 0.01;

//   while (this->rosNode->ok()) {
//     this->rosQueue1.callAvailable(ros::WallDuration(timeout));
//   }
// }
//내가 추가한거 -------------------------------------------------------------------------------------------------

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFailure);
}
