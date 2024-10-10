/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include "CommandMotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "MotorSpeed.pb.h"
#include "Float.pb.h"
#include "Wind.pb.h"

#include "common.h"

namespace turning_direction
{
  const static int CCW = 1;
  const static int CW = -1;
}

namespace gazebo
{ // 가장 큰 바운더리
  // Default values
  static const std::string kDefaultNamespace = "";
  static const std::string kDefaultCommandSubTopic = "/gazebo/command/motor_speed";
  static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
  static const std::string kDefaultMotorFailureNumSubTopic1 = "/gazebo/motor_failure_num1";
  static const std::string kDefaultMotorFailureNumSubTopic2 = "/gazebo/motor_failure_num2";
  static const std::string kDefaultMotorFailureNumSubTopic3 = "/gazebo/motor_failure_num3";
  static const std::string kDefaultMotorVelocityPubTopic = "/motor_speed";
  static const std::string wind_sub_topic_ = "/world_wind";

  typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
  typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;

  /*
  // Protobuf test
  typedef const boost::shared_ptr<const mav_msgs::msgs::MotorSpeed> MotorSpeedPtr;
  static const std::string kDefaultMotorTestSubTopic = "motors";
  */

  // Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
  static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
  static constexpr double kDefaultMotorConstant = 8.54858e-06;
  static constexpr double kDefaultMomentConstant = 0.016;
  static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
  static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
  static constexpr double kDefaulMaxRotVelocity = 838.0;
  static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
  static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;
  static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

  // 이 클래스는 모터의 물리적 모델을 시뮬레이션하고, 모터 속도 명령을 수신 및 처리하며, 모터 고장 상황을 시뮬레이션합니다.
  class GazeboMotorModel : public MotorModel, public ModelPlugin
  { // gazebo 바운더리 안에 GazeboMotorModel 바운더리
  public:
    GazeboMotorModel()
        : ModelPlugin(), // 매개변수가 없는 생성자
          MotorModel()
    {
    }

    virtual ~GazeboMotorModel();

    virtual void InitializeParams();
    virtual void Publish();
    // void testProto(MotorSpeedPtr &msg);
  protected:
    virtual void UpdateForcesAndMoments();

    /// \brief A function to check the motor_Failure_Number_ and stimulate motor fail
    /// \details Doing joint_->SetVelocity(0,0) for the flagged motor to fail
    virtual void UpdateMotorFail();
    virtual void UpdateMotorFail1();
    virtual void UpdateMotorFail2();
    virtual void UpdateMotorFail3();
    // UpdateMotorFail 함수는 모터 고장 시나리오를 시뮬레이션하기 위한 메서드입니다.
    // 이 함수의 주요 목적은 설정된 모터 고장 번호(motor_Failure_Number_)를 확인하고, 해당 모터를 실패 상태로 전환하는 것입니다.
    // 함수 설명에서 언급된 대로, 모터를 실패 상태로 만들기 위해 joint_->SetVelocity(0,0)을 호출합니다.
    // 이 호출은 지정된 모터의 속도를 0으로 설정하여, 실제로 해당 모터가 작동을 멈춘 것처럼 행동하게 합니다.
    // 함수 내에서 motor_Failure_Number_ 변수는 고장 난 모터의 번호를 나타냅니다.
    // 이 변수의 값에 따라 해당 모터의 joint_ 객체의 SetVelocity 메서드를 호출하여 모터의 속도를 0으로 설정합니다.

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

  private:
    std::string command_sub_topic_{kDefaultCommandSubTopic};               // 모터 속도 명령을 sub하는 토픽
    std::string motor_failure_sub_topic_{kDefaultMotorFailureNumSubTopic}; // 모터 고장 번호를 sub하는 토픽  //  static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
    std::string motor_failure_sub_topic_1{kDefaultMotorFailureNumSubTopic1};
    std::string motor_failure_sub_topic_2{kDefaultMotorFailureNumSubTopic2};
    std::string motor_failure_sub_topic_3{kDefaultMotorFailureNumSubTopic3};
    std::string joint_name_;                                           // Gazebo 모델 내의 특정 조인트와 링크의 이름입니다. 이들은 모터와 연결된 물리적 요소를 지정
    std::string link_name_;                                            // Gazebo 모델 내의 특정 조인트와 링크의 이름입니다. 이들은 모터와 연결된 물리적 요소를 지정
    std::string motor_speed_pub_topic_{kDefaultMotorVelocityPubTopic}; // 모터 속도 정보를 발행하는 토픽
    std::string namespace_;

    msgs::Int motor_failure_num_pub_topic_;
    msgs::Int motor_failure_num_pub_topic_1;
    msgs::Int motor_failure_num_pub_topic_2;
    msgs::Int motor_failure_num_pub_topic_3;

    int motor_number_{0};                          // 모터의 고유 번호입니다. 이 번호는 모터를 식별하는 데 사용
    int turning_direction_{turning_direction::CW}; // 모터의 회전 방향을 나타냅니다. turning_direction::CW는 시계 방향을 의미

    int motor_Failure_Number_{0}; /*!< motor_Failure_Number is (motor_number_ + 1) as (0) is considered no_fail. Publish accordingly */
    int motor_Failure_Number_1{0};
    int motor_Failure_Number_2{0};
    int motor_Failure_Number_3{0};

    // 현재 실패한 모터의 번호입니다. 0은 모터 고장이 없음
    int tmp_motor_num; // A temporary variable used to print msg
    int tmp_motor_num1;
    int tmp_motor_num2;
    int tmp_motor_num3;

    int screen_msg_flag = 1;
    int screen_msg_flag1 = 1;
    int screen_msg_flag2 = 1;
    int screen_msg_flag3 = 1;
    // int a = 0;

    // max_force_, max_rot_velocity_, moment_constant_, motor_constant_: 모터의 최대 힘, 최대 회전 속도, 모멘트 상수, 모터 상수 등 모터의 물리적 특성
    double max_force_{kDefaultMaxForce};
    double max_rot_velocity_{kDefaulMaxRotVelocity};
    double moment_constant_{kDefaultMomentConstant};
    double motor_constant_{kDefaultMotorConstant};
    double ref_motor_rot_vel_{0.0}; // 참조 모터 회전 속도 //0.0 이라는건 초기에 double형식으로 0이라는 뜻인듯?
    double rolling_moment_coefficient_{kDefaultRollingMomentCoefficient};
    double rotor_drag_coefficient_{kDefaultRotorDragCoefficient};
    double rotor_velocity_slowdown_sim_{kDefaultRotorVelocitySlowdownSim};
    double time_constant_down_{kDefaultTimeConstantDown};
    double time_constant_up_{kDefaultTimeConstantUp};

    bool reversible_{false}; // 모터가 역방향 회전이 가능한지 여부를 나타내는 부울 값

    transport::NodePtr node_handle_;             // Gazebo 및 ROS 통신을 위한 노드 핸들
    transport::PublisherPtr motor_velocity_pub_; // Gazebo 및 ROS 통신을 위한 퍼블리셔 객채
    transport::SubscriberPtr command_sub_;       // Gazebo 및 ROS 통신을 위한 서브스크라이버 객체
    transport::SubscriberPtr motor_failure_sub_; /*!< Subscribing to motor_failure_sub_topic_; receiving motor number to fail, as an integer */
    transport::SubscriberPtr motor_failure_sub_1;
    transport::SubscriberPtr motor_failure_sub_2;
    transport::SubscriberPtr motor_failure_sub_3;
    ////Gazebo 및 ROS 통신을 위한 서브스크라이버 객체
    transport::SubscriberPtr wind_sub_; ////Gazebo 및 ROS 통신을 위한 서브스크라이버 객체

    ignition::math::Vector3d wind_vel_; // 현재 풍속을 나타내는 벡터 //3d는 3차원 double

    physics::ModelPtr model_;
    physics::JointPtr joint_;

    // / \def JointPtr
    // / \brief Boost shared pointer to a Joint object
    // typedef boost::shared_ptr<Joint> JointPtr;
    // gazebo::physics::JointPtr 형태이다
    // gazebo::physics::boost::shared_ptr<Joint>
    // joint_ = model_->GetJoint(joint_name_); 여기서 joint_ 를 얻어온다.

    common::PID pid_;
    bool use_pid_;
    physics::LinkPtr link_;
    /// \brief Pointer to the update event connection.
    event::ConnectionPtr updateConnection_;

    boost::thread callback_queue_thread_;
    void QueueThread();
    std_msgs::msgs::Float turning_velocity_msg_;
    void VelocityCallback(CommandMotorSpeedPtr &rot_velocities); // 모터 속도 명령을 처리하는 콜백 함수
    // MotorFailureCallback 모터 고장 메시지를 처리하는 콜백 함수
    void MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg); /*!< Callback for the motor_failure_sub_ subscriber */
    void MotorFailureCallback1(const boost::shared_ptr<const msgs::Int> &fail_msg);
    void MotorFailureCallback2(const boost::shared_ptr<const msgs::Int> &fail_msg);
    void MotorFailureCallback3(const boost::shared_ptr<const msgs::Int> &fail_msg);
    void WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg); // 풍속 변경에 대응하는 콜백 함수
    // WindVelocityCallback 은  boost::shared_ptr<Publisher> 이러한 타입인 publisher가 쏘는 physics_msgs::msgs::Wind 타입의 msg를 받는다?
    // 그래서 gazebo_wind_plugin.h 파일의 physics_msgs::msgs::Wind wind_msg; 을 받는다?

    std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
    /*
      // Protobuf test
      std::string motor_test_sub_topic_;
      transport::SubscriberPtr motor_sub_;
    */
  };
}
