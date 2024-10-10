/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
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

#include "gazebo_wind_plugin.h"
#include "common.h"

namespace gazebo
{

  GazeboWindPlugin::~GazeboWindPlugin()
  {
    update_connection_->~Connection();
    delete my_wind_v;
  }

  void GazeboWindPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
  {
    world_ = world;

    double wind_gust_start = kDefaultWindGustStart;
    double wind_gust_duration = kDefaultWindGustDuration;

    double wind_ramp_start = kDefaultWindRampStart;
    double wind_ramp_duration = kDefaultWindRampDuration;

    if (sdf->HasElement("robotNamespace"))
    {
      namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    else
    {
      gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
    }

    // Create a new transport node
    // transport::NodePtr node_handle(new transport::Node()); 이렇게도 가능하다.
    // Gazebo의 transport 시스템은 시뮬레이션 내의 다양한 컴포넌트 간에 메시지 기반 통신을 가능하게 하는 매커니즘이다
    /*
     이는 Gazebo 내에서의 데이터 교환과 이벤트 처리를 위한 중요한 구조적 요소입니다.
     따라서 node_handle_은 Gazebo 환경에 특화된 노드로, Gazebo 시뮬레이션 밖에서는 사용되지 않습니다.
    */
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    getSdfParam<std::string>(sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
    double pub_rate = 2.0;
    getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate); // Wind topic publishing rates
    pub_interval_ = (pub_rate > 0.0) ? 1 / pub_rate : 0.0;
    getSdfParam<std::string>(sdf, "frameId", frame_id_, frame_id_);
    // Get the wind params from SDF.
    getSdfParam<double>(sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
    getSdfParam<double>(sdf, "windVelocityMax", wind_velocity_max_, wind_velocity_max_);
    getSdfParam<double>(sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
    getSdfParam<ignition::math::Vector3d>(sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);
    getSdfParam<double>(sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);
    // Get the wind gust params from SDF.
    getSdfParam<double>(sdf, "windGustStart", wind_gust_start, wind_gust_start);
    getSdfParam<double>(sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
    getSdfParam<double>(sdf, "windGustVelocityMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
    getSdfParam<double>(sdf, "windGustVelocityMax", wind_gust_velocity_max_, wind_gust_velocity_max_);
    getSdfParam<double>(sdf, "windGustVelocityVariance", wind_gust_velocity_variance_, wind_gust_velocity_variance_);
    getSdfParam<ignition::math::Vector3d>(sdf, "windGustDirectionMean", wind_gust_direction_mean_, wind_gust_direction_mean_);
    getSdfParam<double>(sdf, "windGustDirectionVariance", wind_gust_direction_variance_, wind_gust_direction_variance_);

    wind_direction_mean_.Normalize();
    wind_gust_direction_mean_.Normalize();
    wind_gust_start_ = common::Time(wind_gust_start);
    wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
    // Set random wind velocity mean and standard deviation
    wind_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_velocity_mean_, sqrt(wind_velocity_variance_)));
    // Set random wind direction mean and standard deviation
    wind_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.X(), sqrt(wind_direction_variance_)));
    // std::normal_distribution<double>은 정규 분포를 나타내며
    // 이 설정을 통해, 시뮬레이션에서 바람의 방향은 설정된 평균값 주변으로 정규 분포를 따라 무작위 변동하게 됩니다. 이는 자연스러운 바람의 변화를 모방하는데 사용
    wind_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Y(), sqrt(wind_direction_variance_)));
    wind_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Z(), sqrt(wind_direction_variance_)));
    // Set random wind gust velocity mean and standard deviation
    wind_gust_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_gust_velocity_mean_, sqrt(wind_gust_velocity_variance_)));
    // Set random wind gust direction mean and standard deviation
    wind_gust_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(), sqrt(wind_gust_direction_variance_)));
    wind_gust_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(), sqrt(wind_gust_direction_variance_)));
    wind_gust_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(), sqrt(wind_gust_direction_variance_)));

    // Get the ramped wind params from SDF.
    getSdfParam<double>(sdf, "windRampStart", wind_ramp_start, wind_ramp_start);
    getSdfParam<double>(sdf, "windChangeRampDuration", wind_ramp_duration, wind_ramp_duration);
    getSdfParam<ignition::math::Vector3d>(sdf, "windRampWindVectorComponents", ramped_wind_vector, ramped_wind_vector);

    wind_ramp_start_ = common::Time(wind_ramp_start);
    wind_ramp_duration_ = common::Time(wind_ramp_duration);

    // Initialize my_wind_v
    my_wind_v = new gazebo::msgs::Vector3d();
    my_wind_v->set_x(0);
    my_wind_v->set_y(0);
    my_wind_v->set_z(0);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));
    // transport::NodePtr node_handle_; // Gazebo 및 ROS 통신을 위한 노드 핸들
    // wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);
    wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/world_wind", 10);
    // wind_pub_topic_ 이놈이 world_wind 토픽을 보내고 있다.

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "ros_wind_sub", ros::init_options::NoSigintHandler);
    }
    // Create our ROS node. This acts in a similar manner to the Gazebo node
    this->rosNode = new ros::NodeHandle("ros_wind_sub");
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/my_custom_wind",
                                                                                     10, boost::bind(&GazeboWindPlugin::OnWindMsg, this, _1),
                                                                                     ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    // this->my_wind_sub = this->rosNode->subscribe<geometry_msgs::Vector3>("/my_custom_wind", 10, &GazeboWindPlugin::OnWindMsg, this);
    // 이런식으로 rosnode는 this를 붙여야한다.
    this->rosQueueThread = std::thread(std::bind(&GazeboWindPlugin::QueueThread, this));

#if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
#else
    last_time_ = world_->GetSimTime();
#endif
  }

  // This gets called by the world update start event.
  void GazeboWindPlugin::OnUpdate(const common::UpdateInfo &_info)
  {
    // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world_->SimTime();
#else
    common::Time now = world_->GetSimTime();
#endif
    if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0)
    {
      return;
    }
    last_time_ = now;

    // Calculate the wind force.
    // Get normal distribution wind strength
    double wind_strength = std::abs(wind_velocity_distribution_(wind_velocity_generator_));
    wind_strength = (wind_strength > wind_velocity_max_) ? wind_velocity_max_ : wind_strength;
    // Get normal distribution wind direction
    ignition::math::Vector3d wind_direction;
    // wind_direction의 X값
    wind_direction.X() = wind_direction_distribution_X_(wind_direction_generator_);
    // wind_direction_generator_: 난수를 생성하는 엔진
    // 이 라인의 실행은 wind_direction.X()에 새로운 값을 할당하는데, 그 값은 wind_direction_distribution_X_ 분포에 따라 wind_direction_generator_로 생성된 난수입니다
    wind_direction.Y() = wind_direction_distribution_Y_(wind_direction_generator_);
    wind_direction.Z() = wind_direction_distribution_Z_(wind_direction_generator_);
    // Calculate total wind velocity
    ignition::math::Vector3d wind = wind_strength * wind_direction;
    // 여기서 구한 wind가 160번째줄에 사용

    ignition::math::Vector3d wind_gust(0, 0, 0);
    // Calculate the wind gust velocity.
    if (now >= wind_gust_start_ && now < wind_gust_end_)
    {
      // Get normal distribution wind gust strength
      double wind_gust_strength = std::abs(wind_gust_velocity_distribution_(wind_gust_velocity_generator_));
      wind_gust_strength = (wind_gust_strength > wind_gust_velocity_max_) ? wind_gust_velocity_max_ : wind_gust_strength;
      // Get normal distribution wind gust direction
      ignition::math::Vector3d wind_gust_direction;
      wind_gust_direction.X() = wind_gust_direction_distribution_X_(wind_gust_direction_generator_);
      wind_gust_direction.Y() = wind_gust_direction_distribution_Y_(wind_gust_direction_generator_);
      wind_gust_direction.Z() = wind_gust_direction_distribution_Z_(wind_gust_direction_generator_);
      wind_gust = wind_gust_strength * wind_gust_direction;
    }

    // Calculate the wind with the added ramped up wind component
    double ramp_factor = 0.;
    if (wind_ramp_duration_.Double() > 0)
    {
      ramp_factor = constrain((now - wind_ramp_start_).Double() / wind_ramp_duration_.Double(), 0., 1.);
    }

    wind += ramp_factor * ramped_wind_vector;
    // wind = wind + (ramp_factor * ramped_wind_vector)
    // 여기서 wind는 ignition::math::Vector3d wind 타입이다.
    gazebo::msgs::Vector3d *wind_v = new gazebo::msgs::Vector3d();
    // new 란? int* ptr = new int; // 정수를 저장할 수 있는 메모리를 할당하고, 그 주소를 ptr에 저장합니다.
    // *ptr = 5; // 할당된 메모리에 5를 저장합니다.
    // 이 코드는 특히 Gazebo의 메시지 전송 시스템에서 사용될 벡터 데이터를 생성하고 초기화할 때 사용됩니다.

    wind_v->set_x(wind.X() + wind_gust.X());
    wind_v->set_y(wind.Y() + wind_gust.Y());
    wind_v->set_z(wind.Z() + wind_gust.Z());
    // std::cout << "[wind_v befor change] :  (" << wind_v->x() << ", " << wind_v->y() << ", " << wind_v->z() << ")" << std::endl;

    // std::cout << "[my_wind_v in Load] :  (" << my_wind_v->x() << ", " << my_wind_v->y() << ", " << my_wind_v->z() << ")" << std::endl;

    if (my_wind_v->x() != 0 || my_wind_v->y() != 0 || my_wind_v->z() != 0)
    {
      wind_v->set_x(my_wind_v->x());
      wind_v->set_y(my_wind_v->y());
      wind_v->set_z(my_wind_v->z());
      std::cout << "[wind_v after  change] :  (" << wind_v->x() << ", " << wind_v->y() << ", " << wind_v->z() << ")" << std::endl;
    }

    // std::cout << "[wind_v in Onupdate] :  (" << wind_v->x() << ", " << wind_v->y() << ", " << wind_v->z() << ")" << std::endl;

    wind_msg.set_frame_id(frame_id_);
    wind_msg.set_time_usec(now.Double() * 1e6);
    wind_msg.set_allocated_velocity(wind_v);
    // set_allocated_velocity는 gazebo::msgs::Vector3d 타입의 포인터를 받는다 즉, wind_v는 gazebo::msgs::Vector3d이다.
    /*
    inline void Wind::set_allocated_velocity(::gazebo::msgs::Vector3d* velocity) {
    ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
    if (message_arena == NULL) {
      delete reinterpret_cast< ::google::protobuf::MessageLite*>(velocity_);
    }
    if (velocity) {
      ::google::protobuf::Arena* submessage_arena = NULL;
      if (message_arena != submessage_arena) {
        velocity = ::google::protobuf::internal::GetOwnedMessage(
            message_arena, velocity, submessage_arena);
      }
      set_has_velocity();
    } else {
      clear_has_velocity();
    }
    velocity_ = velocity;
    // @@protoc_insertion_point(field_set_allocated:physics_msgs.msgs.Wind.velocity)
    }
    */
    // velocity_ 변수에 velocity의 포인터를 할당 즉, velocity_에 wind_v 의 포인터가 할당된다.
    // physics_msgs::msgs::Wind 타입인 wind_msg 이다.

    // my_wind_msg.set_frame_id(frame_id_); // frame_id_ 설정 필요
    // my_wind_msg.set_time_usec(now.Double() * 1e6); // now 업데이트 필요
    // my_wind_msg.set_allocated_velocity(my_wind_v);
    // wind_pub_->Publish(my_wind_msg);
    wind_pub_->Publish(wind_msg);
    // gazebo::msgs::Vector3d 타입인 wind_v를 wind_pub이 pub 한다.
    // gazebo_wind_plugin.h 파일의 physics_msgs::msgs::Wind wind_msg 를 보낸다.
    // 보낸 wind_msg 는 void GazeboMotorModel::WindVelocityCallback(WindPtr& msg) 에서 WindVelocityCallback 이 콜백 함수가 받는다.
  }

  void GazeboWindPlugin::QueueThread()
  {
    static const double timeout = 0.01;

    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboWindPlugin::OnWindMsg(const geometry_msgs::Vector3::ConstPtr &msg)
  {
    ignition::math::Vector3d my_wind = ignition::math::Vector3d(msg->x, msg->y, msg->z);

    my_wind_v->set_x(my_wind.X());
    my_wind_v->set_y(my_wind.Y());
    my_wind_v->set_z(my_wind.Z());

    std::cout << "[my_wind_v in OnWindMsg] :  (" << my_wind_v->x() << ", " << my_wind_v->y() << ", " << my_wind_v->z() << ")" << std::endl;

    // wind_pub_->Publish(my_wind_msg); // my_wind_pub_ 설정 및 초기화 필요
  }

  GZ_REGISTER_WORLD_PLUGIN(GazeboWindPlugin); // 월드 일경우 GZ_REGISTER_WORLD_PLUGIN 마무리
  // GZ_REGISTER_MODEL_PLUGIN 모델이면 이걸로 마무리
  // GZ_REGISTER_SENSOR_PLUGIN
  // GZ_REGISTER_GUI_PLUGIN
  // GZ_REGISTER_SYSTEM_PLUGIN
  // GZ_REGISTER_VISUAL_PLUGIN
}
