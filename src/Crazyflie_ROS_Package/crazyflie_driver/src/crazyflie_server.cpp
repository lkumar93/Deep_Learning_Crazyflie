#include "ros/ros.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"

//#include <regex>
#include <thread>
#include <mutex>

#include <crazyflie_cpp/Crazyflie.h>

constexpr double pi() { return std::atan(1)*4; }

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    float roll_trim,
    float pitch_trim,
    bool enable_logging,
    bool enable_parameters,
    std::vector<crazyflie_driver::LogBlock>& log_blocks,
    bool use_ros_time,
    bool enable_logging_imu,
    bool enable_logging_temperature,
    bool enable_logging_magnetic_field,
    bool enable_logging_pressure,
    bool enable_logging_battery)
    : m_cf(link_uri)
    , m_tf_prefix(tf_prefix)
    , m_isEmergency(false)
    , m_roll_trim(roll_trim)
    , m_pitch_trim(pitch_trim)
    , m_enableLogging(enable_logging)
    , m_enableParameters(enable_parameters)
    , m_logBlocks(log_blocks)
    , m_use_ros_time(use_ros_time)
    , m_enable_logging_imu(enable_logging_imu)
    , m_enable_logging_temperature(enable_logging_temperature)
    , m_enable_logging_magnetic_field(enable_logging_magnetic_field)
    , m_enable_logging_pressure(enable_logging_pressure)
    , m_enable_logging_battery(enable_logging_battery)
    , m_serviceEmergency()
    , m_serviceUpdateParams()
    , m_subscribeCmdVel()
    , m_pubGyro()
    , m_pubAccel()
    , m_pubOrientation()
    , m_pubOrientationQuaternion()
    , m_pubGravity()
    , m_pubAccZBase()
    , m_pubInitialized()
    , m_pubTemp()
    , m_pubMag()
    , m_pubPressure()
    , m_pubHeight()
    , m_pubBattery()
    , m_pubRssi()
    , m_sentSetpoint(false)
  {
    ros::NodeHandle n;
    m_subscribeCmdVel = n.subscribe(tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
    m_subscribeDeepLearntCmdVel = n.subscribe(tf_prefix + "/deep_learning/cmd_vel", 1, &CrazyflieROS::cmdVelChangedDeepLearnt, this);
    m_serviceEmergency = n.advertiseService(tf_prefix + "/emergency", &CrazyflieROS::emergency, this);
    m_serviceUpdateParams = n.advertiseService(tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);

    if (m_enable_logging_imu) {
      m_pubGyro = n.advertise<geometry_msgs::Vector3Stamped>(tf_prefix + "/gyro", 10);
      m_pubAccel = n.advertise<geometry_msgs::Vector3Stamped>(tf_prefix + "/accel", 10);
      m_pubOrientation = n.advertise<geometry_msgs::Vector3Stamped>(tf_prefix + "/orientation/euler", 10);
      m_pubOrientationQuaternion = n.advertise<geometry_msgs::QuaternionStamped>(tf_prefix + "/orientation/quaternion", 10);
      m_pubGravity = n.advertise<geometry_msgs::Vector3Stamped>(tf_prefix + "/gravity_direction", 10);
      m_pubAccZBase = n.advertise<geometry_msgs::Vector3Stamped>(tf_prefix + "/accel/base_z", 10);
      m_pubInitialized = n.advertise<std_msgs::UInt8>(tf_prefix + "/sensors/initialized", 10);
    }
    if (m_enable_logging_temperature) {
      m_pubTemp = n.advertise<sensor_msgs::Temperature>(tf_prefix + "/temperature", 10);
    }
    if (m_enable_logging_magnetic_field) {
      m_pubMag = n.advertise<sensor_msgs::MagneticField>(tf_prefix + "/magnetic_field", 10);
    }
    if (m_enable_logging_pressure) {
      m_pubPressure = n.advertise<std_msgs::Float32>(tf_prefix + "/pressure", 10);
      m_pubHeight = n.advertise<geometry_msgs::Vector3Stamped>(tf_prefix + "/height", 1);
    }
    if (m_enable_logging_battery) {
      m_pubBattery = n.advertise<std_msgs::Float32>(tf_prefix + "/battery", 10);
    }
    m_pubRssi = n.advertise<std_msgs::Float32>(tf_prefix + "/rssi", 10);

    for (auto& logBlock : m_logBlocks)
    {
      m_pubLogDataGeneric.push_back(n.advertise<crazyflie_driver::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
    }

    std::thread t(&CrazyflieROS::run, this);
    t.detach();
  }

private:
  struct logImu {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } __attribute__((packed));

  struct log2 {
    float mag_x;
    float mag_y;
    float mag_z;
    float baro_temp;
    float baro_pressure;
    float pm_vbat;
  } __attribute__((packed));


  struct logOrientation {
    float roll;
    float pitch;
    float yaw;
    float height;
    uint16_t thrust;
    float asl;
  } __attribute__((packed));

  struct logSensorFusion {
    float qw;
    float qx;
    float qy;
    float qz;
    float accZbase;
    uint8_t isInit;
  } __attribute__((packed));

  struct logGravityVector{
    float grav_x;
    float grav_y;
    float grav_z;
  } __attribute__((packed));


private:
  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    m_isEmergency = true;

    return true;
  }

  template<class T, class U>
  void updateParam(uint8_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.setParam<T>(id, (T)value);
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("Update parameters");
    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    return true;
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!m_isEmergency) {
      float roll = msg->linear.y + m_roll_trim;
      float pitch = - (msg->linear.x + m_pitch_trim);
      float yawrate = msg->angular.z;
      uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

      m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
      m_sentSetpoint = true;
    }
  }

  void cmdVelChangedDeepLearnt(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!m_isEmergency) {
      deeplearnt_roll = msg->linear.y ;
      deeplearnt_pitch = -(msg->linear.x);
      deeplearnt_yawrate = msg->angular.z;
      deeplearnt_thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

     // m_cf.sendSetpoint(deeplearnt_roll, deeplearnt_pitch, deeplearnt_yawrate, deeplearnt_thrust);
      //m_sentSetpoint = true;

      ROS_INFO("Inside cmdVelChangedDeepLearnt block %d",deeplearnt_thrust);
    }
  }

  void run()
  {
    // m_cf.reboot();

    m_cf.logReset();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);

    auto start = std::chrono::system_clock::now();

    if (m_enableParameters)
    {
      ROS_INFO("Requesting parameters...");
      m_cf.requestParamToc();
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
      }
    }

    std::unique_ptr<LogBlock<logImu> > logBlockImu;
    std::unique_ptr<LogBlock<logOrientation> > logBlockOrientation;
    std::unique_ptr<LogBlock<logSensorFusion> > logBlockSensorFusion;
    std::unique_ptr<LogBlock<logGravityVector> > logBlockGravityVector;
    std::unique_ptr<LogBlock<log2> > logBlock2;
    std::vector<std::unique_ptr<LogBlockGeneric> > logBlocksGeneric(m_logBlocks.size());
    if (m_enableLogging) {

      std::function<void(const crtpPlatformRSSIAck*)> cb_ack = std::bind(&CrazyflieROS::onEmptyAck, this, std::placeholders::_1);
      m_cf.setEmptyAckCallback(cb_ack);

      ROS_INFO("Requesting Logging variables...");
      m_cf.requestLogToc();

      if (m_enable_logging_imu) {
        std::function<void(uint32_t, logImu*)> cb = std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockImu.reset(new LogBlock<logImu>(
          &m_cf,{
            {"acc", "x"},
            {"acc", "y"},
            {"acc", "z"},
            {"gyro", "x"},
            {"gyro", "y"},
            {"gyro", "z"},

          }, cb));
        logBlockImu->start(1); // 10ms

	std::function<void(uint32_t, logOrientation*)> cb3 = std::bind(&CrazyflieROS::onOrientationData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockOrientation.reset(new LogBlock<logOrientation>(
          &m_cf,{
            {"stabilizer", "roll"},
            {"stabilizer", "pitch"},
            {"stabilizer", "yaw"},
            {"posEstimatorAlt", "estimatedZ"},
            {"stabilizer", "thrust"},
	    {"baro", "asl"},
 
          }, cb3));
        logBlockOrientation->start(1); // 10ms

	std::function<void(uint32_t, logSensorFusion*)> cb4 = std::bind(&CrazyflieROS::onSensorFusionData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockSensorFusion.reset(new LogBlock<logSensorFusion>(
          &m_cf,{
            {"sensorfusion6", "qw"},
            {"sensorfusion6", "qx"},
            {"sensorfusion6", "qy"},
            {"sensorfusion6", "qz"},
            {"sensorfusion6", "accZbase"},
	    {"sensorfusion6", "isInit"},
 
          }, cb4));
        logBlockSensorFusion->start(1); // 10ms

	std::function<void(uint32_t, logGravityVector*)> cb5 = std::bind(&CrazyflieROS::onGravityVectorData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockGravityVector.reset(new LogBlock<logGravityVector>(
          &m_cf,{
            {"sensorfusion6", "gravityX"},
            {"sensorfusion6", "gravityY"},
            {"sensorfusion6", "gravityZ"},
 
          }, cb5));
        logBlockGravityVector->start(1); // 10ms

      }


      if (   m_enable_logging_temperature
          || m_enable_logging_magnetic_field
          || m_enable_logging_pressure
          || m_enable_logging_battery)
      {
        std::function<void(uint32_t, log2*)> cb2 = std::bind(&CrazyflieROS::onLog2Data, this, std::placeholders::_1, std::placeholders::_2);

        logBlock2.reset(new LogBlock<log2>(
          &m_cf,{
	    {"mag","x"},
	    {"mag","y"},
            {"mag","z"},
            {"baro", "temp"},
            {"baro", "pressure"},
            {"pm", "vbat"},
          }, cb2));
        logBlock2->start(1); // 10ms
      }

      // custom log blocks
      size_t i = 0;
      for (auto& logBlock : m_logBlocks)
      {
        std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
          std::bind(
            &CrazyflieROS::onLogCustom,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

        logBlocksGeneric[i].reset(new LogBlockGeneric(
          &m_cf,
          logBlock.variables,
          (void*)&m_pubLogDataGeneric[i],
          cb));
        logBlocksGeneric[i]->start(logBlock.frequency / 10);
        ++i;
      }

    }

   
    //std::string AltHoldParam = "/crazyflie/flightmode/althold" ;
    //uint8_t AltHoldFlag = 1;
    //updateParam<int>(AltHoldFlag,AltHoldParam);

    

    ROS_INFO("Ready...");
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("Elapsed: %f s", elapsedSeconds.count());

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // make sure we ping often enough to stream data out
      if (m_enableLogging && !m_sentSetpoint) {
        m_cf.sendPing();
      }
      m_cf.sendSetpoint(deeplearnt_roll, deeplearnt_pitch, deeplearnt_yawrate, deeplearnt_thrust);
      m_sentSetpoint = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Make sure we turn the engines off
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

  }

  void onImuData(uint32_t time_in_ms, logImu* data) {
    if (m_enable_logging_imu) {

      geometry_msgs::Vector3Stamped msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
 
      // measured in deg/s; need to convert to rad/s
      msg.vector.x = degToRad(data->gyro_x);
      msg.vector.y = degToRad(data->gyro_y);
      msg.vector.z = degToRad(data->gyro_z);

      m_pubGyro.publish(msg);

      // measured in mG; need to convert to m/s^2
      msg.vector.x = data->acc_x * 9.81;
      msg.vector.y = data->acc_y * 9.81;
      msg.vector.z = data->acc_z * 9.81;

      m_pubAccel.publish(msg);
 
      
    }
  }

  void onOrientationData(uint32_t time_in_ms, logOrientation* data) {

   //ROS_INFO("Roll = %f , Pitch = %f , Yaw = %f, Height = %f, Thrust = %d, ASL = %f",data->pitch,data->roll,data->yaw,(data->height),data->thrust, data->asl);

   geometry_msgs::Vector3Stamped msg;

   if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
     msg.header.frame_id = m_tf_prefix + "/base_link";

     
      // hPa (=mbar)
     msg.vector.z = data->height;

     m_pubHeight.publish(msg);
  
     msg.vector.x = data->pitch ;
     msg.vector.y = data->roll ;
     msg.vector.z = data->yaw ;

     m_pubOrientation.publish(msg);

  }

  void onSensorFusionData(uint32_t time_in_ms, logSensorFusion* data) {

   ROS_INFO("qw = %f , qx = %f , qy = %f, qz = %f, accZbase = %f, isInit = %d",data->qw,data->qx,data->qy,(data->qz),data->accZbase, data->isInit);

   geometry_msgs::Vector3Stamped msg;
   geometry_msgs::QuaternionStamped quat_msg;
   std_msgs::UInt8 init_msg;

   if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
      quat_msg.header.stamp = ros::Time::now();

    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      quat_msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
     msg.header.frame_id = m_tf_prefix + "/base_link";
     quat_msg.header.frame_id = m_tf_prefix + "/base_link";

     
      // hPa (=mbar)

     msg.vector.z = data->accZbase ;

     m_pubAccZBase.publish(msg);

     quat_msg.quaternion.x = data->qx;
     quat_msg.quaternion.y = data->qy;
     quat_msg.quaternion.z = data->qz;
     quat_msg.quaternion.w = data->qw;

     m_pubOrientationQuaternion.publish(quat_msg);

     init_msg.data = data->isInit;

     m_pubInitialized.publish(init_msg);     

  }

 void onGravityVectorData(uint32_t time_in_ms, logGravityVector* data) {

   ROS_INFO("Grav X = %f , Grav Y = %f , Grav Z = %f", data->grav_x, data->grav_y, data->grav_z);

   geometry_msgs::Vector3Stamped msg;

   if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
     msg.header.frame_id = m_tf_prefix + "/base_link";

     msg.vector.x = data->grav_x ;
     msg.vector.y = data->grav_y ;
     msg.vector.z = data->grav_z ;

     m_pubGravity.publish(msg);

  }

  void onLog2Data(uint32_t time_in_ms, log2* data) {

    if (m_enable_logging_temperature) {
      sensor_msgs::Temperature msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
      // measured in degC
      msg.temperature = data->baro_temp;
      m_pubTemp.publish(msg);
    }

    if (m_enable_logging_magnetic_field) {
      sensor_msgs::MagneticField msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";

      // measured in Tesla
      msg.magnetic_field.x = data->mag_x;
      msg.magnetic_field.y = data->mag_y;
      msg.magnetic_field.z = data->mag_z;
      m_pubMag.publish(msg);
    }


    if (m_enable_logging_pressure) {
      std_msgs::Float32 msg;
      // hPa (=mbar)
      msg.data = data->baro_pressure;
      m_pubPressure.publish(msg);
    }

    if (m_enable_logging_battery) {
      std_msgs::Float32 msg;
      // V
      msg.data = data->pm_vbat;
      m_pubBattery.publish(msg);
    }
  }

  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

    ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

    crazyflie_driver::GenericLogData msg;
    if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
    msg.header.frame_id = m_tf_prefix + "/base_link";
    msg.values = *values;

    pub->publish(msg);
  }

  void onEmptyAck(const crtpPlatformRSSIAck* data) {
      std_msgs::Float32 msg;
      // dB
      msg.data = data->rssi;
      m_pubRssi.publish(msg);
  }

  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        ROS_WARN("Link Quality low (%f)", linkQuality);
      }
  }

private:
  Crazyflie m_cf;
  std::string m_tf_prefix;
  bool m_isEmergency;
  float m_roll_trim;
  float m_pitch_trim;
  bool m_enableLogging;
  bool m_enableParameters;
  float deeplearnt_roll = 0.0;
  float deeplearnt_pitch =0.0;
  float deeplearnt_yawrate = 0.0;
  uint16_t deeplearnt_thrust = 0;
  std::vector<crazyflie_driver::LogBlock> m_logBlocks;
  bool m_use_ros_time;
  bool m_enable_logging_imu;
  bool m_enable_logging_temperature;
  bool m_enable_logging_magnetic_field;
  bool m_enable_logging_pressure;
  bool m_enable_logging_battery;

  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceUpdateParams;
  ros::Subscriber m_subscribeCmdVel;
  ros::Subscriber m_subscribeDeepLearntCmdVel;
  ros::Publisher m_pubAccel;
  ros::Publisher m_pubGyro;
  ros::Publisher m_pubOrientation;
  ros::Publisher m_pubOrientationQuaternion;
  ros::Publisher m_pubGravity;
  ros::Publisher m_pubAccZBase;
  ros::Publisher m_pubInitialized;
  ros::Publisher m_pubTemp;
  ros::Publisher m_pubMag;
  ros::Publisher m_pubPressure;
  ros::Publisher m_pubHeight;
  ros::Publisher m_pubBattery;
  ros::Publisher m_pubRssi;
  std::vector<ros::Publisher> m_pubLogDataGeneric;

  bool m_sentSetpoint;
};

bool add_crazyflie(
  crazyflie_driver::AddCrazyflie::Request  &req,
  crazyflie_driver::AddCrazyflie::Response &res)
{
  ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d, Parameters: %d, Use ROS time: %d",
    req.uri.c_str(),
    req.tf_prefix.c_str(),
    req.roll_trim,
    req.pitch_trim,
    req.enable_parameters,
    req.enable_logging,
    req.use_ros_time);

  // Leak intentionally
  CrazyflieROS* cf = new CrazyflieROS(
    req.uri,
    req.tf_prefix,
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging,
    req.enable_parameters,
    req.log_blocks,
    req.use_ros_time,
    req.enable_logging_imu,
    req.enable_logging_temperature,
    req.enable_logging_magnetic_field,
    req.enable_logging_pressure,
    req.enable_logging_battery);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_crazyflie", add_crazyflie);
  ros::spin();

  return 0;
}
