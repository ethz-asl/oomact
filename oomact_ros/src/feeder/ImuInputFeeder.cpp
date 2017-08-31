#include <aslam/calibration/data/ObservationManagerI.h>
#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/data/AccelerometerMeasurement.h>
#include <aslam/calibration/data/GyroscopeMeasurement.h>
#include <aslam/calibration/input/InputReceiverI.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

#include "aslam/calibration/ros/InputFeederFactoryRegistry.h"
#include "aslam/calibration/ros/InputFeederFactoryI.h"
#include "aslam/calibration/ros/InputFeederTemplates.h"

namespace aslam {
namespace calibration {
namespace ros {

bool msg2Measurement(const sensor_msgs::Imu &msg, AccelerometerMeasurement & m){
  if(msg.linear_acceleration_covariance[0] < 0.0){ // no measurement available (http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
    return false;
  }
  m.a = rosVector3dToEigenVector3(msg.linear_acceleration);
  m.cov = rosArray9CovToEigenMatrix3(msg.linear_acceleration_covariance);
  return true;
}

bool msg2Measurement(const sensor_msgs::Imu &msg, GyroscopeMeasurement & m){
  if(msg.angular_velocity_covariance[0] < 0.0){ // no measurement available (http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
    return false;
  }
  m.w = rosVector3dToEigenVector3(msg.angular_velocity);
  m.cov = rosArray9CovToEigenMatrix3(msg.angular_velocity_covariance);
  return true;
}

namespace {
InputFeederFactoryRegistry::RegistryEntry regEntries[] = {
    new InputFeederFactoryForMessageWithHeader<sensor_msgs::Imu, AccelerometerMeasurement>,
    new InputFeederFactoryForMessageWithHeader<sensor_msgs::Imu, GyroscopeMeasurement>
};

}

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */
