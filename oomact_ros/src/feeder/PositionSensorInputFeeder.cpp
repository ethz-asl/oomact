#include <aslam/calibration/data/ObservationManagerI.h>
#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/data/PositionMeasurement.h>
#include <aslam/calibration/input/InputReceiverI.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "aslam/calibration/ros/InputFeederFactoryRegistry.h"
#include "aslam/calibration/ros/InputFeederFactoryI.h"
#include "aslam/calibration/ros/InputFeederTemplates.h"

#include "aslam/calibration/ros/internal/Tools.h"
namespace aslam {
namespace calibration {
namespace ros {

bool msg2Measurement(const geometry_msgs::PointStamped &msg, PositionMeasurement & m){
  m.t = rosVector3dToEigenVector3(msg.point);
  return true;
}

bool msg2Measurement(const geometry_msgs::Vector3Stamped &msg, PositionMeasurement & m){
  m.t = rosVector3dToEigenVector3(msg.vector);
  return true;
}

namespace {
InputFeederFactoryRegistry::RegistryEntry regEntries[] = {
    new InputFeederFactoryForMessageWithHeader<geometry_msgs::PointStamped, PositionMeasurement>,
    new InputFeederFactoryForMessageWithHeader<geometry_msgs::Vector3Stamped, PositionMeasurement>,
};
}

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */
