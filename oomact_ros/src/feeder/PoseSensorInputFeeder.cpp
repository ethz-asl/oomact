#include <aslam/calibration/data/ObservationManagerI.h>
#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/data/PoseMeasurement.h>
#include <aslam/calibration/input/InputReceiverI.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include "aslam/calibration/ros/InputFeederFactoryRegistry.h"
#include "aslam/calibration/ros/InputFeederFactoryI.h"
#include "aslam/calibration/ros/InputFeederTemplates.h"

#include "aslam/calibration/ros/internal/Tools.h"
namespace aslam {
namespace calibration {
namespace ros {

bool msg2Measurement(const geometry_msgs::TransformStamped &msg, PoseMeasurement & m){
  m.t = rosVector3dToEigenVector3(msg.transform.translation);
  static_assert(PoseMeasurement::USE_XYZW_ORDER, "");
  m.q = rosQuaternionToVector4dXYZW(msg.transform.rotation, PoseMeasurement::USE_JPL_MULT);
  return true;
}

bool msg2Measurement(const geometry_msgs::PoseStamped &msg, PoseMeasurement & m){
  m.t = rosVector3dToEigenVector3(msg.pose.position);
  static_assert(PoseMeasurement::USE_XYZW_ORDER, "");
  m.q = rosQuaternionToVector4dXYZW(msg.pose.orientation, PoseMeasurement::USE_JPL_MULT);
  return true;
}

namespace {
InputFeederFactoryRegistry::RegistryEntry regEntries[] = {
    new InputFeederFactoryForMessageWithHeader<geometry_msgs::PoseStamped, PoseMeasurement>,
    new InputFeederFactoryForMessageWithHeader<geometry_msgs::TransformStamped, PoseMeasurement>,
};
}

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */
