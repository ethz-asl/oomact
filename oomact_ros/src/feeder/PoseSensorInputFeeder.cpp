#include <aslam/calibration/data/ObservationManagerI.h>
#include <aslam/calibration/input/InputRecieverI.h>
#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/data/PoseMeasurement.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>

#include "aslam/calibration/ros/InputFeederFactoryRegistry.h"
#include "aslam/calibration/ros/InputFeederFactoryI.h"
#include "aslam/calibration/ros/InputFeederI.h"

namespace aslam {
namespace calibration {
namespace ros {


typedef InputReceiverIT<PoseMeasurement> Receiver;

class PoseSensorInputFeeder : public InputFeederImpl<geometry_msgs::PoseStamped, Receiver, PoseSensorInputFeeder> {
 public:
  using InputFeederImpl<geometry_msgs::PoseStamped, Receiver, PoseSensorInputFeeder>::InputFeederImpl;
  virtual ~PoseSensorInputFeeder() = default;

  void feed(const geometry_msgs::PoseStamped & m, const Receiver & receiver, ObservationManagerI & obsManager) const {
    PoseMeasurement pm {
      Eigen::Vector3d(m.pose.position.x, m.pose.position.y, m.pose.position.z),
      Eigen::Vector4d(m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w),
    };
    auto t = Timestamp::fromNumerator(m.header.stamp.toNSec());
    receiver.addInputTo(t, pm, obsManager.getCurrentStorage());

    const Sensor & sensor = dynamic_cast<const Sensor&>(receiver.getModule());
    VLOG(3) << "Sensor=" << sensor.getName() << " : t =" << obsManager.secsSinceStart(t) << " secs.";
    obsManager.addMeasurementTimestamp(t, sensor);
  }
};

namespace {
  class IFF : public InputFeederFactoryIT<Receiver, PoseSensorInputFeeder, IFF> {};
  InputFeederFactoryRegistry::RegistryEntry regEntry(new IFF);
}

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */
