#include <aslam/calibration/test/MockMotionCaptureSource.h>
#include <aslam/calibration/Timestamp.hpp>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>

using namespace aslam::calibration;

void writeMotionCaptureSourceToBag(std::string path, std::string topic, const test::MockMotionCaptureSource & mcs, Timestamp till){
  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Write);

  for(auto & m: mcs.getPoses(till)){
    geometry_msgs::PoseStamped ps;
    ros::Duration dur;
    dur.fromNSec(m.time.getNumerator());
    ps.header.stamp = ros::TIME_MIN + dur;
    ps.pose.orientation.x = m.q[0];
    ps.pose.orientation.y = m.q[1];
    ps.pose.orientation.z = m.q[2];
    ps.pose.orientation.w = m.q[3];
    ps.pose.position.x = m.p[0];
    ps.pose.position.x = m.p[1];
    ps.pose.position.x = m.p[2];

    bag.write(topic, ps.header.stamp, ps);
  }
  bag.close();
}
