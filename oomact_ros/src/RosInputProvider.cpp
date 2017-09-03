#include "aslam/calibration/ros/RosInputProvider.h"

#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/Sensor.hpp>
#include <rosbag/view.h>

#include "aslam/calibration/ros/InputFeederFactoryRegistry.h"

namespace aslam {
namespace calibration {

namespace ros {

std::string getTopic(const Sensor & s) {
  return s.getMyConfig().getString("topic");
}

RosInputProvider::RosInputProvider(const std::shared_ptr<const Model> & model) : model_(model) {
  auto && sensors = model_->getSensors();

  LOG(INFO) << "Creating feeders from " << sensors.size() << " sensors.";
  std::vector<std::reference_wrapper<Sensor>> remaining;
  InputFeederFactoryRegistry::applyToMatching(sensors, [this](const Sensor & s, std::unique_ptr<InputFeederI> feeder){
    std::string topic = getTopic(s);
    LOG(INFO) << "Feeding topic " << topic << " with " << *feeder;
    topic2FeedersMap_.emplace(topic, std::move(feeder));
  }, &remaining);
  LOG(INFO) << "Created " << topic2FeedersMap_.size() << " feeders from " << sensors.size() << " sensors.";
  for(Sensor & s : remaining){
    if(s.isUsed()){
      LOG(WARNING) << "Could not create any feeder for " << s.getName() << "!";
    }
  }
}

RosInputProvider::~RosInputProvider() {
}

void RosInputProvider::feedMessage(const rosbag::MessageInstance& m, ObservationManagerI& obsManager) {
  auto equal_range = topic2FeedersMap_.equal_range(m.getTopic());
  if (equal_range.first != topic2FeedersMap_.end()) {
    for (auto it = equal_range.first; it != equal_range.second; it++) {
      it->second->feed(m, obsManager);
    }
  }
}

void RosInputProvider::feedBag(const std::string& bagPath, ObservationManagerI& obsManager) {
  rosbag::Bag bag;
  bag.open(bagPath, rosbag::bagmode::Read);
  feedBag(bag, obsManager);
}

void RosInputProvider::feedBag(const rosbag::Bag& bag, ObservationManagerI & obsManager) {
  for(auto & m : rosbag::View(bag))
  {
    feedMessage(m, obsManager);
  }
}

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

