#ifndef H5DCE57E7_B971_44A7_9CEB_6477D70B9F4C
#define H5DCE57E7_B971_44A7_9CEB_6477D70B9F4C

#include <memory>
#include <unordered_map>
#include <vector>

#include <aslam/calibration/data/ObservationManagerI.h>
#include <rosbag/bag.h>

#include "InputFeederI.h"

namespace aslam {
namespace calibration {
namespace ros {

class RosInputProvider {
 public:
  RosInputProvider(const std::shared_ptr<const Model> & model);
  virtual ~RosInputProvider();

  void feedBag(const std::string & bagPath, ObservationManagerI & obsManager);
  void feedBag(const rosbag::Bag & bag, ObservationManagerI & obsManager);
  void feedMessage(const rosbag::MessageInstance& m, ObservationManagerI& obsManager);
 private:
  std::shared_ptr<const Model> model_;
  std::unordered_multimap<std::string, std::unique_ptr<InputFeederI>> topic2FeedersMap_;
};

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H5DCE57E7_B971_44A7_9CEB_6477D70B9F4C */
