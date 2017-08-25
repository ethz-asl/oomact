#ifndef H481BD7A1_2233_4044_B4F2_CDD02D575958
#define H481BD7A1_2233_4044_B4F2_CDD02D575958

#include <functional>
#include <vector>

#include "../algo/MotionCaptureSource.hpp"

namespace aslam {
namespace calibration {
namespace test {
/**
 * A mock motion capture source sampling at 100Hz from a function providing a pose given start and current time.
 */
class MockMotionCaptureSource : public MotionCaptureSource {
 public:
  MockMotionCaptureSource(std::function<void(Timestamp start, Timestamp now, PoseStamped & p)> func) : func(func){}
  virtual ~MockMotionCaptureSource();
  virtual std::vector<PoseStamped> getPoses(Timestamp from, Timestamp till) const override;
  virtual PoseStamped getPoseAt(Timestamp start, Timestamp at) const;
 private:
  std::function<void(Timestamp start, Timestamp now, PoseStamped & p)> func;
};

extern MockMotionCaptureSource mmcsStraightLine;

extern MockMotionCaptureSource mmcsRotatingStraightLine;

} /* namespace test */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H481BD7A1_2233_4044_B4F2_CDD02D575958 */
