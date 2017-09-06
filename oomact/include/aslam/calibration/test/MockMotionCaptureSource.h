#ifndef H481BD7A1_2233_4044_B4F2_CDD02D575958
#define H481BD7A1_2233_4044_B4F2_CDD02D575958

#include <functional>
#include <vector>

#include "../input/MotionCaptureSource.hpp"

namespace aslam {
namespace calibration {
namespace test {
/**
 * A mock motion capture source sampling at 100Hz from a function providing a pose given start and current time.
 */
class MockMotionCaptureSource : public MotionCaptureSource {
 public:
  constexpr static Timestamp StartTime = Timestamp::Zero();

  MockMotionCaptureSource(std::function<void(Timestamp now, PoseStamped & p)> func) : func(func){}
  virtual ~MockMotionCaptureSource();
  virtual std::vector<PoseStamped> getPoses(Timestamp from, Timestamp till) const override;
  virtual std::vector<PoseStamped> getPoses(Timestamp till) const { return getPoses(StartTime, till); }
  virtual PoseStamped getPoseAt(Timestamp at) const;
 private:
  std::function<void(Timestamp now, PoseStamped & p)> func;
};

extern MockMotionCaptureSource MmcsStraightLine;
extern MockMotionCaptureSource MmcsRotatingStraightLine;
extern MockMotionCaptureSource MmcsCircle;

} /* namespace test */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H481BD7A1_2233_4044_B4F2_CDD02D575958 */
