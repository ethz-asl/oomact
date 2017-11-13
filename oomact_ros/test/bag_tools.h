#ifndef H12CC03ED_5B11_4DE8_AE61_CF57E441DF86
#define H12CC03ED_5B11_4DE8_AE61_CF57E441DF86

#include <aslam/calibration/test/MockMotionCaptureSource.h>
#include <aslam/calibration/Timestamp.h>

void writeMotionCaptureSourceToBag(std::string path, std::string topic, const aslam::calibration::test::MockMotionCaptureSource & mcs, aslam::calibration::Timestamp till);

#endif /* H12CC03ED_5B11_4DE8_AE61_CF57E441DF86 */
