#ifndef HAA39AA34_DD3B_4A27_95E3_90FD35D78BC4
#define HAA39AA34_DD3B_4A27_95E3_90FD35D78BC4

namespace aslam {
namespace calibration {
class CalibratorI;

class CalibratorRef {
 public:
  CalibratorRef(CalibratorI& calib) : calib_(calib) {}

  CalibratorI & getCalibrator() {
    return calib_;
  }
  const CalibratorI & getCalibrator() const {
    return calib_;
  }
 private:
  CalibratorI& calib_;
};

}
}

#endif /* HAA39AA34_DD3B_4A27_95E3_90FD35D78BC4 */
