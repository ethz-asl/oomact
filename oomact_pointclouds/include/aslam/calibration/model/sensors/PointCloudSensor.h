#ifndef HF14E7B7B_2C28_405E_A1E7_612F3B46EBDE
#define HF14E7B7B_2C28_405E_A1E7_612F3B46EBDE

#include <iosfwd>

#include <aslam/calibration/model/Sensor.hpp>
#include <aslam/calibration/tools/Printable.h>

#include <aslam/calibration/clouds/NanPolicy.h>
#include <aslam/calibration/clouds/PmFilter.h>
#include <aslam/calibration/clouds/PointCloudsPlugin.h>

namespace aslam {
namespace calibration {
class CloudBatch;
class CloudBatches;
class CalibratorI;
class PointCloudSensor;
class CloudMeasurements;
class EuclideanCloudMeasurements;

std::ostream& operator << (std::ostream&, const CloudBatch& c);

class PointCloudPolicy : public virtual Named {
 public:
  virtual ~PointCloudPolicy() = default;
  virtual void prepareForEstimation(const PointCloudsPlugin& pcp, CloudBatches& clouds) const;
  virtual void prepareForNewData(const PointCloudsPlugin& pcp, CloudBatches& clouds, Timestamp t, const PointCloudSensor& sensor) const = 0;
};

struct SimplePointCloudPolicy : public PointCloudPolicy, public NamedMinimal {
  SimplePointCloudPolicy(std::string name, ValueStoreRef config);
  bool isReady(CloudBatch& b) const;
  bool wouldStillFit(CloudBatch& b, Timestamp t) const;
  void prepareForEstimation(const PointCloudsPlugin& pcp, CloudBatches& clouds) const override;
  void prepareForNewData(const PointCloudsPlugin& pcp, CloudBatches& clouds, Timestamp t, const PointCloudSensor& sensor) const override;
  void print(std::ostream& o) const override;
 private:
  Duration maximalDuration;
  Duration minimalDuration;
  Duration minimalGap;
  Duration startPadding;
};

class PointCloudSensor : public Sensor {
 public:
  PointCloudSensor(Model& model, const std::string& name, sm::value_store::ValueStoreRef config);
  void filterPointCloud(CloudBatch& data);

  virtual size_t findPointsInFieldOfView(const DP& pointCloud, const sm::kinematics::Transformation& T_sensor_pointCloud, std::vector<bool>&goodPoints) const;

  virtual Eigen::Matrix3d covPoint(bool useSurfaceNormal, const Eigen::Vector3d& pInSensorFrame, const Eigen::Vector3d& nInSensorFrame) const;

  virtual const PointCloudPolicy& getPointCloudPolicy(const PointCloudsPlugin& pcp) const;

  virtual ~PointCloudSensor();

  void setPointCloudPolicy(const std::shared_ptr<const PointCloudPolicy>& pointCloudPolicy) {
    this->pointCloudPolicy = pointCloudPolicy;
  }

  Interval getCurrentMeasurementTimestampRange(const CalibratorI& calib) const override;

  virtual bool isProvidingCorrection() const;

  virtual aslam::backend::EuclideanExpression calcMeasurementExpressionByUnfilteredIndex(const CloudBatch& cloud, size_t pointFilteredIndex) const;
  aslam::backend::EuclideanExpression calcMeasurementExpressionByFilteredIndex(const CloudBatch& cloud, size_t pointIndex) const;

  typedef EuclideanCloudMeasurements CloudMeasurementsImpl;
  virtual std::unique_ptr<CloudMeasurements> createCloudMeasurements(CloudBatch& cloudBatch) const;

 protected:
  void writeConfig(std::ostream& out) const override;

 private:
  PmFilter filter;
  mutable std::shared_ptr<const PointCloudPolicy> pointCloudPolicy;

 protected:
  NanPolicy nanPolicy;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HF14E7B7B_2C28_405E_A1E7_612F3B46EBDE */
