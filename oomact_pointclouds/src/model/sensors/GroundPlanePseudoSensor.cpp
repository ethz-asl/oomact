#include <aslam/calibration/model/sensors/GroundPlanePseudoSensor.h>

#include <cmath>

#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/VectorExpressionToGenericMatrixTraits.hpp>
#include <aslam/calibration/calibrator/CalibrationConfI.h>
#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/error-terms/MeasurementErrorTerm.h>
#include <aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h>
#include <boost/make_shared.hpp>
#include <glog/logging.h>

#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/clouds/CloudBatch.h>
#include <aslam/calibration/clouds/CloudsContainer.h>
#include <aslam/calibration/laser-scanner/laser3d.h>
#include <aslam/calibration/clouds/PointCloudsPlugin.h>

using aslam::backend::EuclideanExpression;
using aslam::backend::ScalarExpression;
namespace aslam {
namespace calibration {

struct GroundPlanePseudoSensorPoinCloudPolicy : public PointCloudPolicy, public NamedMinimal {
  GroundPlanePseudoSensorPoinCloudPolicy() : NamedMinimal("GroundPlanePseudoSensorPoinCloudPolicy") {}

  void prepareForNewData(const PointCloudsPlugin&, CloudBatches &, Timestamp, const PointCloudSensor &) const override {
    SM_THROW(std::runtime_error, "Not implemented");
  }
};

GroundPlanePseudoSensor::GroundPlanePseudoSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config)
  : Lidar3d(model, name, config),
    baseFrame_(getModel().getFrame(getMyConfig().getString("baseFrame"))),
    baseTrajectory_(*this, "poseTrajectory")
{
  cutOffDistance_ = getMyConfig().getDouble("cutOffDistance");
  hightSigma_ = getMyConfig().getDouble("hightSigma", 1e-9);
  attitudeSigma_ = getMyConfig().getDouble("attitudeSigma", 1e-9);
  gridDelta = getMyConfig().getDouble("gridDelta", 0.1);
  SM_ASSERT_GT(std::runtime_error, gridDelta, 0.05, "Grid delta needs to be bigger than 0");
  gridNx = getMyConfig().getInt("gridNx", 500);
  SM_ASSERT_GT(std::runtime_error, gridNx, 0, "Grid gridNx must be bigger than 0");
  gridNy = getMyConfig().getInt("gridNy", 500);
  SM_ASSERT_GT(std::runtime_error, gridNy, 0, "Grid gridNy must be bigger than 0");
}

size_t GroundPlanePseudoSensor::findPointsInFieldOfView(const DP& piontCloud, const sm::kinematics::Transformation& T_sensor_pointCloud, std::vector<bool>&goodPoints) const {
  using namespace sm::kinematics;

  const size_t inputSize = piontCloud.features.cols();

  goodPoints.resize(inputSize, false);

  LOG(INFO)<< "Finding points in GroundPlanePseudoSensor field of view in " << inputSize << " points.";
  size_t countGood = 0;
  for(size_t index = 0; index < inputSize; index ++){ //TODO B transform as block
    const auto vPoint = (T_sensor_pointCloud * piontCloud.features.col(index).head<3>().cast<double>().eval()).eval();
    if(std::abs(vPoint[2]) <= cutOffDistance_){
      countGood ++;
      goodPoints[index] = true;
    }
  }
  LOG(INFO) << "Found good cloud points " << countGood;
  return countGood;
}

GroundPlanePseudoSensor::~GroundPlanePseudoSensor() {
}

void GroundPlanePseudoSensor::preProcessNewWindow(CalibratorI& calib) {
  auto& interval = calib.getCurrentEffectiveBatchInterval();

  const size_t nX = gridNx, nY = gridNy;
  CloudBatch::PointCloud points(3, nX * nY);
  const double d = gridDelta;
  const double oX = -(nX * d / 2), oY = -(nY * d / 2);

  for(size_t x = 0; x < nX; x++){
    for(size_t y = 0; y < nY; y++){
      points.col( x * nY + y) << d * x + oX, d * y + oY, 0.0;
    }
  }

  CloudBatch::TimestampsInputVector timestamps(points.cols(), 1);
  Timestamp timeStamp(interval.start);
  timestamps.setConstant(timeStamp);
  LOG(INFO)<< "Adding ground plain point clout at " << calib.secsSinceStart(timeStamp) << ".";
  auto& pcp = calib.getPlugin<PointCloudsPlugin>();
  CloudBatches& clouds = getClouds(calib.getCurrentStorage());
  clouds.createNewCloud(pcp, *this).getMeasurements<EuclideanCloudMeasurements>().addData(timestamps, points, points.cols());
  clouds.finishCurrentCloud(pcp);
}

void GroundPlanePseudoSensor::addMeasurementErrorTerms(CalibratorI& calib, const CalibrationConfI&/*cc*/, ErrorTermReceiver& problem, bool observeOnly) const {
  if(!baseTrajectory_.isResolved() || !baseTrajectory_.get().isUsed()){
    LOG(INFO)<< "Not creating GroundPlain constraint error terms because it is upper body trajectory only!";
    return;
  }

  Timestamp
    minTime = calib.getCurrentEffectiveBatchInterval().start,
    maxTime = calib.getCurrentEffectiveBatchInterval().end;

  const double elapsedTime = maxTime - minTime;
  const int numSegments = std::ceil(baseTrajectory_.get().getKnotsPerSecond() * 2 * elapsedTime);

  ErrorTermStatisticsWithProblemAndPredictor hightStatWPAP(calib, "GroundPlaneHightConstraint", problem, observeOnly);
  ErrorTermStatisticsWithProblemAndPredictor attitudeStatWPAP(calib, "GroundPlaneAttitudeConstraint", problem, observeOnly);

  Eigen::Vector3d zVector = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d normalVector = zVector;
  auto normalExp = convertToGME(aslam::backend::EuclideanExpression(normalVector));
  auto zExp = aslam::backend::EuclideanExpression(zVector);

  ErrorTermGroupReference hightEtGroup(hightStatWPAP.getName());
  ErrorTermGroupReference attitudeEtGroup(attitudeStatWPAP.getName());

  for (int i = 0; i < numSegments + 1 ; i++) {
    Timestamp timestamp = minTime + Timestamp((double)i * elapsedTime / numSegments);
    auto fromLowerBodyToGroundPlaneT = getTransformationExpressionTo(calib.getModelAt(timestamp, 0, {true}), baseFrame_).inverse();
    hightStatWPAP.add(timestamp, boost::make_shared<MeasurementErrorTerm<1, ScalarExpression>>(convertToGME(fromLowerBodyToGroundPlaneT.toEuclideanExpression()).transpose() * normalExp, 0, Eigen::MatrixXd::Identity(1, 1) * hightSigma_ * hightSigma_, hightEtGroup));
    attitudeStatWPAP.add(timestamp, boost::make_shared<MeasurementErrorTerm<3, EuclideanExpression>>(fromLowerBodyToGroundPlaneT.toRotationExpression() * zExp, normalVector, Eigen::MatrixXd::Identity(3, 3) * attitudeSigma_ * attitudeSigma_, attitudeEtGroup));
  }
  hightStatWPAP.printInto(LOG(INFO));
  attitudeStatWPAP.printInto(LOG(INFO));
}

void GroundPlanePseudoSensor::writeConfig(std::ostream& out) const {
  Lidar3d::writeConfig(out);
  out << ", cutOffDistance=" << cutOffDistance_;
  out << ", hightSigma=" << hightSigma_;
  out << ", attitudeSigma=" << attitudeSigma_;
  out << ", gridDelta=" << gridDelta;
  out << ", gridNx=" << gridNx;
  out << ", gridNy=" << gridNy;
}

std::shared_ptr<const PointCloudPolicy> GroundPlanePseudoSensor::getDefaultPointCloudPolicy(const PointCloudsPlugin&) const {
  return std::make_shared<GroundPlanePseudoSensorPoinCloudPolicy>();
}

} /* namespace calibration */
} /* namespace aslam */

