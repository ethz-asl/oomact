#include <aslam/calibration/model/PoseTrajectory.h>

#include <boost/make_shared.hpp>
#include <glog/logging.h>

#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>
#include <bsplines/NsecTimePolicy.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/Transformation.hpp>

#include "aslam/calibration/algo/OdometryPath.h"
#include "aslam/calibration/calibrator/CalibratorI.hpp"
#include <aslam/calibration/DesignVariableReceiver.hpp>
#include "aslam/calibration/data/PoseMeasurement.h"
#include <aslam/calibration/error-terms/ErrorTermTangency.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/model/fragments/So3R3Trajectory.h>
#include <aslam/calibration/model/sensors/WheelOdometry.h>
#include <aslam/calibration/model/sensors/PoseSensorI.hpp>
#include <aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h>
#include <aslam/calibration/tools/MeasurementContainerTools.h>

using bsplines::NsecTimePolicy;
using sm::kinematics::Transformation;
using sm::timing::NsecTime;

namespace aslam {
namespace calibration {


Eigen::Vector4d negateQuatIfThatBringsItCloser(const Eigen::Vector4d& pquat, const
    Eigen::Vector4d& cquat) {
  if ((pquat + cquat).norm() < (pquat - cquat).norm()) {
    return -cquat;
  } else {
    return cquat;
  }
}

class BaseTrajectoryBatchState : public BatchState, public So3R3Trajectory {
 public:
  BaseTrajectoryBatchState(PoseTrajectory & baseTrajectory)
    : So3R3Trajectory(baseTrajectory)
  {}

  void writeToFile(const CalibratorI & calib, const std::string & pathPrefix) const override;
};


constexpr double TAN_CONSTRAINT_VARIANCE_DEFAULT = 1e-8;

PoseTrajectory::PoseTrajectory(Model& model, const std::string& name, sm::value_store::ValueStoreRef config) :
  Module(model, name, config),
  So3R3TrajectoryCarrier(getMyConfig().getChild("splines"), model.getFrame(getMyConfig().getString("frame"))),
  estimate(getMyConfig().getBool("estimate", true)),
  useTanConstraint(getMyConfig().getBool("tangentialConstraint/used", false)),
  tanConstraintVariance(getMyConfig().getDouble("tangentialConstraint/variance", TAN_CONSTRAINT_VARIANCE_DEFAULT)),
  initWithPoseMeasurements(getMyConfig().getBool("initWithPoseMeasurements", false)),
  poseSensor(*this, "McSensor", initWithPoseMeasurements),
  odometrySensor(*this, "OdomSensor"),
  assumeStatic(getMyConfig().getBool("assumeStatic", false)),
  referenceFrame_(model.getFrame(getMyConfig().getString("referenceFrame")))
{
  if(isUsed()){
    if(!estimate){
      LOG(WARNING) << "Not going to estimate the " << getName() << "!";
    } else {
      LOG(INFO) << "Going to estimate the " << getName() << ".";
    }
  }
}

void PoseTrajectory::writeConfig(std::ostream& out) const {
  MODULE_WRITE_PARAMETER(estimate);
  MODULE_WRITE_PARAMETER(assumeStatic);
  MODULE_WRITE_PARAMETER(initWithPoseMeasurements);
  MODULE_WRITE_PARAMETER(poseSensor);
  MODULE_WRITE_PARAMETER(odometrySensor);
  MODULE_WRITE_PARAMETER(referenceFrame_);
  So3R3TrajectoryCarrier::writeConfig(out);
  if(useTanConstraint){
    MODULE_WRITE_PARAMETER(tanConstraintVariance);
  }
}

bool initSplines(CalibratorI & calib, So3R3Trajectory & trajectory, const PoseSensorI& poseSensor, const Frame & referenceFrame) {
  auto & storage = calib.getCurrentStorage();
  if (!poseSensor.hasMeasurements(storage)) {
    LOG(WARNING) << "Pose sensor " << poseSensor.getSensor() << " has no measurements! Cannot initialize based on it!";
    return false;
  }

  const PoseMeasurements & measurements = poseSensor.getAllMeasurements(storage);

  const Interval & effectiveBatchInterval = calib.getCurrentEffectiveBatchInterval();
  SM_ASSERT_TRUE(Exception, effectiveBatchInterval, "effectiveBatchInterval must be set");

  const size_t numMeasurements = measurements.size();

  CHECK_EQ(poseSensor.getTargetFrame(), referenceFrame); //TODO Support trajectory initializing with more remote pose sensors

  LOG(INFO) << "Initializing " << getObjectName(trajectory.getCarrier()) << " with "<< numMeasurements << " poses from " << poseSensor.getSensor().getName();
  std::vector<NsecTime> timestamps;
  timestamps.reserve(numMeasurements);
  std::vector<Eigen::Vector3d> transPoses;
  transPoses.reserve(numMeasurements);
  std::vector<Eigen::Vector4d> rotPoses;
  rotPoses.reserve(numMeasurements);

  const auto & trajectoryFrame = trajectory.getCarrier().getFrame();
  const auto T_sens_traj = poseSensor.getSensor().getTransformationTo(calib, trajectoryFrame).inverse();

  const Timestamp currentDelay = poseSensor.getSensor().getDelay();

  for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
    Timestamp timestamp = it->first - currentDelay;
    if(effectiveBatchInterval.contains(timestamp)){
      sm::kinematics::Transformation T_measured(it->second.q, it->second.t);
      T_measured = T_measured * T_sens_traj;

      timestamps.push_back(timestamp.getNumerator());
      rotPoses.push_back(rotPoses.empty() ? T_measured.q() : negateQuatIfThatBringsItCloser(rotPoses.back(), T_measured.q()));
      transPoses.push_back(T_measured.t());
    }
  }

  trajectory.fitSplines(effectiveBatchInterval, numMeasurements, timestamps, transPoses, rotPoses);

  const auto startTimestamp = trajectory.getRotationSpline().getMinTime();
  const auto endTimestamp = trajectory.getRotationSpline().getMaxTime();
  static_cast<void>(startTimestamp); // prevent unused warning
  static_cast<void>(endTimestamp);
  assert(trajectory.getTranslationSpline().getMinTime() == startTimestamp);
  assert(trajectory.getTranslationSpline().getMaxTime() == endTimestamp);

  assert(effectiveBatchInterval.start == Timestamp(startTimestamp));
  assert(effectiveBatchInterval.end == Timestamp(endTimestamp));
  return true;
}


PoseMeasurement getFirstPoseMeasurement(CalibratorI & calib, Timestamp & startTime, const Sensor & sensor, bool respectDelayLowerBound, const Frame * transformToFramePtr) {
  if(respectDelayLowerBound && sensor.hasDelay()){
    if(startTime - sensor.getDelayUpperBound() < Timestamp(calib.getCurrentEffectiveBatchInterval().start)){
      startTime = Timestamp(calib.getCurrentEffectiveBatchInterval().start) + sensor.getDelayUpperBound(); //TODO C use Timestamp everywhere
    }
  }
  SM_ASSERT_TRUE(std::runtime_error, sensor.isA<PoseSensorI>(), "")

  auto & poseSensor = sensor.as<PoseSensorI>();
  auto & storage = calib.getCurrentStorage();
  CHECK(poseSensor.hasMeasurements(storage));

  auto ps = getMeasurementsSlice(poseSensor.getAllMeasurements(storage), startTime, (startTime + Timestamp(1.0)));
  CHECK(!ps.empty()) << "Did not get any pose measurement!"; //TODO B implement nicer solution

  startTime = ps[0].first;

  PoseMeasurement & pose = ps[0].second;
  if(transformToFramePtr){
    sm::kinematics::Transformation m_f(pose.q, pose.t);

    std::string poseToString(const sm::kinematics::Transformation & trafo);

    m_f = m_f * sm::kinematics::Transformation(sensor.getTransformationExpressionTo(calib.getModelAt(startTime, 0, {false, false}), *transformToFramePtr).toTransformationMatrix()).inverse();

    pose.q = m_f.q();
    pose.t = m_f.t();
  }
  return pose;
}


bool initSplines(CalibratorI & calib, So3R3Trajectory & trajectory, const WheelOdometry & wheelOdometry, const ModuleLink<PoseSensorI>& poseSensor) {
  if(!wheelOdometry.isUsed()){
    throw std::runtime_error("Attempt to initialize from unused WheelOdometry!");
  }
  using namespace aslam::backend;

  auto & wheelSpeedsMeasurements = wheelOdometry.getMeasurements();

  LOG(INFO) << "Initializing Spline with integration of "<< wheelSpeedsMeasurements.size() << " wheel speeds measurements ";
  const size_t numWheelSpeedsMeasurements = wheelSpeedsMeasurements.size();

  std::vector<Eigen::Vector4d> rotPoses;
  std::vector<Eigen::Vector3d> transPoses;

  const Interval & effectiveBatchInterval = calib.getCurrentEffectiveBatchInterval();
  CHECK(effectiveBatchInterval);

  const Timestamp startTimestamp = effectiveBatchInterval.start;
  const Timestamp endTimestamp = effectiveBatchInterval.end;

  std::vector<NsecTime> timestampsWheelSpeeds;
  timestampsWheelSpeeds.reserve(numWheelSpeedsMeasurements);

  // TODO: Here we can initialize also to previous value
  Eigen::Vector3d transPose = Eigen::Vector3d::Zero();
  Eigen::Vector4d rotPose = (Eigen::Vector4d() << 0.0,0.0,0.0,1.0).finished();

  transPoses.reserve(numWheelSpeedsMeasurements);
  rotPoses.reserve(numWheelSpeedsMeasurements);

  Timestamp prevTimestamp = startTimestamp;

  if(poseSensor.isResolved()) {
    Timestamp tmpStart(prevTimestamp);
    auto pose = getFirstPoseMeasurement(calib, tmpStart, poseSensor.get().getSensor(), false, &trajectory.getCarrier().getFrame()); //TODO B sync

    if(double(tmpStart - prevTimestamp) > 0.02){
      LOG(WARNING) << "Motion capture data is quite sparse or starts too late (searched for "<< calib.secsSinceStart(prevTimestamp) << " found " << calib.secsSinceStart(tmpStart) << "!";
    }
    transPose = pose.t;
    rotPose = pose.q;
  }

  std::string poseToString(const Eigen::Vector3d & trans, const Eigen::Vector4d & rot);

  LOG(INFO) << "Using start pose " << poseToString(transPose, rotPose);

  timestampsWheelSpeeds.push_back(prevTimestamp);
  transPoses.push_back(transPose);
  rotPoses.push_back(rotPose);

  auto prevRotPose = rotPose;
  auto prevTransPose = transPose;

  wheelOdometry.getDelayVariable().printValuesNiceInto(LOG(INFO) << "Using wheelDelay:\n");
  const Timestamp delayValue = wheelOdometry.getDelay();
  Timestamp timestamp;

  for (auto it = wheelSpeedsMeasurements.cbegin(), end = wheelSpeedsMeasurements.cend(); it != end; ++it) {
    timestamp = it->first - delayValue;
    if(timestamp <= startTimestamp){
      LOG(INFO) << "Throwing away wheelSpeedMeasurements (" << calib.secsSinceStart(timestamp) << ")";
      continue;
    }
    if(timestamp > endTimestamp){
      LOG(INFO) << "Throwing away wheelSpeedMeasurements starting from " << calib.secsSinceStart(timestamp);
      break;
    }

    const auto pairLocVel = odomKinemModel(it->second.left, it->second.right, wheelOdometry.getWheelRadiusL()->toScalar(), wheelOdometry.getWheelRadiusR()->toScalar(), wheelOdometry.getL()->toScalar());
    const auto v_r_mr_km1 = pairLocVel.first;
    const auto w_r_mr_km1 = pairLocVel.second;

    const auto q_m_r_km1 = prevRotPose;
    Transformation T_m_r_k = integrateMotionModel(prevTransPose, q_m_r_km1, v_r_mr_km1, w_r_mr_km1, timestamp - prevTimestamp);

    prevRotPose = T_m_r_k.q();
    prevTransPose = T_m_r_k.t();
    prevTimestamp = timestamp;

    VLOG(2) << "t: " << T_m_r_k.t().transpose() << " q: " << T_m_r_k.q().transpose();

    timestampsWheelSpeeds.push_back(timestamp.getNumerator());

    // Best quaternion correction
    auto q_m_r_k = negateQuatIfThatBringsItCloser(q_m_r_km1, T_m_r_k.q());
    transPoses.push_back(T_m_r_k.t());
    rotPoses.push_back(q_m_r_k);
  }
  if(timestamp < endTimestamp){
    timestampsWheelSpeeds.push_back(endTimestamp);
    transPoses.push_back(transPoses.back());
    rotPoses.push_back(rotPoses.back());
  }

  CHECK_EQ(timestampsWheelSpeeds.size(), transPoses.size());
  CHECK_EQ(rotPoses.size(), transPoses.size());
  CHECK(!transPoses.empty());

  trajectory.fitSplines(effectiveBatchInterval, numWheelSpeedsMeasurements, timestampsWheelSpeeds, transPoses, rotPoses);

  // TODO: Initialize bias splines

  // Bias splines initialization to zero, TODO: It might be better to initialize to previous batch bias
  //const double elapsedTime = (timestampsWheelSpeeds.back() - timestampsWheelSpeeds.front()) /
  //    (double)NsecTimePolicy::getOne();
  //const int measPerSec = std::round(numWheelSpeedsMeasurements / elapsedTime);
  //int numSegments;

  CHECK_EQ(Timestamp(trajectory.getRotationSpline().getMinTime()), startTimestamp);
  CHECK_EQ(Timestamp(trajectory.getRotationSpline().getMaxTime()), endTimestamp);
  CHECK_EQ(Timestamp(trajectory.getTranslationSpline().getMinTime()), startTimestamp);
  CHECK_EQ(Timestamp(trajectory.getTranslationSpline().getMaxTime()), endTimestamp);
  return true;
}

bool PoseTrajectory::initState(CalibratorI& calib) {
  state_ = std::make_shared<BaseTrajectoryBatchState>(*this);

  if(assumeStatic){
    LOG(INFO) << getName() << " : Initializing statically!";
    getCurrentTrajectory().initSplinesConstant(calib.getCurrentEffectiveBatchInterval(), 1);
    return true;
  } else {
    if(initWithPoseMeasurements){
      if(poseSensor.isResolved()){
        return initSplines(calib, getCurrentTrajectory(), poseSensor, getReferenceFrame());
      } else {
        throw std::runtime_error(getName() + ".initWithPoseMeasurements is true but " + poseSensor.toString() + " is not resolved!");
      }
    }
    CHECK(odometrySensor.isResolved()) << getName() << ".initWithPoseMeasurements is false but " << odometrySensor.toString() << " is not resolved!";
    return initSplines(calib, getCurrentTrajectory(), odometrySensor, poseSensor);
  }
}

void PoseTrajectory::addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem) {
  const bool stateActive = estimate && !assumeStatic && stateActivator.isActive(*this);
  if(!estimate){
    LOG(WARNING) << "Not going to estimate " << getName() << "!";
  }
  if(stateActive){
    LOG(INFO) << "Activating " << getName() << "'s splines.";
  }
  CHECK(state_);
  state_->addToProblem(stateActive, problem);
  batchStateReceiver.addBatchState(*this, state_);
}

void PoseTrajectory::addErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem) const {
  if(useTanConstraint && state_){
    LOG(INFO) << "Adding soft constraints error terms.";

    const bool observerOnly = !ec.getStateActivator().isActive(*this) && !ec.getCalibrationActivator().isActive(*this);

    auto & trajectory = getCurrentTrajectory();

    Timestamp
      minTime = calib.getCurrentEffectiveBatchInterval().start,
      maxTime = calib.getCurrentEffectiveBatchInterval().end;

    const double elapsedTime = maxTime - minTime;

    const int numSegments = std::ceil(getKnotsPerSecond() * 2 * elapsedTime);
    const double tangentialVariance = tanConstraintVariance;

    ErrorTermStatisticsWithProblemAndPredictor statWPAP(calib, "TangentialConstraint", problem, observerOnly);
    ErrorTermGroupReference etgr(statWPAP.getName());
    for (int i = 0; i < numSegments + 1 ; i++) {
      Timestamp timestamp = minTime + Timestamp((double)i * elapsedTime / numSegments);

      auto translationExpressionFactory = trajectory.getTranslationSpline().getExpressionFactoryAt<1>(timestamp);
      auto rotationExpressionFactory = trajectory.getRotationSpline().getExpressionFactoryAt<0>(timestamp);

      aslam::backend::EuclideanExpression v_r_mwl, v_r_mwr;
      aslam::backend::RotationExpression R_m_r;

      R_m_r = aslam::backend::Vector2RotationQuaternionExpressionAdapter::adapt(rotationExpressionFactory.getValueExpression());
      const auto v_m_mr = translationExpressionFactory.getValueExpression(1);
      const auto v_r_mr = R_m_r.inverse() * v_m_mr;

      // Is it missing a constraint on the direction of the velocity?? By construction quaternion spline is not constrained to be tangent to the pose
      // We should add that constraint. Possibly in Jerome case we could not see that since there was an error term on the veloctiy,
      // That was constraining the local direction of versor i, in robot (vehicle frame). Otherwise rotation is not exactly well constrained

      // TODO: B Add a constraint on the velocity to be parallel to the orientation i x v = 0
      auto tangency_constraint = v_r_mr.cross(aslam::backend::EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)));
      auto e_tan = boost::make_shared<ErrorTermTangency>(tangency_constraint, Eigen::Vector3d(tangentialVariance, tangentialVariance, tangentialVariance).asDiagonal(), etgr);
      statWPAP.add(timestamp, e_tan);
    }
    statWPAP.printInto(LOG(INFO));
  }
}

void BaseTrajectoryBatchState::writeToFile(const CalibratorI & calib, const std::string& pathPrefix) const {
  So3R3Trajectory::writeToFile(calib, pathPrefix);
}

PoseTrajectory::~PoseTrajectory() {
}

const So3R3Trajectory& PoseTrajectory::getCurrentTrajectory() const {
  CHECK(state_);
  return *state_;
}

So3R3Trajectory& PoseTrajectory::getCurrentTrajectory() {
  CHECK(state_);
  return *state_;
}

} /* namespace calibration */
} /* namespace aslam */
