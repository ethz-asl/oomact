#include <aslam/calibration/model/sensors/Velodyne.hpp>

#include <string>
#include <limits>
#include <cstddef>

#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/calibration/calibrator/CalibratorI.hpp>
#include <aslam/calibration/clouds/CloudsContainer.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/tools/tools.h>
#include <base/BinaryBufferReader.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string.hpp>
#include <ethz/velodyne/Velodyne32Calibration.h>
#include <glog/logging.h>
#include <sensor/Converter.h>
#include <sm/PropertyTreeImplementation.hpp>

#include "aslam/calibration/CalibrationConfI.h"
#include "aslam/calibration/clouds/CloudBatch.h"
#include "aslam/calibration/laser-scanner/laser3d.h"
#include "aslam/calibration/clouds/PointCloudsPlugin.h"

using aslam::backend::ScalarExpression;
using boost::algorithm::starts_with;
using ethz::velodyne::Velodyne32Calibration;
using namespace sm::value_store;

//TODO Deploy move libvelodyne dependent stuff to a velodyne_calibration_package to get rid of this dependency

namespace aslam {
namespace calibration {
namespace {

  class CalibrationValueStore : public ValueStore {
   public:

    struct CorreValueHanleImpl : ValueHandleImpl<double> {
      virtual bool isUpdateable() const { return true; }
      virtual void update(double newValue) override { calibration->setDistCorr(index, Converter::mMeterConversion * newValue);}
      virtual double get() const override { return calibration->getDistCorr(index) / Converter::mMeterConversion; }
      virtual ~CorreValueHanleImpl(){}

      CorreValueHanleImpl(std::shared_ptr<Velodyne32Calibration> calibration, int index) : calibration(calibration), index(index) { }
     private:
      std::shared_ptr<Velodyne32Calibration> calibration;
      int index;
    };
    struct VertCorrValueHanleImpl : ValueHandleImpl<double> {
      virtual bool isUpdateable() const override { return true; }
      virtual void update(double newValue) override { calibration->setVertCorr(index, newValue);}
      virtual double get() const override { return calibration->getVertCorr(index); }
      virtual ~VertCorrValueHanleImpl(){}

      VertCorrValueHanleImpl(std::shared_ptr<Velodyne32Calibration> calibration, int index) : calibration(calibration), index(index) { }
     private:
      std::shared_ptr<Velodyne32Calibration> calibration;
      int index;
    };

    CalibrationValueStore(std::string prefix, std::shared_ptr<Velodyne32Calibration> calibration) : prefix(prefix), calibration(calibration) {
      CHECK(calibration);
    }

    virtual ValueHandle<bool> getBool(const std::string& fullPath, boost::optional<bool> def = boost::optional<bool>()) const{
      if(def) {
        return def.get();
      }
      throw sm::ConstPropertyTree::InvalidKeyException("Invalid key " + fullPath);
    }
    virtual ValueHandle<int> getInt(const std::string& fullPath, boost::optional<int> def = boost::optional<int>()) const override {
      if(def) {
        return def.get();
      }
      throw sm::ConstPropertyTree::InvalidKeyException("Invalid key " + fullPath);
    }
    virtual ValueHandle<double> getDouble(const std::string& fullPath, boost::optional<double> def = boost::optional<double>()) const override {
      const int i = getIndex(fullPath);
      if (isPossibleIndex(i)) {
        switch(prefix.back()) {
          case 'D':
            return ValueHandle<double>(new CorreValueHanleImpl{calibration, i });
          case 'P':
            return ValueHandle<double>(new VertCorrValueHanleImpl{calibration, i });
          default:
            LOG(FATAL) << "Unknown prefix " << prefix;
        }
      }
      if(def) {
        return def.get();
      }
      throw sm::ConstPropertyTree::InvalidKeyException("Invalid key " + fullPath);
    }
    virtual ValueHandle<std::string> getString(const std::string& fullPath, boost::optional<std::string> def = boost::optional<std::string>()) const override {
      if(def) {
        return def.get();
      }
      throw sm::ConstPropertyTree::InvalidKeyException("Invalid key " + fullPath);
    }

    int getIndex(const std::string& fullPath) const {
      if(boost::starts_with(fullPath, prefix)){
        auto postfix = fullPath.substr(prefix.size());
        if(postfix.back() == '/') postfix.resize(postfix.size() - 1);
        int i = atol(postfix.c_str());
        return i;
      } else {
        return -1;
      }
    }

    virtual bool hasKey(const std::string& fullPath) const override {
      return isPossibleIndex(getIndex(fullPath));
    }

    virtual KeyValueStorePair getChild(const std::string &) const override {
      throw(std::runtime_error("getChild not implemented in CalibrationValueStore"));
    }

    virtual bool isChildSupported() const override { return false; }

    virtual std::vector<KeyValueStorePair> getChildren() const override {
      throw(std::runtime_error("getChildren not implemented in CalibrationValueStore"));
    }
   private:
    std::string prefix;
    std::shared_ptr<Velodyne32Calibration> calibration;

    bool isPossibleIndex(int i) const {
      return i >= 0 && i < 32;
    }
};
}


Velodyne::Velodyne(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
    Lidar3d(model, name, config),
    doIntrinsicCalibration(getMyConfig().getBool("doIntrinsicCalibration", false)),
    doBeamAngleCalibration(doIntrinsicCalibration && getMyConfig().getBool("doBeamAngleCalibration", true))
{
  std::string calibPath = model.resolveConfigPath(getMyConfig().getString("intrinsicCalibrationFile"));
  LOG(INFO) << "Loading Velodyne calibration from " << calibPath << ".";
  calibration = ethz::velodyne::loadVelodyne32Calibration(calibPath);

  {
    std::set<double> vertAngles;
    CHECK_GE(calibration->getNumLasers(), 1);
    for(size_t i = 0; i < calibration->getNumLasers(); ++i){
      vertAngles.insert(sm::kinematics::rad2deg(calibration->getVertCorr(i)));
    }
    filterMaxVerticalAngleDeg = getMyConfig().getDouble("filterMaxVerticalAngleDeg", *vertAngles.rbegin() + 0.5);
    filterMinVerticalAngleDeg = getMyConfig().getDouble("filterMinVerticalAngleDeg", *vertAngles.begin() - 0.5);
  }

  std::string velBeamPrefix = getName() + "_BD";
  std::string velBeamVerticalAnglePrefix = getName() + "_BP";

  sm::value_store::ValueStoreRef vs(std::make_shared<CalibrationValueStore>(velBeamPrefix, calibration));
  sm::value_store::ValueStoreRef vsVA(std::make_shared<CalibrationValueStore>(velBeamVerticalAnglePrefix, calibration));

  if(doIntrinsicCalibration){
    for(size_t i = 0; i < 32; i++) { //TODO D support other beam numbers
      std::string name = velBeamPrefix + std::to_string(i);
      beamDistanceOffsets.emplace_back(new ScalarCv(name, vs.getChild(name)));
      beamDistanceOffsetExpressions.emplace_back(beamDistanceOffsets.back()->toExpression());
      if(doBeamAngleCalibration){
        std::string name = velBeamVerticalAnglePrefix + std::to_string(i);
        beamVerticalAngle.emplace_back(new ScalarCv(name, vsVA.getChild(name)));
        beamVerticalAngleExpressions.emplace_back(beamVerticalAngle.back()->toExpression());
      }
    }
  }
}

Velodyne::~Velodyne() {
}

size_t Velodyne::findPointsInFieldOfView(const DP& piontCloud, const sm::kinematics::Transformation& T_sensor_pointCloud, std::vector<bool>&goodPoints) const {
  using namespace sm::kinematics;

  const double maxAngle = deg2rad(filterMaxVerticalAngleDeg);
  const double minAngle = deg2rad(filterMinVerticalAngleDeg);
  //TODO A remove far away points as well

  size_t inputSize = piontCloud.features.cols();

  goodPoints.resize(inputSize, false);

  LOG(INFO)<< "Finding points in Velodyne field of view in " << inputSize << " points.";
  size_t countGood = 0;
  for(size_t index = 0; index < inputSize; index ++){ //TODO B transform as block
    const auto vPoint = T_sensor_pointCloud * piontCloud.features.col(index).head<3>().cast<double>().eval();
    const double vPointNorm = vPoint.norm();
    if(vPointNorm < minimalDistance) continue;

    const double vPointTheta = asin(double(vPoint(2)/vPointNorm));
    if(vPointTheta >= minAngle && vPointTheta <= maxAngle){
      countGood ++;
      goodPoints[index] = true;
    }
  }
  LOG(INFO) << "Found good cloud points " << countGood;
  return countGood;
}

bool Velodyne::isProvidingCorrection() const {
  return doIntrinsicCalibration;
}

std::map<float, unsigned char> computeAngle2Index(const Velodyne32Calibration& calibration){
  std::map<float, unsigned char> ret;
  for(size_t i = 0; i < 32u; i ++){
    ret.emplace(calibration.getVertCorr(i), i);
  }
  return ret;
}

int getLaserIndexFromPoint(const Velodyne32Calibration& calibration, const Eigen::Vector3d& point){
  static std::map<float, unsigned char> angles2index = computeAngle2Index(calibration);

  const double xydist = point.head<2>().norm();
  const double z = point(2);
  const float angle = std::atan2(z, xydist);
  auto lb = angles2index.lower_bound(angle);
  if(lb == angles2index.end() || lb == angles2index.begin()) return angles2index.begin()->second;
  auto ub = lb;
  lb --;

  unsigned char beam = (fabs(lb->first - angle) < fabs(ub->first - angle)) ? lb->second : ub->second;
  return beam;
}

void Velodyne::registerWithModel() {
  Lidar3d::registerWithModel();
  if(isUsed()){
    if(doIntrinsicCalibration){
      for(auto cv : beamDistanceOffsets){
        getModel().addCalibrationVariables({cv});
      }
    }
    if(doBeamAngleCalibration){
      for(auto cv : beamVerticalAngle){
        getModel().addCalibrationVariables({cv});
      }
    }
  }
}

void Velodyne::setActive(bool spatial, bool temporal) {
  Lidar3d::setActive(spatial, temporal);
  if(doIntrinsicCalibration){
    for(auto cv : beamDistanceOffsets){
      cv->setActive(spatial);
    }
  }
  if(doBeamAngleCalibration){
    for(auto cv : beamVerticalAngle){
      cv->setActive(spatial);
    }
  }
}

float Velodyne::getBeamDistantanceCorrection(int laserIndex) const {
  assert(laserIndex < 32);
  if (doIntrinsicCalibration) {
    return beamDistanceOffsets[laserIndex]->getValue();
  } else {
    return calibration->getDistCorr(laserIndex);
  }
}
ScalarExpression Velodyne::getBeamDistantanceCorrectionExpression(int laserIndex) const {
  assert(doIntrinsicCalibration);
  assert(laserIndex < 32);
  return beamDistanceOffsetExpressions[laserIndex];
}

float Velodyne::getBeamVerticalAngleCorrection(int laserIndex) const {
  assert(laserIndex < 32);
  if (doBeamAngleCalibration) {
    return beamVerticalAngle[laserIndex]->getValue();
  } else {
    return calibration->getVertCorr(laserIndex);
  }
}

ScalarExpression Velodyne::getBeamVerticalAngleCorrectionExpression(int laserIndex) const {
  assert(doBeamAngleCalibration);
  assert(laserIndex < 32);
  return beamVerticalAngleExpressions[laserIndex];
}

void Velodyne::writeConfig(std::ostream& out) const {
  Lidar3d::writeConfig(out);
  MODULE_WRITE_PARAM(doIntrinsicCalibration);
  MODULE_WRITE_PARAM(doBeamAngleCalibration);
  MODULE_WRITE_PARAM(filterMinVerticalAngleDeg);
  MODULE_WRITE_PARAM(filterMaxVerticalAngleDeg);
}

void Velodyne::writeSnapshot(const CalibrationConfI& cc, bool /*stateWasUpdatedSinceLastTime*/) const {
  if(doIntrinsicCalibration){
    //TODO C cleanup this hack to write the intrinsic calibration
    const std::string calibFile = cc.getOutputFolder() + "Calibration";
    LOG(INFO) << "Updating beam variables and writing to " << calibFile;
    for(auto& b : beamDistanceOffsets){
      b->updateStore();
    }
    for(auto& b : beamVerticalAngle){
      b->updateStore();
    }
    std::ofstream stream;
    openStream(stream, calibFile);
    if(stream.is_open()){
      stream << *calibration;
    }
  }
}


namespace {

class VelodynePacketDataPacket : public DataPacket {
 public:
  void readFromVelodynePacket(const char * data, size_t size) {
    BinaryBufferReader r(data, size);
    readRawPacket(r);
  }

  VelodynePacketDataPacket(const std::string& data){
    readFromVelodynePacket(&data[0], data.length());
  }
};

void convertPacketInto(const VelodynePacketDataPacket& dataPacket, CloudBatch::PointCloud& velodyneScan, double minimalDistance, double maximalDistance, const Velodyne32Calibration& cal){
  VdynePointCloud pointCloud;
  Converter::toPointCloud(dataPacket, (Calibration&)cal, pointCloud, minimalDistance, maximalDistance);

  velodyneScan.resize(3, pointCloud.getSize());

  size_t i = 0;
  for (auto it = pointCloud.getPointBegin(), end = pointCloud.getPointEnd(); it != end; it++) {
    velodyneScan.col(i++) = aslam::calibration::PmVector3(it->mX, it->mY, it->mZ);
  }
}

struct RawMeasurement {
  float distance;
  float yaw;
  unsigned char beamIndex;
};
struct RawMeasurements {
  std::vector<RawMeasurement> rawMeasurements;
  void append(const RawMeasurements& source){
    rawMeasurements.reserve(rawMeasurements.size() + source.rawMeasurements.size());
    std::copy(source.rawMeasurements.begin(), source.rawMeasurements.end(), std::back_inserter(rawMeasurements));
  }
};

void toRawMeasurements(const VelodynePacketDataPacket& dataPacket, RawMeasurements& scanCloud, float minDistance, float maxDistance) {
  scanCloud.rawMeasurements.reserve(scanCloud.rawMeasurements.size() + dataPacket.mDataChunkNbr);
  for (size_t i = 0; i < dataPacket.mDataChunkNbr; ++i) {
    size_t idxOffs = 0;
    const DataPacket::DataChunk& data = dataPacket.getDataChunk(i);
    if (data.mHeaderInfo == dataPacket.mLowerBank)
      idxOffs = data.mLasersPerPacket;
    const float rotation = Calibration::deg2rad(static_cast<float>(data.mRotationalInfo) / static_cast<float>(dataPacket.mRotationResolution));
    for (size_t j = 0; j < data.mLasersPerPacket; ++j) {
      size_t laserIdx = idxOffs + j;
      const float distance = (static_cast<float>(data.mLaserData[j].mDistance) / static_cast<float>(dataPacket.mDistanceResolution)) / static_cast<float>(Converter::mMeterConversion);
      if ((distance < minDistance) || (distance > maxDistance))
        continue;
      scanCloud.rawMeasurements.emplace_back(RawMeasurement{distance, rotation, static_cast<unsigned char>(laserIdx)});
    }
  }
}

PmVector3 toEuclideanPoint(const RawMeasurement& m, const Velodyne& velodyne, bool applyCorrection, bool ignoreDistance = false) {
  const int laserIdx = m.beamIndex;
  const float yaw = m.yaw; //TODO AAA add correction
  const float sinRot = sin(yaw);
  const float cosRot = cos(yaw);

  float sinVertCorr;
  float cosVertCorr;
  const auto& calibration = velodyne.getCalibration();
  //TODO X support more Velodyne corrections
  if(applyCorrection){ //TODO AAA this should be doAngle...
    const double vertCorr = velodyne.getBeamVerticalAngleCorrection(laserIdx);
    sinVertCorr = sin(vertCorr);
    cosVertCorr = cos(vertCorr);
  } else {
//  const float horizOffsCorr = calibration.getHorizOffsCorr(laserIdx) / static_cast<float>(Converter::mMeterConversion);
//  const float vertOffsCorr = calibration.getVertOffsCorr(laserIdx) / static_cast<float>(Converter::mMeterConversion);
//  const float xyDist = distance * calibration.getCosVertCorr(laserIdx) - vertOffsCorr * calibration.getSinVertCorr(laserIdx);
    cosVertCorr = calibration.getCosVertCorr(laserIdx);
    sinVertCorr = calibration.getSinVertCorr(laserIdx);
  }

  PmVector3 p;
  p(0) = cosVertCorr * sinRot;
  p(1) = cosVertCorr * cosRot;
  p(2) = sinVertCorr;
  if(ignoreDistance){
    return p;
  }

  float distance = m.distance;
  if(applyCorrection){
    distance += velodyne.getBeamDistantanceCorrection(laserIdx);
  }
  return p * distance;
}

aslam::backend::EuclideanExpression toEuclideanExpression(const RawMeasurement& m, const Velodyne& velodyne, bool doBeamAngleCalibration) {
  aslam::backend::EuclideanExpression p;
  if(doBeamAngleCalibration){
    const float yaw = m.yaw; //TODO AAA add correction
    const float sinRot = sin(yaw);
    const float cosRot = cos(yaw);
    aslam::backend::ScalarExpression bVAC = velodyne.getBeamVerticalAngleCorrectionExpression(m.beamIndex);
    aslam::backend::ScalarExpression sinVertCorr = sin(bVAC);
    aslam::backend::ScalarExpression cosVertCorr = cos(bVAC);
    p =   aslam::backend::EuclideanExpression(Eigen::Vector3d::UnitX() * sinRot) * cosVertCorr
        + aslam::backend::EuclideanExpression(Eigen::Vector3d::UnitY() * cosRot) * cosVertCorr
        + aslam::backend::EuclideanExpression(Eigen::Vector3d::UnitZ()) *sinVertCorr;
  } else {
    p = aslam::backend::EuclideanExpression(toEuclideanPoint(m, velodyne, false, true).cast<double>());
  }
  aslam::backend::ScalarExpression beamDistantanceCorrectionExpression = velodyne.getBeamDistantanceCorrectionExpression(m.beamIndex);

  //TODO O support both: offset and measurement expressions?
  return p * (beamDistantanceCorrectionExpression + m.distance);
}

class VelodyneCloudMeasurements : public EuclideanCloudMeasurements, private RawMeasurements {
 public:
  using EuclideanCloudMeasurements::EuclideanCloudMeasurements;

  int addData(const Velodyne& velodyne, const Timestamp t, const RawMeasurements& scan) {
    assert(maximalDuration > Timestamp::Zero());

    //TODO C deduplicate with EuclideanCloudMeasurements::addData
    if(measurements.empty()){
      minTimestamp = maxTimestamp = t;
    }
    else{
      minTimestamp = std::min(minTimestamp, t);
      maxTimestamp = std::max(maxTimestamp, t);
    }

    const size_t size = scan.rawMeasurements.size();
    if(t > Timestamp(getMinTimestamp() + getMaximalDuration())) {
      VLOG(1) << "Skipping " << size << " points.";
      return 0;
    }

    measurements.reserve(measurements.size() + size);

    for(size_t pointIndex = 0; pointIndex < size; ++pointIndex){
      PointMeasurement pm{t, toEuclideanPoint(scan.rawMeasurements[pointIndex], velodyne, true)};
      measurements.emplace_back(pm);
    }
    append(scan);
    CHECK_EQ(rawMeasurements.size(), measurements.size());
    VLOG(1) << "Appended " << size << " points.";
    return size;
  }

  void updatePoints(const Velodyne& velodyne){
    int i = 0;
    for(auto& rm : rawMeasurements){
      measurements[i++].p = toEuclideanPoint(rm, velodyne, true);
    }
  }

  const RawMeasurement& getRawMeasurement(size_t index) const {
    return rawMeasurements.at(index);
  }
};

}

CloudBatch::TimestampsInputVector getTimestamps(const Timestamp& t, size_t scanSize){
  CloudBatch::TimestampsInputVector timestamps(scanSize, 1);
  timestamps.setConstant(t.getNumerator()); //TODO A use individual timestamps!
  return timestamps;
}

void Velodyne::addNewPackage(const Timestamp& t, const std::string& data, ModuleStorage& storage) const {
  if(!doIntrinsicCalibration){
    CloudBatch::PointCloud scan;
    convertPacketInto(data, scan, minimalDistance, maximalDistance, getCalibration());
    getClouds(storage).addCloudScan(*this, scan, getTimestamps(t, scan.cols()));
  } else {
    RawMeasurements scan;
    toRawMeasurements(data, scan, minimalDistance, maximalDistance);
    const size_t size = scan.rawMeasurements.size();
    getClouds(storage).applyCloudDataFunctor(
        [&, t](CloudBatch& toCloud){
          return toCloud.getMeasurements<VelodyneCloudMeasurements>().addData(*this, t, scan);
        },
        getTimestamps(t, size)
      );
  }
}

void Velodyne::addInputTo(Timestamp t, const VelodynePackageRef& input, ModuleStorage& storage) const {
  addNewPackage(t, std::string(input.data, input.length), storage);
}

void Velodyne::addInputTo(Timestamp t, const VelodynePointsFunctor& input, ModuleStorage& storage) const {
  CHECK(!doIntrinsicCalibration) << "Pointwise input is not supported in combination with intrinsic calibration.";
  CloudBatch::PointCloud scan(3, input.length);
  input.fillCloud(scan);
  getClouds(storage).addCloudScan(*this, scan, getTimestamps(t, scan.cols()));
}

void Velodyne::estimatesUpdated(CalibratorI& calib) const {
  if(doIntrinsicCalibration){
    LOG(INFO) << "Updating point clouds after an update of the intrinsics.";
    for(CloudBatch& c : getClouds(calib.getCurrentStorage())){
      c.getMeasurements<VelodyneCloudMeasurements>().updatePoints(*this);
    }
  }
}

aslam::backend::EuclideanExpression Velodyne::calcMeasurementExpressionByUnfilteredIndex(const CloudBatch& cloud, size_t pointIndex) const {
  CHECK(doIntrinsicCalibration);
  auto& m = cloud.getMeasurements<VelodyneCloudMeasurements>().getRawMeasurement(pointIndex);
  return toEuclideanExpression(m, *this, doBeamAngleCalibration);
}

std::unique_ptr<CloudMeasurements> Velodyne::createCloudMeasurements(CloudBatch& /*cloudBatch*/) const {
  return std::unique_ptr<CloudMeasurements>(doIntrinsicCalibration ? new VelodyneCloudMeasurements(nanPolicy_) : new EuclideanCloudMeasurements(nanPolicy_));
}

} /* namespace calibration */
} /* namespace aslam */
