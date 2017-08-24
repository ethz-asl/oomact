#include <aslam/calibration/model/FrameGraphModel.h>

#include <aslam/backend/KinematicChain.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>
#include <aslam/calibration/model/fragments/So3R3Trajectory.h>
#include <aslam/calibration/model/Model.h>
#include <glog/logging.h>

#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/tools/Tree.hpp>

namespace aslam {
namespace calibration {

class FrameGraph: public Tree<const Frame*, PoseTrajectory*> {
};

constexpr int getVariability(BoundedTimeExpression*){
  return 1;
}
constexpr int getVariability(Timestamp*){
  return 0;
}

template <typename T>
constexpr int addVariablitiy(int i){
  return i + getVariability(static_cast<T*>(nullptr));
}

using namespace aslam::backend;

template <typename A>
aslam::backend::CoordinateFrame computeTrajectoryFrame(boost::shared_ptr<const CoordinateFrame> parent, A expressionFactories, bool needGlobalPosition, int maximalDerivativeOrder){
  return CoordinateFrame(
        parent,
        //TODO O The adapter is a big waste of time there are more direct ways (UnitQuaternion expressions ..)
        Vector2RotationQuaternionExpressionAdapter::adapt(expressionFactories.rot.getValueExpression()),
        needGlobalPosition ? expressionFactories.trans.getValueExpression(0) : EuclideanExpression(),
        maximalDerivativeOrder >= 1 ? -EuclideanExpression(expressionFactories.rot.getAngularVelocityExpression()) : EuclideanExpression(),
        maximalDerivativeOrder >= 1 ? EuclideanExpression(expressionFactories.trans.getValueExpression(1)) : EuclideanExpression(),
        maximalDerivativeOrder >= 2 ? -EuclideanExpression(expressionFactories.rot.getAngularAccelerationExpression()) : EuclideanExpression(),
        maximalDerivativeOrder >= 2 ? EuclideanExpression(expressionFactories.trans.getValueExpression(2)) : EuclideanExpression()
      );
}

template <typename Time>
class FrameGraphModelAtTimeImpl: public ModelAtTimeImpl {
 public:
  FrameGraphModelAtTimeImpl(const FrameGraphModel & fgModel, Time timestamp, int maximalDerivativeOrder, const ModelSimplification& simplification) :
   maximalDerivativeOrder_(maximalDerivativeOrder),
   fgModel_(fgModel),
   simplification_(simplification),
   timestamp_(timestamp)
  {
  }

  template <int maximalDerivativeOrder>
  aslam::backend::CoordinateFrame getCoordinateFrame(const Frame & to, const Frame & from, size_t originalMaximalDerivativeOrder) const {
    boost::shared_ptr<aslam::backend::CoordinateFrame> f, fInv;
    fgModel_.frameGraph_->walkPath(&to , &from, [&](PoseTrajectory* trajPtr, bool inverse){
      CHECK(!inverse) << "to=" << to << ", from=" << from;
      CHECK_NOTNULL(trajPtr);
      f = boost::make_shared<CoordinateFrame>(computeTrajectoryFrame(f, trajPtr->getCurrentTrajectory().getExpressionFactoryPair<maximalDerivativeOrder>(timestamp_), simplification_.needGlobalPosition, originalMaximalDerivativeOrder));
    });
    return std::move(*f);
  }

  aslam::backend::CoordinateFrame getCoordinateFrame(const Frame & to, const Frame & from) const {
    if(from == to) return CoordinateFrame();
    switch(addVariablitiy<Time>(maximalDerivativeOrder_)){
      case 0:
      case 1:
      case 2: //TODO O support maximalDerivativeOrder_ below 2 in getCoordinateFrame properly (requires turning off accelerations in computeTrajectoryFrame for that case!
        return getCoordinateFrame<2>(to, from, maximalDerivativeOrder_);
      case 3:
        return getCoordinateFrame<3>(to, from, maximalDerivativeOrder_);
      case 4:
        return getCoordinateFrame<4>(to, from, maximalDerivativeOrder_);
      default:
        LOG(FATAL) << "Unsupported maximal derivative order " << maximalDerivativeOrder_;
        throw 0; // dummy
    }
  }

  aslam::backend::TransformationExpression getTransformationToFrom(const Frame & to, const Frame & from) const override {
    const Frame & closestCommonAncestor = *fgModel_.frameGraph_->getClosestCommonAncestor(&to, &from);
    auto toCFrame = getCoordinateFrame(to, closestCommonAncestor);
    auto fromCFrame = getCoordinateFrame(from, closestCommonAncestor);
    return aslam::backend::TransformationExpression(toCFrame.getR_G_L(), toCFrame.getPG()).inverse() * aslam::backend::TransformationExpression(fromCFrame.getR_G_L(), fromCFrame.getPG());
  }

  aslam::backend::EuclideanExpression getAcceleration(const Frame & to, const Frame & from) const override {
    return getCoordinateFrame(to, from).getAG();
  }

  aslam::backend::EuclideanExpression getVelocity(const Frame & to, const Frame & from) const override {
    return getCoordinateFrame(to, from).getVG();
  }

  aslam::backend::EuclideanExpression getAngularVelocity(const Frame & to, const Frame & from) const override {
    return getCoordinateFrame(to, from).getOmegaG();
  }
  aslam::backend::EuclideanExpression getAngularAcceleration(const Frame & to, const Frame & from) const override {
    return getCoordinateFrame(to, from).getAlphaG();
  }
 private:
  int maximalDerivativeOrder_;
  const FrameGraphModel & fgModel_;
  const ModelSimplification simplification_;
  Time timestamp_;
};

FrameGraphModel::FrameGraphModel(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver, const std::vector<const Frame*> frames) :
  Model(config, configPathResolver, frames),
  frameGraph_(new FrameGraph())
{
}

FrameGraphModel::~FrameGraphModel(){
}

ModelAtTime FrameGraphModel::getAtTime(Timestamp timestamp, int maximalDerivativeOrder, const ModelSimplification& simplification) const {
  return ModelAtTime(std::unique_ptr<ModelAtTimeImpl>(new FrameGraphModelAtTimeImpl<Timestamp>(*this, timestamp, maximalDerivativeOrder, simplification)));
}

ModelAtTime FrameGraphModel::getAtTime(const BoundedTimeExpression& boundedTimeExpresion, int maximalDerivativeOrder, const ModelSimplification& simplification) const {
  return ModelAtTime(std::unique_ptr<ModelAtTimeImpl>(new FrameGraphModelAtTimeImpl<BoundedTimeExpression>(*this, boundedTimeExpresion, maximalDerivativeOrder, simplification)));
}

void FrameGraphModel::registerModule(Module& m) {
  Model::registerModule(m);
  if(auto trajPtr = m.ptrAs<PoseTrajectory>()){
    frameGraph_->add(&trajPtr->getFrame(), &trajPtr->getReferenceFrame(), trajPtr);
  }
}

void FrameGraphModel::init() {
  frameGraph_->init();
  Model::init();
}

} /* namespace calibration */
} /* namespace aslam */
