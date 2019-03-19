#include <aslam/calibration/model/FrameGraphModel.h>

#include <glog/logging.h>

#include <aslam/backend/KinematicChain.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>
#include <aslam/calibration/model/fragments/So3R3Trajectory.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/FrameLinkI.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/model/fragments/PoseCv.h>
#include <aslam/calibration/tools/Tree.h>

using aslam::backend::CoordinateFrame;


namespace aslam {
namespace calibration {
std::ostream & operator << (std::ostream& o, const Frame * f) {
  return o << *f;
}

struct FrameLinkStorage {
  union Ptr {
    const AbstractStaticFrameLink* staticFrameLink;
    const FrameLinkI* frameLink;

    Ptr(const AbstractStaticFrameLink* staticFrameLink) : staticFrameLink(staticFrameLink) {}
    Ptr(const FrameLinkI* frameLink) : frameLink(frameLink) {}
  } ptr;
  enum class Type {
    Static, NonStatic
  } type;

  FrameLinkStorage(const AbstractStaticFrameLink* staticFL) : ptr(staticFL), type(Type::Static) {}
  FrameLinkStorage(const FrameLinkI* frameLink) : ptr(frameLink), type(Type::NonStatic) {}
};

class FrameGraph: public Tree<const Frame*, FrameLinkStorage> {
};


boost::shared_ptr<CoordinateFrame> relativeKinematics2CF(boost::shared_ptr<CoordinateFrame> parent,
                                                         const RelativeKinematicExpression & rk) {
  return boost::make_shared<CoordinateFrame>(
      parent,
      rk.R, rk.p, rk.omega, rk.v, rk.alpha, rk.a
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

  CoordinateFrame getKinematicChainFromTo(const Frame & fromLocal, const Frame & toGlobal, const size_t maximalDerivativeOrder) const {
    boost::shared_ptr<CoordinateFrame> f, fInv;
    fgModel_.frameGraph_->walkPath(&toGlobal , &fromLocal, [&](const FrameLinkStorage & frameLink, bool towardsLeafs){
      CHECK(towardsLeafs) << "Only walking to leaf frames is currently supported! Path was: to=" << fromLocal << ", from=" << toGlobal;
      switch(frameLink.type){
        case FrameLinkStorage::Type::Static:
          f = relativeKinematics2CF(f, frameLink.ptr.staticFrameLink->calcRelativeKinematics());
          break;
        case FrameLinkStorage::Type::NonStatic:
          f = relativeKinematics2CF(f, frameLink.ptr.frameLink->calcRelativeKinematics(timestamp_,
                    simplification_, maximalDerivativeOrder));
          break;
        default:
          CHECK(false);
      }

    });
    return std::move(*f);
  }

  aslam::backend::CoordinateFrame getKinematicChainFromTo(const Frame & fromLocal, const Frame & toGlobal) const {
    if(toGlobal == fromLocal) return CoordinateFrame();
    return getKinematicChainFromTo(fromLocal, toGlobal, maximalDerivativeOrder_);
  }

  aslam::backend::TransformationExpression getTransformationToFrom(const Frame & to, const Frame & from) const override {
    const Frame & closestCommonAncestor = *fgModel_.frameGraph_->getClosestCommonAncestor(&to, &from);
    auto to2ca = getKinematicChainFromTo(to, closestCommonAncestor);
    auto from2ca = getKinematicChainFromTo(from, closestCommonAncestor);
    return aslam::backend::TransformationExpression(to2ca.getR_G_L(), to2ca.getPG()).inverse() * aslam::backend::TransformationExpression(from2ca.getR_G_L(), from2ca.getPG());
  }

  aslam::backend::EuclideanExpression getAcceleration(const Frame & of, const Frame & in) const override {
    return getKinematicChainFromTo(of, in).getAG();
  }

  aslam::backend::EuclideanExpression getVelocity(const Frame & of, const Frame & in) const override {
    return getKinematicChainFromTo(of, in).getVG();
  }

  aslam::backend::EuclideanExpression getAngularVelocity(const Frame & of, const Frame & in) const override {
    return getKinematicChainFromTo(of, in).getOmegaG();
  }

  aslam::backend::EuclideanExpression getAngularAcceleration(const Frame & of, const Frame & in) const override {
    return getKinematicChainFromTo(of, in).getAlphaG();
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
  if (auto staticFLPtr = m.ptrAs<AbstractStaticFrameLink>()) {
    frameGraph_->add(&staticFLPtr->getFrame(), &staticFLPtr->getReferenceFrame(), FrameLinkStorage { staticFLPtr });
  } else if (auto flPtr = m.ptrAs<FrameLinkI>()) {
    frameGraph_->add(&flPtr->getFrame(), &flPtr->getReferenceFrame(), FrameLinkStorage { flPtr });
  }
}

void FrameGraphModel::init() {
  frameGraph_->init();
  Model::init();
}

} /* namespace calibration */
} /* namespace aslam */
