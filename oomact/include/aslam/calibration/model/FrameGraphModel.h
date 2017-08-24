#ifndef H94A17E9A_126B_4B3A_BCCB_F82E8EEFA015
#define H94A17E9A_126B_4B3A_BCCB_F82E8EEFA015
#include <aslam/calibration/model/Model.h>

namespace aslam {
namespace calibration {

class PoseTrajectory;
class FrameGraph;

class FrameGraphModel : public Model {
 public:
  FrameGraphModel(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver = nullptr, const std::vector<const Frame *> frames = {});
  virtual ~FrameGraphModel();

  ModelAtTime getAtTime(Timestamp timestamp, int maximalDerivativeOrder, const ModelSimplification & simplification) const override;
  ModelAtTime getAtTime(const BoundedTimeExpression & boundedTimeExpresion, int maximalDerivativeOrder, const ModelSimplification & simplification) const override;

  void init() override;

 protected:
  void registerModule(Module & m) override;
 private:
  template <typename Time> friend class FrameGraphModelAtTimeImpl;
  std::unique_ptr<FrameGraph> frameGraph_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H94A17E9A_126B_4B3A_BCCB_F82E8EEFA015 */
