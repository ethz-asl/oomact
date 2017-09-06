#ifndef CALIBRATIONMATCHER_H_
#define CALIBRATIONMATCHER_H_
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "CloudBatch.h"

namespace aslam {
  namespace calibration {

    struct IICP;

    template <typename T>
    class ObjectPool;

    class CalibrationMatcher {
     public:
      CalibrationMatcher(const std::string& icpConfigName, const std::string& icpConfigData);

      CloudBatch::AssociationsAndTransformation getAssociations(const CloudBatch& reading, const CloudBatch& reference, const std::string& name, const std::vector<bool> * usePoints) const;

      void getMatchedPoints(
            const PM::DataPoints& requestedPts,
            const PM::DataPoints& sourcePts,
            const PM::Matches& matches,
            const PM::OutlierWeights& outlierWeights,
            CloudBatch::Associations& associations) const;


      void enableIcpInspection(bool active, const std::string& inspectionOutputFolder);

      bool isActive() const {
        return active_;
      }

     protected:
      std::shared_ptr<ObjectPool<IICP>> icps_;
      std::string configString_;

      std::shared_ptr<PM::Transformation> rigidTrans_;

      std::string inspectionOutputFolder;
      bool inspectionEnabled = false;
      bool active_;
    };
  }
}

#endif /* CALIBRATIONMATCHER_H_ */
