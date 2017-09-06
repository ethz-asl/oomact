#ifndef H46B27761_4FBB_4A10_B49C_82BC4B6443E1
#define H46B27761_4FBB_4A10_B49C_82BC4B6443E1

#include <string>

#include "PointMatcherIncludes.h"

namespace aslam {
namespace calibration {
class CloudBatch;

class PmFilter {
 public:
  PmFilter(const std::string& filterConfigFile);
  void filter(CloudBatch& cloud) const;
  void filter(DP& data) const;

  virtual ~PmFilter() {
  }
 private:
  DataPointsFiltersSP pmFilters;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H46B27761_4FBB_4A10_B49C_82BC4B6443E1 */
