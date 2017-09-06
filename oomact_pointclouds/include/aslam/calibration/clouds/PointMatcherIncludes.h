#ifndef H39930A7A_37D9_4559_9106_3AC69E4A7BC8
#define H39930A7A_37D9_4559_9106_3AC69E4A7BC8

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pointmatcher/PointMatcher.h>

namespace aslam {
namespace calibration {

// Pointmatcher definitions
typedef float PmScalar;
typedef PointMatcher<PmScalar> PM;
typedef PM::TransformationParameters TP;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;
typedef PM::DataPointsFilters DataPointsFilters;
typedef std::shared_ptr<DP> DataPointsSP;
typedef std::shared_ptr<DataPointsFilters> DataPointsFiltersSP;
typedef PM::Matrix PointCloud;
typedef PM::Matrix PointDescriptor;
typedef DP::View PointCloudView;
template<int Rows> using PmVector = Eigen::Matrix<PmScalar, Rows, 1>;
template<int Rows, int Cols> using PmMatrix = Eigen::Matrix<PmScalar, Rows, Cols>;
typedef PmVector<3> PmVector3;
typedef PmVector<Eigen::Dynamic> PmVectorX;
}
}

#endif /* H39930A7A_37D9_4559_9106_3AC69E4A7BC8 */
