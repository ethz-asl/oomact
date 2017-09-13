#ifndef H29435CF7_130E_456F_AE96_6E7B7FD37A7C
#define H29435CF7_130E_456F_AE96_6E7B7FD37A7C
#include <Eigen/Core>
#include <iosfwd>
#include <sm/value_store/ValueStore.hpp>


namespace aslam {
namespace calibration {

class Covariance {
 public:
  Covariance(sm::value_store::ValueStoreRef valueStore, int dim, bool load = true);

  const Eigen::MatrixXd & getValueSqrt() const{
    return covarianceSqrt;
  }
  Eigen::MatrixXd getValue() const{
    return covarianceSqrt.transpose() * covarianceSqrt;
  }

  friend std::ostream & operator << (std::ostream & o, const Covariance &);
 private:
  Eigen::MatrixXd covarianceSqrt;
};

}
}

#endif /* H29435CF7_130E_456F_AE96_6E7B7FD37A7C */
