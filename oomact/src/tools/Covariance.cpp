#include <aslam/calibration/tools/Covariance.h>

#include <string>
#include <vector>

#include <boost/algorithm/string/replace.hpp>
#include <glog/logging.h>
#include <sm/value_store/ValueStore.hpp>

#include <aslam/calibration/tools/tools.h>

namespace aslam {
namespace calibration {

Covariance::Covariance(sm::ValueStoreRef valueStore, int dim, bool load) {
  if(!load){
    return;
  }
  std::string s = valueStore.getString("sigma", std::string());
  if(s.empty()){
    covarianceSqrt.setIdentity(dim, dim);
  } else {
    const int commas = std::count(s.begin(), s.end(), ',');
    if(commas == 0){
      covarianceSqrt.setIdentity(dim, dim);
      covarianceSqrt *= valueStore.getDouble("sigma").get();
    } else if(commas == dim - 1 || commas == dim * dim - 1){
      boost::algorithm::replace_all(s, " ", "");
      covarianceSqrt.setIdentity(dim, dim);
      std::vector<std::string> parts = splitString(s, ",");
      const bool isDiagOnly = (parts.size() == static_cast<size_t>(dim));
      int i = 0;
      CHECK(isDiagOnly || parts.size() == static_cast<size_t>(dim*dim));
      for(auto p : parts){
        if(isDiagOnly){
          covarianceSqrt(i, i) = std::stod(p.c_str());
        } else {
          covarianceSqrt(i / dim, i % dim) = std::stod(p.c_str());
        }
        i ++;
      }
    } else {
      throw std::runtime_error("Could not parse sigma value '" + s + "'");
    }
  }
}

bool isDiagonal(const Eigen::MatrixXd & x, const double threshold) {
  Eigen::MatrixXd xc = x;
  xc.diagonal().setZero();
  return xc.norm() < threshold;
}

std::ostream & operator << (std::ostream & o, const Covariance &c){
  const auto & vsqrt = c.getValueSqrt();
  o.fill(' ');
  if (isDiagonal(vsqrt, 1e-10)) {
    o << "diag(" << vsqrt.diagonal().transpose() << ")";
  }
  else {
    o << "[";
    for (int i = 0; i < vsqrt.rows(); i++){
      if (i) o << "; ";
      o << vsqrt.row(i);
    }
    o << "]";
  }
  o << "^2";
  return o;
}

} /* namespace calibration */
} /* namespace aslam */
