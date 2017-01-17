#include "aslam/calibration/model/CalibrationVariable.h"

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/MarginalizationPriorErrorTerm.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/calibration/error-terms/ErrorTermGroup.h>
#include <aslam/calibration/error-terms/MeasurementErrorTerm.h>

#include <boost/algorithm/string.hpp>

#include <glog/logging.h>
#include <boost/make_shared.hpp>

using aslam::backend::ScalarExpression;

namespace aslam {
namespace calibration {

const ErrorTermGroupReference CvPriorGroup = getErrorTermGroup("CvPrior");


CalibrationVariable::~CalibrationVariable(){
}

void CalibrationVariable::printBasisInto(std::ostream& out, const Eigen::MatrixXd& mat) const {
  if(mat.rows())
    printFunctorInto(out, [&](int i){ out << mat.row(i).norm();}, mat.rows());
}

const int CalibrationVariable::NameWidth = 20;

const char* getActivityPrefix(const CalibrationVariable& cv) {
  return (cv.getDesignVariable().isActive() ? "* " : "  ");
}

void printNiceInto(std::ostream & out, const CalibrationVariable & cv, std::function<void(int)> f){
  out.fill(' ');
  for(int j = 0; j < cv.getDimension(); ++j){
    out << getActivityPrefix(cv);
    char c = cv.getTangentComponentName(j)[0];
    out << std::setw(CalibrationVariable::NameWidth) << ((j == 0) ? cv.getName() : " ");
    out << " " << (c ? c : ' ');
    out << ":";
    out.width(8);
    f(j);
    out << std::endl;
  }
}

void CalibrationVariable::printFunctorInto(std::ostream& out, std::function<void(int)> f, int limit) const {
  int i = getIndex();
  if(i >= 0){
    assert(i + getDimension() <= limit); static_cast<void>(limit); // for NDEBUG builds
    printNiceInto(out, *this, [&](int j){ f(i+j);});
  }
}

void CalibrationVariable::printValuesNiceInto(std::ostream & out) const {
  auto p = getMinimalComponents();
  auto disp = getDisplacementToLastUpdateValue();
  printNiceInto(out, *this, [&](int i){
    out << p(i);
    if(fabs(disp(i)) > 1e-9){
      out << " (" << (disp(i) > 0 ? "+" : "") << disp(i) << ")";
    }
  });
}


Eigen::MatrixXd CalibrationVariable::getParams() const {
  Eigen::MatrixXd params;
  getDesignVariable().getParameters(params);
  return params;
}

constexpr const char * DVComponentNames<backend::EuclideanPoint>::value[];
constexpr const char * DVComponentNames<backend::RotationQuaternion>::value[];


namespace internal {
  Eigen::VectorXd loadPacked(std::vector<ValueHandle<double>> &vhs){
    Eigen::VectorXd v(vhs.size());
    int i = 0;
    for(auto & vh : vhs){
      v(i++) = vh.get();
    }
    return v;
  }
  void storePacked(std::vector<ValueHandle<double>> &vhs, const Eigen::VectorXd & vPacked){
    int i = 0;
    for(auto & vh : vhs){
      if(vh.isUpdateable()){
        vh.update(vPacked[i]);
      }else{
        LOG(WARNING) << "Trying to update a non updateable value handler." ;
      }
      i++;
    }
  }
}
//TODO B this is wrong: check that it is not used and remove or fix it.
Eigen::Matrix3d pitchRollYawToMatrix(double roll, double pitch, double yaw){
  return sm::kinematics::EulerAnglesYawPitchRoll().parametersToRotationMatrix({yaw, pitch, roll});
}

Eigen::Vector3d matrixToRollPitchYaw(const Eigen::Matrix3d & m){
  Eigen::Vector3d v = sm::kinematics::EulerAnglesYawPitchRoll().rotationMatrixToParameters(m);
  return Eigen::Vector3d(v(2), v(1), v(0));
}

std::function<Eigen::Matrix3d(double roll, double pitch, double yaw)> eulerToMatrix = &pitchRollYawToMatrix;
std::function<Eigen::Vector3d(const Eigen::Matrix3d & m)> matrixToEuler = &matrixToRollPitchYaw;


Eigen::Vector4d ParamsPackTraits<backend::RotationQuaternion>::unpack(const Eigen::VectorXd & v){
  Eigen::Matrix3d M = eulerToMatrix(v(0), v(1), v(2));
  return sm::kinematics::r2quat(M);
}

Eigen::VectorXd ParamsPackTraits<backend::RotationQuaternion>::pack(const Eigen::VectorXd & v){
  return matrixToEuler(sm::kinematics::quat2r(v));
}

boost::shared_ptr<backend::ErrorTerm> PriorErrorTermCreater<backend::Scalar>::createPriorErrorTerm(CalibrationDesignVariable<backend::Scalar> &dv, Eigen::MatrixXd covSqrt){
  return boost::make_shared<MeasurementErrorTerm<1, ScalarExpression>>(dv.toExpression(), dv.getParams()(0, 0), covSqrt, CvPriorGroup, true);
}

using backend::MarginalizationPriorErrorTerm;
class QuaternionPriorErrorTerm : public MarginalizationPriorErrorTerm, public ErrorTermGroupMember {
 public:
  QuaternionPriorErrorTerm(const std::vector<aslam::backend::DesignVariable*>& designVariables,
                           const Eigen::VectorXd& d,
                           const Eigen::MatrixXd& R,
                           ErrorTermGroupReference r) : MarginalizationPriorErrorTerm(designVariables, d, R), ErrorTermGroupMember(r)
  {
  }

  virtual ~QuaternionPriorErrorTerm() = default;
};

boost::shared_ptr<backend::ErrorTerm> PriorErrorTermCreater<backend::RotationQuaternion>::createPriorErrorTerm(CalibrationDesignVariable<backend::RotationQuaternion> &dv,  Eigen::MatrixXd covSqrt) {
  auto err = boost::make_shared<QuaternionPriorErrorTerm>(std::vector<backend::DesignVariable*>({&dv}), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), CvPriorGroup);
  err->vsSetInvR((covSqrt * covSqrt.transpose()).inverse().eval());
  return err;
}


Covariance::Covariance(ValueStoreRef valueStore, int dim) {
  std::string s = valueStore.getString("sigma", std::string());
  if(s.empty()){
    covarianceSqrt.setIdentity(dim, dim);
  } else {
    const int commas = std::count(s.begin(), s.end(), ',');
    if(commas == 0){
      covarianceSqrt.setIdentity(dim, dim);
      covarianceSqrt *= valueStore.getDouble("sigma").get();
    } else if(commas == dim - 1){
      boost::replace_all(s, " ", "");
      covarianceSqrt.setIdentity(dim, dim);
      std::vector<std::string> parts;
      boost::split(parts, s, boost::is_any_of(","));
      int i = 0;
      for(auto p : parts){
        covarianceSqrt(i, i) = std::stod(p.c_str());
        i ++;
      }
      assert(i == dim);
    } else {
      throw std::runtime_error("Could not parse sigma value '" + s + "'");
    }
  }
}

}
}
