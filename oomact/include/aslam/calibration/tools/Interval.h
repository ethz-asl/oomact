#ifndef H461FA092_A14D_432E_BACF_DFCC10B95EB5
#define H461FA092_A14D_432E_BACF_DFCC10B95EB5

#include <aslam/backend/GenericScalarExpression.hpp>

#include "../model/fragments/DelayCv.h"
#include "../Timestamp.h"

namespace aslam {

namespace backend {
template <typename Scalar_>
class GenericScalar;
}

namespace calibration {
typedef aslam::backend::GenericScalar<Timestamp> TimeDesignVariable;
template <typename DesignVariable_, typename Bound>
class BoundedCalibrationVariable;
typedef BoundedCalibrationVariable<TimeDesignVariable, Timestamp> TimeDesignVariableCv;

class Module;
class DelayCv;

struct BoundedTimeExpression {
  backend::GenericScalarExpression<Timestamp> timestampExpresion;
  Timestamp lBound;
  Timestamp uBound;
};


struct Interval {
  Timestamp start = InvalidTimestamp();
  Timestamp end = InvalidTimestamp();
  Interval() = default;
  Interval(Timestamp start, Timestamp end) : start(start), end(end) {}

  void clear() {
    *this = Interval();
  }
  operator bool () const {
    return start >= Timestamp::Zero();
  }

  bool contains(Timestamp t) const {
    return start <= t && t <= end;
  }

  bool contains(Timestamp t, const TimeDesignVariableCv & delay) const;
  bool contains(const BoundedTimeExpression & t) const;
  bool containsModule(Timestamp t, const Module & sensor) const;
  bool contains(Timestamp t, const DelayCv & sensor) const;

  Timestamp getElapsedTime() const {
    return end - start;
  }
};

}
}



#endif /* H461FA092_A14D_432E_BACF_DFCC10B95EB5 */
