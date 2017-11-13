#ifndef ASLAM_CALIBRATION_MEASUREMENTS_CONTAINER_H
#define ASLAM_CALIBRATION_MEASUREMENTS_CONTAINER_H

#include <algorithm>
#include <utility>
#include <vector>

#include "../Timestamp.h"

namespace aslam {
namespace calibration {

//TODO C let calibrator maintain the measurement containers. The Sensors should only create them and the feeders feed them : Module.clearMeasurements and Module.addErrorTerms would move.
class MeasurementContainerI {
 public:
  virtual ~MeasurementContainerI() {
  }
  virtual void clear() = 0;
};

/** The structure MeasurementsContainer represents a generic measurements container.
 \brief Measurements container
 */
template<typename C> struct MeasurementsContainer : public MeasurementContainerI, private std::vector<std::pair<Timestamp, C> > {
  typedef std::vector<std::pair<Timestamp, C> > Super;
  using typename Super::value_type;
  using Super::begin;
  using Super::end;
  using Super::cbegin;
  using Super::cend;
  using Super::rbegin;
  using Super::rend;
  using Super::operator [];
  using Super::size;
  using Super::empty;
  using Super::back;
  using Super::front;
  using Super::clear;
  using Super::reserve;
  inline void push_back(const std::pair<Timestamp, C> & v) {
    emplace_back(v.first, v.second);
  }

  inline void clear() {
    Super::clear();
    maximalGap = Timestamp::Zero();
  }

  static bool lessThan(const typename Super::value_type & a, const typename Super::value_type & b) {
    return a.first < b.first;
  }

  inline void emplace_back(Timestamp t, const C & c) {
    if (empty() || t >= back().first) {
      Super::emplace_back(t, c);
      if (size() >= 2) {
        auto lastGap = back().first - (rbegin() + 1)->first;
        maximalGap = std::max(maximalGap, lastGap);
      }
    } else {
      auto p = std::make_pair(t, c);
      auto it = std::lower_bound(begin(), end(), p, lessThan);
      Super::insert(it, p);
      updateMaximalGap();
    }
  }

  MeasurementsContainer & operator =(const Super & other) {
    Super::operator =(other);
    std::sort(begin(), end(), lessThan);
    return *this;
  }

  Duration getMaximalTimeGap() const {
    return maximalGap;
  }
 private:
  Duration maximalGap = Timestamp::Zero();

  void updateMaximalGap() {
    Timestamp lastT = front().first;
    maximalGap = Timestamp::Zero();
    for (auto & m : *this) {
      auto lastGap = m.first - lastT;
      lastT = m.first;
      maximalGap = std::max(maximalGap, lastGap);
    }
  }
};

}
}

#endif // ASLAM_CALIBRATION_MEASUREMENTS_CONTAINER_H
