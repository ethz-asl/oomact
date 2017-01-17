#ifndef ASLAM_CALIBRATION_OPTIMIZATION_PROBLEM_SPLINE_H
#define ASLAM_CALIBRATION_OPTIMIZATION_PROBLEM_SPLINE_H

#include <vector>
#include <map>
#include <memory>

#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/calibration/model/StateCarrier.h>

namespace aslam {
  namespace calibration {
  class CalibratorI;
  struct EstimationConfiguration;

  /** The class OptimizationProblemSpline is a specialization of
        OptimizationProblem for handling splines.
        \brief Optimization problem for splines.
      */
    class OptimizationProblemSpline : public OptimizationProblem, public BatchStateReceiver {
    public:
      typedef std::unordered_map<std::reference_wrapper<StateCarrier>, BatchStateSP> BatchStates;

      OptimizationProblemSpline();
      OptimizationProblemSpline(const Self& other) = delete;
      OptimizationProblemSpline& operator = (const Self& other) = delete;
      OptimizationProblemSpline(Self&& other) = delete;
      OptimizationProblemSpline& operator = (Self&& other) = delete;
      virtual ~OptimizationProblemSpline();

      /// Insert a batch state into the problem
      void addBatchState(StateCarrier & stateCarrier, const BatchStateSP& batchState) override;

      const BatchStates& getBatchStates() const {
        return _batchStates;
      }

      void writeState(const CalibratorI & calib, const EstimationConfiguration & config) const;
    protected:
      BatchStates _batchStates;
    };
  }
}

#endif // ASLAM_CALIBRATION_OPTIMIZATION_PROBLEM_SPLINE_H
