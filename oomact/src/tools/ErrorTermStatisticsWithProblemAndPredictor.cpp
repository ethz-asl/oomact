#include <aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h>

#include "aslam/calibration/calibrator/CalibratorI.h"

namespace aslam {
namespace calibration {

ErrorTermStatisticsWithProblemAndPredictor::ErrorTermStatisticsWithProblemAndPredictor(CalibratorI & calib, std::string name, aslam::backend::ErrorTermReceiver & passToETReceiver, bool observeOnly) :
    ErrorTermStatistics(name),
    passToETReceiver_(passToETReceiver),
    predictionWriterPtr(calib.createPredictionCollector(name)),
    observeOnly(observeOnly)
{
}

} /* namespace calibration */
} /* namespace aslam */
