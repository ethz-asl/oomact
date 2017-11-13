#ifndef H786129C5_656D_4519_AC5E_FCC314B1BC7F
#define H786129C5_656D_4519_AC5E_FCC314B1BC7F
#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/data/PoseMeasurement.h>
#include <aslam/calibration/Timestamp.h>


namespace aslam {
namespace calibration {

template <typename MsgT>
MeasurementsContainer<MsgT> getMeasurementsSlice(const MeasurementsContainer<MsgT> & allMeasurements, Timestamp from, Timestamp till) {
  MeasurementsContainer<MsgT> measurements;
  std::copy_if(allMeasurements.begin(), allMeasurements.end(), measurements.begin(), [=](const typename MeasurementsContainer<MsgT>::value_type & p){
    Timestamp t = p.first;
    return t <= till && t >= from;
  });
  return measurements;
}


class Sensor;
class Frame;
class CalibratorI;

PoseMeasurement getFirstPoseMeasurement(CalibratorI & calib, Timestamp & startTime, const Sensor & sensor, bool respectDelayLowerBound, const Frame * transformToFramePtr);

}
}


#endif /* H786129C5_656D_4519_AC5E_FCC314B1BC7F */
