#ifndef H786129C5_656D_4519_AC5E_FCC314B1BC7F
#define H786129C5_656D_4519_AC5E_FCC314B1BC7F


namespace aslam {
namespace calibration {

template <typename MsgT>
MeasurementsContainer<MsgT> getMeasurementsSlice(const MeasurementsContainer<MsgT> & allMeasurements, Timestamp from, Timestamp till) {
  MeasurementsContainer<MsgT> measurements;
  std::copy_if(allMeasurements.begin(), allMeasurements.end(), measurements.begin(), [=](const PoseMeasurements::value_type & p){
    Timestamp t = p.first;
    return t <= till && t >= from;
  });
  return measurements;
}

}
}


#endif /* H786129C5_656D_4519_AC5E_FCC314B1BC7F */
