#include <aslam/calibration/tools/ErrorTermStatistics.h>

namespace aslam {
namespace calibration {

bool ErrorTermStatistics::add(aslam::backend::ErrorTerm& e, bool ignoreInactive) {
  if (ignoreInactive || errorTermIsActive(e)) {
    cost += evaluateError ? e.evaluateError() : e.getSquaredError();
    counter++;
    return true;
  } else {
    inactiveCounter++;
    return false;
  }
}

std::ostream& aslam::calibration::ErrorTermStatistics::printInto(std::ostream& out) const {
  out << "Total initial "<< name << " cost : " << cost << " in " << counter << " error terms.";
  if(counter) out << " Avg=" << (cost / counter) << ".";
  if(inactiveCounter) out << " Inactive=" << inactiveCounter << ".";
  if(skipCounter) {
    out << " Skipped=" << skipCounter << ".";
  }
  return out;
}

std::ostream& aslam::calibration::ErrorTermStatistics::printShortInto(std::ostream& out) const {
  out << name << "(";

  if(counter == 1){
    out << cost;
  } else if(counter) {
    out  << cost << " / " << counter << " = " << (cost / counter);
  } else {
    out << "NONE";
  }

  if(inactiveCounter){
    out << ", " << inactiveCounter;
  }

  if(skipCounter) {
    out << ", S=" << skipCounter;
  }
  out << ")";
  return out;
}

} /* namespace calibration */
} /* namespace aslam */
