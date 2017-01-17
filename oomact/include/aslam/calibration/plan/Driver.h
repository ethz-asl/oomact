#ifndef H1EE6111A_99E9_42EE_92B8_E143CA9B1A09
#define H1EE6111A_99E9_42EE_92B8_E143CA9B1A09

#include <string>

namespace aslam {
namespace calibration {
namespace plan {

class Driver {
 public:
  Driver();

  virtual void setVelocity(double tv, double rv) = 0;
  void stop() { setVelocity(0., 0.); }

  virtual void markHome() = 0;

  virtual void goHome() = 0;

  virtual bool hasHome() const = 0;

  virtual double getHomeYaw() const = 0;

  virtual void turnTo(double yaw) = 0;

  virtual std::string getHome() const = 0;
  virtual void setHome(const std::string & homeString) = 0;

  virtual void announceDriving(const std::string & s) = 0;

  virtual ~Driver();
};

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H1EE6111A_99E9_42EE_92B8_E143CA9B1A09 */
