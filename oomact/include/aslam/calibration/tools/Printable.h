#ifndef HA3FC657B_AF46_41F4_B077_5156FF1D0989
#define HA3FC657B_AF46_41F4_B077_5156FF1D0989

#include <iosfwd>

namespace aslam {
namespace calibration {

class Printable {
 public:
  virtual void print(std::ostream & o) const = 0;
  virtual ~Printable() {}

  friend std::ostream & operator << (std::ostream & o, const Printable & p){
    p.print(o);
    return o;
  }

  std::string toString() const;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HA3FC657B_AF46_41F4_B077_5156FF1D0989 */
