#ifndef HFDABA723_6463_4DF7_A67E_03082EFD0425
#define HFDABA723_6463_4DF7_A67E_03082EFD0425

namespace aslam {
namespace calibration {

void checkNotNull(void *);

template <typename T>
inline T* checkNotNull(T*p) {
  if(!p) checkNotNull(p);
  return p;
}

} /* namespace calibration */
} /* namespace aslam */

#endif /* HFDABA723_6463_4DF7_A67E_03082EFD0425 */
