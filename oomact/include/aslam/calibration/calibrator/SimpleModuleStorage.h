#ifndef INCLUDE_ASLAM_CALIBRATION_CALIBRATOR_SIMPLEMODULESTORAGE_H_
#define INCLUDE_ASLAM_CALIBRATION_CALIBRATOR_SIMPLEMODULESTORAGE_H_
#include <aslam/calibration/data/MapStorage.h>
#include <aslam/calibration/model/Module.h>

namespace aslam {
namespace calibration {

class SimpleModuleStorage : public ModuleStorage
{
 public:
  using ModuleStorage::ModuleStorage;

  void remove(ModuleStorage::Key key) override
  {
    impl.remove(key);
  }
  void clear()
  {
    impl.clear();
  }

  void * get(ModuleStorage::Key key) const override
  {
    return impl.get(key);
  }

  void add(ModuleStorage::Key key, StorageElement && data) override
  {
    impl.add(key, std::move(data));
  }

  size_t size() const override
  {
    return impl.size();
  }

 private:
  MapStorage<const Module*> impl;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* INCLUDE_ASLAM_CALIBRATION_CALIBRATOR_SIMPLEMODULESTORAGE_H_ */
