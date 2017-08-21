#include <aslam/calibration/model/Model.h>

#include <gtest/gtest.h>
#include <sstream>

#include "../test_tools/SimpleModel.hpp"
#include "../test_tools/Tools.hpp"

using namespace aslam::calibration;

class MockModule : public Module {
  using Module::Module;
};

class MockSensor : public Sensor {
  using Sensor::Sensor;
};
class MockSensor2 : public MockSensor {
  using MockSensor::MockSensor;
};
class MockSensor3 : public MockSensor {
  using MockSensor::MockSensor;
};

TEST(Module, modelUid) {
  ValueStoreRef config;
  Model m(config, std::make_shared<SimpleConfigPathResolver>());
  MockModule mm1(m, "A", config);
  MockModule mm2(m, "A", config);

  m.add(mm1);
  ASSERT_EQ(mm1.getUid(), "A");
  m.add(mm2);
  ASSERT_EQ(mm2.getUid(), "A1");
}


TEST(Module, getSensors) {

  ValueStoreRef config = ValueStoreRef::fromString(
      "frames=a:b,"
      "S1{frame=a, rotation/used=false,translation/used=false,delay/used=false}"
      "S2{frame=b, rotation/used=false,translation/used=false,delay/used=false}"
      );
;
  Model m(config, std::make_shared<SimpleConfigPathResolver>());
  MockModule mm1(m, "A", config);
  MockSensor s1(m, "S1", config);
  MockSensor2 s2(m, "S2", config);

  m.add(mm1);
  m.add(s1);
  m.add(s2);

  EXPECT_EQ(2, m.getSensors().size());
  EXPECT_EQ(2, m.getSensors<MockSensor>().size());
  EXPECT_EQ(1, m.getSensors<MockSensor2>().size());
  EXPECT_EQ(0, m.getSensors<MockSensor3>().size());
}
