#include <aslam/calibration/model/Model.h>

#include <gtest/gtest.h>
#include <sstream>

#include <aslam/calibration/test/SimpleModel.h>
#include <aslam/calibration/test/Tools.h>

using namespace aslam::calibration;
using namespace aslam::calibration::test;

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

TEST(Model, modelUid) {
  ValueStoreRef config;
  Model m(config, std::make_shared<SimpleConfigPathResolver>());
  MockModule mm1(m, "A", config);
  MockModule mm2(m, "A", config);

  m.addModule(mm1);
  ASSERT_EQ(mm1.getUid(), "A");
  m.addModule(mm2);
  ASSERT_EQ(mm2.getUid(), "A1");
}

TEST(Model, getSensors) {
  ValueStoreRef config = ValueStoreRef::fromString(
      "frames=a:b,"
      "S1{referenceFrame=a, rotation/used=false,translation/used=false,delay/used=false}"
      "S2{referenceFrame=b, rotation/used=false,translation/used=false,delay/used=false}"
      );

  Model m(config, std::make_shared<SimpleConfigPathResolver>());
  MockModule mm1(m, "A", config);
  MockSensor s1(m, "S1", config);
  MockSensor2 s2(m, "S2", config);

  m.addModule(mm1);
  m.addModule(s1);
  m.addModule(s2);

  EXPECT_EQ(2u, m.getSensors().size());
  EXPECT_EQ(2u, m.getSensors<MockSensor>().size());
  EXPECT_EQ(1u, m.getSensors<MockSensor2>().size());
  EXPECT_EQ(0u, m.getSensors<MockSensor3>().size());
}
