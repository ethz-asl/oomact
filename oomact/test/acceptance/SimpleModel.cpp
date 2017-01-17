#include <cmath>

#include <gtest/gtest.h>

#include <aslam/calibration/model/Model.h>
#include <sm/BoostPropertyTree.hpp>
#include <sm/PropertyTree.hpp>
#include <sm/value_store/ValueStore.hpp>

#include "SimpleModel.hpp"
namespace aslam {
namespace calibration {
namespace {

class SimpleModelFrame : public Frame, public NamedMinimal {
  using NamedMinimal::NamedMinimal;
} global("Global"), lowerBody("LowerBody"), upperBody("UpperBody");

}

SimpleModel::SimpleModel(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver)
  : Model(config, configPathResolver, {&global, &lowerBody, &upperBody}),
    config_(config),
    sensorsConfig_(config.getChild("sensors")),
    wheelOdometry_(*this, "WheelOdometry", sensorsConfig_)
{
  wheelOdometry_.registerWithModel();
}

}
}
