#include <ros/ros.h>

#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/ros/RosInputProvider.h>
#include <aslam/calibration/calibrator/CalibratorI.hpp>
#include <aslam/calibration/model/sensors/PoseSensor.hpp>
#include <aslam/calibration/model/sensors/PositionSensor.hpp>

#include <sm/BoostPropertyTree.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>

namespace cal = aslam::calibration;

std::string getFilePathFromRosParam(const std::string& param_name,
                                    ros::NodeHandle* nh) {
  std::string file_path;
  if (!nh->getParam(param_name, file_path)) {
    ROS_FATAL_STREAM("Could not find " << file_path << ", exiting");
    exit(EXIT_FAILURE);
  }
  return file_path;
}

void loadSensorParameters(const cal::ValueStoreRef vs_sensors,
                          std::shared_ptr<cal::FrameGraphModel> model,
                          std::vector<std::unique_ptr<cal::Sensor>>* sensors) {
  DCHECK(sensors != nullptr);

  std::vector<cal::KeyValueStorePair> vs_sensors_vector =
      vs_sensors.getChildren();

  sensors->clear();
  sensors->reserve(vs_sensors_vector.size());
  for (cal::KeyValueStorePair vs_sensor : vs_sensors_vector) {
    const std::string sensor_name = vs_sensor.getKey();
    const std::string sensor_type = vs_sensor.getString("type");
    ROS_INFO_STREAM("  loading " << sensor_type << " sensor: " << sensor_name << "...");

    if (sensor_type == std::string("pose")) {
      sensors->emplace_back(new cal::PoseSensor(*model, sensor_name, vs_sensors));
    } else if (sensor_type == std::string("position")) {
      sensors->emplace_back(new cal::PositionSensor(*model, sensor_name, vs_sensors));
    } else if (sensor_type == std::string("imu")) {
      // sensors.emplace_back(new IMUSensor(*model, sensor_name, vs_sensors));
    } else {
      ROS_ERROR("  loading failed, sensor type not supported.");
    }

    model->add(*(sensors->back()));
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "CalibrationNode");

  ros::NodeHandle nh, nh_private("~");

  ROS_INFO("Started node.");

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(FLAGS_v > 0 ? google::INFO : google::WARNING);
  google::InstallFailureSignalHandler();

  ROS_INFO("Loading ros parameters...");

  std::string config_file = getFilePathFromRosParam("config_file", &nh_private);
  std::string bag_file = getFilePathFromRosParam("bag_file", &nh_private);
  std::string output_file = getFilePathFromRosParam("output_file", &nh_private);

  ROS_INFO("Loading config file...");

  cal::ValueStoreRef vs = cal::ValueStoreRef::fromFile(config_file);
  cal::ValueStoreRef vs_model = vs.getChild("model");

  ROS_INFO("Setting up model and trajectory...");

  std::shared_ptr<cal::FrameGraphModel> model = std::make_shared<cal::FrameGraphModel>(vs_model);
  cal::PoseTrajectory traj(*model, "traj", vs_model);
  model->addModule(traj);

  ROS_INFO("Loading sensor parameters...");

  std::vector<std::unique_ptr<cal::Sensor>> sensors;
  loadSensorParameters(vs.getChild("sensors"), model, &sensors);

  ROS_INFO_STREAM("Loaded " << sensors.size() << " sensors.");

  ROS_INFO("Setting up calibration...");

  model->init();

  std::unique_ptr<cal::BatchCalibratorI> calibrator = cal::createBatchCalibrator(vs.getChild("calibrator"), model);

  ROS_INFO("Loading data bag...");

  cal::ros::RosInputProvider(model).feedBag(bag_file, *calibrator);

  ROS_INFO("Starting calibration...");

  calibrator->calibrate();

  ROS_INFO("Calibration done, generating results...");

  for (const std::unique_ptr<cal::Sensor>& sensor : sensors) {
    ROS_INFO_STREAM("Sensor       : " << sensor->getName());
    ROS_INFO_STREAM("  Translation: " << sensor->getTranslationToParent().transpose());
    ROS_INFO_STREAM("  Rotation   : " << sensor->getRotationQuaternionToParent().transpose());
  }

  //dynamic_cast<sm::PropertyTreeValueStore&>(vs.getValueStore()).save(output_file);

  ROS_INFO("Saving done, Exiting");

  return 0;
}
