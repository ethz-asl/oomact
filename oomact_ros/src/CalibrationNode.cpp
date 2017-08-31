#include <ros/ros.h>

#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/ros/RosInputProvider.h>
#include <aslam/calibration/calibrator/CalibratorI.hpp>
#include <aslam/calibration/model/sensors/PoseSensor.hpp>

namespace cal = aslam::calibration;

int main(int argc, char **argv) {
  ros::init(argc, argv, "CalibrationNode");

  ros::NodeHandle nh, nh_private("~");

  ROS_INFO("Started node");

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(FLAGS_v > 0 ? google::INFO : google::WARNING);
  google::InstallFailureSignalHandler();

  ROS_INFO("Loading config file");

  std::string config_file;
  if (!nh_private.getParam("config_file", config_file)) {
    ROS_FATAL("Could not find config_file parameter, exiting");
    exit(EXIT_FAILURE);
  }

  cal::ValueStoreRef vs = cal::ValueStoreRef::fromFile(config_file);
  cal::ValueStoreRef vs_model = vs.getChild("model");

  ROS_INFO("Setting up model and trajectory");

  std::shared_ptr<cal::FrameGraphModel> model =
      std::make_shared<cal::FrameGraphModel>(vs_model);
  cal::PoseTrajectory traj(*model, "traj", vs_model);
  model->add(traj);

  ROS_INFO("Loading pose sensor parameters");

  int num_pose_sensors;
  nh_private.param("num_pose_sensors", num_pose_sensors, 0);
  if (num_pose_sensors < 0) {
    ROS_ERROR("Number of pose sensors cannot be negative");
    num_pose_sensors = 0;
  }

  std::vector<cal::PoseSensor> pose_sensors;
  pose_sensors.reserve(num_pose_sensors);
  for (size_t i = 0; i < num_pose_sensors; ++i) {
    pose_sensors.emplace_back(*model, std::string("pose") + std::to_string(i),
                              vs_model);
    model->add(pose_sensors.back());
  }

  ROS_INFO_STREAM("Loaded " << pose_sensors.size() << " pose sensors");

  ROS_INFO("Setting up calibration");

  model->init();

  std::unique_ptr<cal::BatchCalibratorI> calibrator =
      cal::createBatchCalibrator(vs.getChild("calibrator"), model);

  ROS_INFO("Loading data bag");

  std::string bag_file;
  if (!nh_private.getParam("bag_file", bag_file)) {
    ROS_FATAL("Could not find bag_file parameter, exiting");
    exit(EXIT_FAILURE);
  }

  cal::ros::RosInputProvider(model).feedBag(bag_file, *calibrator);

  ROS_INFO("Starting calibration...");

  calibrator->calibrate();

  ROS_INFO("Calibration done, generating results");

  // Output results
  for (size_t i = 0; i < pose_sensors.size(); ++i) {
    ROS_INFO_STREAM("Pose sensor  : " << i);
    ROS_INFO_STREAM("  Translation: "
                    << pose_sensors[i].getTranslationToParent().transpose());
    ROS_INFO_STREAM(
        "  Rotation   : "
        << pose_sensors[i].getRotationQuaternionToParent().transpose());
  }

  /*for (const boost::shared_ptr<aslam::calibration::CalibrationVariable>&
  calib_vals :
       model->getCalibrationVariables()) {
    calib_vals->updateStore();
  }*/

  //dynamic_cast<PropertyTreeValueStore>(vs.getValueStore()).save();

  return 0;
}
