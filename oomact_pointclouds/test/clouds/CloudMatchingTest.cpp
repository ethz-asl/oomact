/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file CloudMatchingTest.cpp
    \brief This file tests the CloudMatching between two arbitrarily datasets.
 */

#include <cmath>
#include <vector>
#include <cmath>
#include <utility>
#include <unordered_set>
#include <iostream>

#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/random.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>
#include <sm/eigen/gtest.hpp>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/calibration/clouds/CloudsContainer.h>

#include "aslam/calibration/tools/tools.h"
#include "aslam/calibration/tools/Parallelizer.hpp"

using namespace aslam::backend;
using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testCloudMatching) {
  const int numPointCloudToTest = 3;

  // Files, not used for now
  std::string filePointCloud1 = "something";
  std::string filePointCloud2 = "somethingelse";
  std::string filePointCloud3 = "somethingelse2";
  // CloudsContainer creation

  // Pass the configuration file
  const std::string icpConfigFile = "icp_default.yaml";
  const std::string filterConfigFile ="notUsedItWillLoadDefault";
  // This is just to avoid to make the test run without configuration file
  ASSERT_TRUE(boost::filesystem::exists(icpConfigFile));

  const size_t num2dLidars = 0;
  CloudsContainer cloudsContainer(0, numPointCloudToTest, num2dLidars, icpConfigFile,filterConfigFile, [](size_t){ return std::string(); });


  /*cloudsContainer.lidar3dCloudBatches_[0].loadFromFile(filePointCloud1);
  cloudsContainer.lidar3dCloudBatches_[1].loadFromFile(filePointCloud2);
  cloudsContainer.lidar3dCloudBatches_[2].loadFromFile(filePointCloud3);*/

  // Creating the clouds

  // let's create a plane on x-z direction

  // Or it is better to create three planes? like cartesian axes (or half of a parallelepiped)

  // A identifies the first plane
  // B identifies the second plane, moved
  // C identifies the third plane, slightly moved

  // These should be sent by argument parameters
  const int numPointsEachPlane = 300;
  const int numPoints = numPointsEachPlane*3;
  const bool flagShuffle = true;

  const double zLength = 3;
  const double xLength = 5;
  const double yLength = 4;
  const double x0 = 0.0;
  const double z0 = 0.0;
  const double y0 = 0.0;

  std::vector<Eigen::VectorXd> pointsA;
  std::vector<Eigen::VectorXd> pointsB;
  std::vector<Eigen::VectorXd> pointsC;

  std::vector<Eigen::VectorXd> normalsA;
  std::vector<Eigen::VectorXd> normalsB;
  std::vector<Eigen::VectorXd> normalsC;

  pointsA.reserve(numPoints);
  pointsB.reserve(numPoints);
  pointsC.reserve(numPoints);
  normalsA.reserve(numPoints);
  normalsB.reserve(numPoints);
  normalsC.reserve(numPoints);
  // Create times (Not strictly necessary but needed by the Associations)
  std::vector<Timestamp> timesA(numPoints, Timestamp(1E9));
  std::vector<Timestamp> timesB(numPoints, Timestamp(2E9));
  std::vector<Timestamp> timesC(numPoints, Timestamp(3E9));

  ASSERT_EQ(timesA.size(),numPoints);

  // Create two Transformation matrices
  auto T_b_a = sm::kinematics::Transformation();
  auto T_c_a = sm::kinematics::Transformation();

  // sm::kinematics::UncertainTransformation;
  // Set some bounded random values:

  T_b_a.setRandom(2,M_PI/8);
  T_c_a.setRandom(0.1, 0.1);

  // Create measurements for plane xy
  for(int i=0; i < numPointsEachPlane; i++){
    double xRand = xLength * sm::random::uniform() + x0;
    double yRand = yLength * sm::random::uniform() + y0;
    auto pA = (Eigen::Vector3d() << xRand, yRand, 0.0).finished();
    auto nA = (Eigen::Vector3d() << 0.0, 0.0, 1.0).finished();
    pointsA.push_back(pA);
    normalsA.push_back(nA);

    pointsB.push_back(T_b_a * pA);
    normalsB.push_back(T_b_a.rotate(nA));

    pointsC.push_back(T_c_a * pA);
    normalsC.push_back(T_c_a.rotate(nA));

  }


  // Set the random seed

  //sm::random::seed();
  // Create measurements for plane yz
  for(int i=0; i < numPointsEachPlane; i++){
    double zRand = zLength * sm::random::uniform() + z0;
    double yRand = yLength * sm::random::uniform() + y0;
    auto pA = (Eigen::Vector3d() << 0.0, yRand, zRand).finished();
    auto nA = (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished();
    pointsA.push_back(pA);
    normalsA.push_back(nA);

    pointsB.push_back(T_b_a * pA);
    normalsB.push_back(T_b_a.rotate(nA));

    pointsC.push_back(T_c_a * pA);
    normalsC.push_back(T_c_a.rotate(nA));

  }
  // Create measurements for plane xz
  for(int i=0; i < numPointsEachPlane; i++){
    double xRand = xLength * sm::random::uniform() + x0;
    double zRand = zLength * sm::random::uniform() + z0;
    auto pA = (Eigen::Vector3d() << xRand, 0.0, zRand).finished();
    auto nA = (Eigen::Vector3d() << 0.0, 1.0, 0.0).finished();
    pointsA.push_back(pA);
    normalsA.push_back(nA);

    pointsB.push_back(T_b_a * pA);
    normalsB.push_back(T_b_a.rotate(nA));

    pointsC.push_back(T_c_a * pA);
    normalsC.push_back(T_c_a.rotate(nA));

  }
  ASSERT_EQ(pointsA.size(),numPoints);
  ASSERT_EQ(pointsB.size(),numPoints);
  ASSERT_EQ(pointsC.size(),numPoints);
  ASSERT_EQ(normalsA.size(),numPoints);
  ASSERT_EQ(normalsB.size(),numPoints);
  ASSERT_EQ(normalsC.size(),numPoints);


  // Test: In order to check if the same elements with distance still appear, I want to
  // Shuffle the elements

  if(flagShuffle){
    std::random_shuffle(pointsA.begin(),pointsA.end());
    std::random_shuffle(pointsB.begin(),pointsB.end());
    std::random_shuffle(pointsC.begin(),pointsC.end());
  }


  // Add measurements to the batches

  cloudsContainer.lidar3dCloudBatches_[0].addDataWithNormals(timesA, pointsA, normalsA);
  cloudsContainer.lidar3dCloudBatches_[1].addDataWithNormals(timesB, pointsB, normalsB);
  cloudsContainer.lidar3dCloudBatches_[2].addDataWithNormals(timesC, pointsC, normalsC);

  // Tests on the correct management of the timestamps (Accessing directly the timestamps)
  ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[0].getTimestamp(0),timesA[0]);
  ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[1].getTimestamp(0),timesB[0]);
  ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[2].getTimestamp(0),timesC[0]);

  cloudsContainer.lidar3dCloudBatches_[0].createCloudWithNormals();
  cloudsContainer.lidar3dCloudBatches_[1].createCloudWithNormals();
  cloudsContainer.lidar3dCloudBatches_[2].createCloudWithNormals();



  // Tests on updating the indices:
  cloudsContainer.lidar3dCloudBatches_[0].updateIndices();
  cloudsContainer.lidar3dCloudBatches_[1].updateIndices();
  cloudsContainer.lidar3dCloudBatches_[2].updateIndices();

  // Assertions on the empty structures
  for(int i = 0; i < numPointCloudToTest; i++){
    ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[i].cloudAssociations[i].indices.size(), 0);
    //ASSERT_TRUE(cloudsContainer.lidar3dCloudBatches_[i].cloudScanVec.empty());
    ASSERT_TRUE(cloudsContainer.lidar3dCloudBatches_[i].matchingPairs[i].empty());
    //ASSERT_FALSE(cloudsContainer.lidar3dCloudBatches_[i].cloudTimestamps.empty());
    ASSERT_FALSE(cloudsContainer.lidar3dCloudBatches_[i].getMeasurements().empty());
    ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[i].getMeasurements().size(), numPoints);
    for(size_t j = 0; j < numPoints; j++){
      ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[i].getMeasurements()[j].second,
                cloudsContainer.lidar3dCloudBatches_[i].getFeature(j));

    }
    /*for(size_t j = 0; j < cloudsContainer.lidar3dCloudBatches_[i].cloudScan.cols(); j++){
      ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[i].getFeature(j),
                          cloudsContainer.lidar3dCloudBatches_[i].getMeasurementFeature(
                              cloudsContainer.lidar3dCloudBatches_[i].getMeasurementIndexFromFilteredIndex(j)));

        }*/

  }

  // Tests on the correct management of the timestamps
  ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[0].getMeasurementTimestamp(0),timesA[0]);
  ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[1].getMeasurementTimestamp(0),timesB[0]);
  ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[2].getMeasurementTimestamp(0),timesC[0]);



  // TODO: If the problem still persists, try to feed an initial guess (random around the random transformation)
  try {
    cloudsContainer.computeAssociations(SensorType::POINT_CLOUD_3D);
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

  // Test the transformations
  for(int i = 0; i < numPointCloudToTest - 1; i++){
    for(int j = i + 1; j < numPointCloudToTest; j++){
      auto &T_I = cloudsContainer.lidar3dCloudBatches_[i].cloudAssociations[j].T_ref_read;
      Eigen::MatrixXd T_J = cloudsContainer.lidar3dCloudBatches_[j].cloudAssociations[i].T_ref_read.inverse();

      sm::eigen::assertNear(T_I,
                            T_J,
                            1e-1, SM_SOURCE_FILE_POS,
                            std::string("Transformation pair (") + i + "," + j + ") do not match.");
    }
  }

  // Test that the number of associations in different clouds are different
  for(int i = 0; i < numPointCloudToTest - 1; i++){
    for(int j = i + 1; j < numPointCloudToTest; j++){
      const auto& associations_ij = cloudsContainer.lidar3dCloudBatches_[i].cloudAssociations[j];
      const auto& associations_ji = cloudsContainer.lidar3dCloudBatches_[j].cloudAssociations[i];

      // I and J
      ASSERT_EQ(associations_ij.indices.size(),associations_ij.matches.size());
      ASSERT_EQ(associations_ij.indices.size(),associations_ij.dists.size());
      // J and I
      ASSERT_EQ(associations_ji.indices.size(),associations_ji.matches.size());
      ASSERT_EQ(associations_ji.indices.size(),associations_ji.dists.size());

      //EXPECT_NE(associations_ij.indices.size(),associations_ji.indices.size()) << "Size of associations " << i << " and " << j << " are equal! This is not a big deal, it can be by chance";
    }
  }

  // Re-Assertions on the empty structures
  for(int i = 0; i < numPointCloudToTest; i++){
    ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[i].cloudAssociations[i].indices.size(), 0);
    ASSERT_FALSE(cloudsContainer.lidar3dCloudBatches_[i].getMeasurements().empty());
    ASSERT_TRUE(cloudsContainer.lidar3dCloudBatches_[i].matchingPairs[i].empty());
  }

  try {
    cloudsContainer.associationsIntersection();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

  // Re-Assertions on the empty structures
  for(int i = 0; i < numPointCloudToTest; i++){
    ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[i].cloudAssociations[i].indices.size(), 0);
    ASSERT_FALSE(cloudsContainer.lidar3dCloudBatches_[i].getMeasurements().empty());
    ASSERT_TRUE(cloudsContainer.lidar3dCloudBatches_[i].matchingPairs[i].empty());
  }

  // Checking the symmetric associations
  for(int i = 0; i < numPointCloudToTest; i++){
    for(int j = 0; j < numPointCloudToTest; j++){
      if(i==j)
        continue;
      const auto& readMatchingPairs = cloudsContainer.lidar3dCloudBatches_[i].matchingPairs[j];
      const auto& refMatchingPairs =  cloudsContainer.lidar3dCloudBatches_[j].matchingPairs[i];

      ASSERT_EQ(readMatchingPairs.size(), refMatchingPairs.size());
      for(auto it = readMatchingPairs.begin(); it != readMatchingPairs.end(); ++it){
        ASSERT_NE(refMatchingPairs.find(std::make_pair(it->first.second,it->first.first)),refMatchingPairs.end());
      }
    }
  }

  // Re-Assertions on the empty structures
  for(int i = 0; i < numPointCloudToTest; i++){
    ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[i].cloudAssociations[i].indices.size(), 0);
    ASSERT_FALSE(cloudsContainer.lidar3dCloudBatches_[i].getMeasurements().empty());
    ASSERT_TRUE(cloudsContainer.lidar3dCloudBatches_[i].matchingPairs[i].empty());
  }


  // Test random subsample
  try {
    cloudsContainer.associationsRandomSubsample(1.0);
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
  // Re-Assertions on the empty structures
  for(int i = 0; i < numPointCloudToTest; i++){
    ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[i].cloudAssociations[i].indices.size(), 0);
    ASSERT_FALSE(cloudsContainer.lidar3dCloudBatches_[i].getMeasurements().empty());
    ASSERT_TRUE(cloudsContainer.lidar3dCloudBatches_[i].matchingPairs[i].empty());
  }

  // Re-Assertions on the empty structures
  for(int i = 0; i < numPointCloudToTest; i++){
    ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[i].cloudAssociations[i].indices.size(), 0);
    ASSERT_FALSE(cloudsContainer.lidar3dCloudBatches_[i].getMeasurements().empty());
    ASSERT_TRUE(cloudsContainer.lidar3dCloudBatches_[i].matchingPairs[i].empty());
  }


  // Re-Test that the number of associations in different clouds are different
  for(int i = 0; i < numPointCloudToTest - 1; i++){
    for(int j = i + 1; j < numPointCloudToTest; j++){
      const auto& associations_ij = cloudsContainer.lidar3dCloudBatches_[i].cloudAssociations[j];
      const auto& associations_ji = cloudsContainer.lidar3dCloudBatches_[j].cloudAssociations[i];

      // I and J
      ASSERT_EQ(associations_ij.indices.size(),associations_ij.matches.size());
      ASSERT_EQ(associations_ij.indices.size(),associations_ij.dists.size());
      ASSERT_EQ(associations_ij.indices.size(),associations_ij.weights.size());
      // J and I
      ASSERT_EQ(associations_ji.indices.size(),associations_ji.matches.size());
      ASSERT_EQ(associations_ji.indices.size(),associations_ji.dists.size());
      ASSERT_EQ(associations_ji.indices.size(),associations_ji.weights.size());

      //EXPECT_NE(associations_ij.indices.size(),associations_ji.indices.size()) << "Size of associations " << i << " and " << j << " are equal! This is not a big deal, it can be by chance";
    }
  }

  // Tests on the correct management of the timestamps
  ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[0].getTimestamp(0),timesA[0]);
  ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[1].getTimestamp(0),timesB[0]);
  ASSERT_EQ(cloudsContainer.lidar3dCloudBatches_[2].getTimestamp(0),timesC[0]);


  // Save files:
  std::string folderName = "cloudTestResults/";
  std::string associationsName = folderName + "associations_batch_1/";
  std::string redAssociationsName = folderName + "reduced_associations_batch_1/";

  if (!boost::filesystem::exists(folderName))
    boost::filesystem::create_directory(folderName);
  if (!boost::filesystem::exists(associationsName))
    boost::filesystem::create_directory(associationsName);
  if (!boost::filesystem::exists(redAssociationsName))
    boost::filesystem::create_directory(redAssociationsName);
  cloudsContainer.saveTransformations(folderName);
  {
    Parallelizer p(1);
    cloudsContainer.saveFilteredClouds(folderName, 0, p);
  }
  cloudsContainer.saveAlignedClouds(folderName, 0);
  cloudsContainer.saveAssociations(folderName);
  cloudsContainer.saveReducedAssociations(folderName);
}
