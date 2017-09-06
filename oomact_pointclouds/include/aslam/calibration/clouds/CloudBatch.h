#ifndef CLOUDBATCH_H_
#define CLOUDBATCH_H_

#include <functional>
#include <memory>
#include <unordered_map>

#include <aslam/calibration/Timestamp.hpp>
#include <Eigen/Core>
#include <sm/kinematics/Transformation.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

#include "NanPolicy.h"
#include "PointCloudsPlugin.h"
#include "PointMatcherIncludes.h"

namespace aslam {
namespace calibration {

class PointCloudSensor;
class Sensor;

template <class T>
void hash_combine(std::size_t& seed, const T& v)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
};

template<typename P>
struct pair_hash
{
  inline std::size_t operator()(const P& v) const
  {
    std::size_t seed = 0;
    hash_combine(seed, v.first);
    hash_combine(seed, v.second);
    return seed;
  }
};

template <typename Derived>
struct Id {
  explicit Id(size_t value) : value(value) {}

  bool operator == (const Derived& other) const { return value == other.value; }
  bool operator < (const Derived other) const { return value < other.value; }
  bool operator > (const Derived other) const { return value > other.value; }
  bool operator <= (const Derived other) const { return value <= other.value; }
  bool operator >= (const Derived other) const { return value >= other.value; }
  friend std::ostream& operator << (std::ostream& o, const Id& id) {
    o << id.value; return o;
  }
  friend std::string operator + (const std::string& s, const Id& id) {
    return s + std::to_string(id.value);
  }
  size_t getValue() const {
    return value;
  }
 protected:
  size_t value;
};
namespace cloud_batch {
struct SensorId : public Id<SensorId> {
  SensorId(const Sensor &);
};
}

struct CloudId : public Id<CloudId> {
  using Id::Id;
};

struct PointMeasurement {
   Timestamp t;
   PmVector3 p;
};

typedef Eigen::Matrix<Timestamp, Eigen::Dynamic,1> TimestampsInputVector;

class CloudMeasurements {
 public:
  CloudMeasurements(NanPolicy nanPolicy);
  virtual ~CloudMeasurements() = default;

  virtual void fillMeasurementsInto(const MeasurementTransformer& transformer, PointCloudView cloud, PointDescriptor::RowXpr indices) const = 0;

  virtual const PointMeasurement& getMeasurement(const size_t index) const = 0;

  Timestamp getTimestamp(const size_t index) const { return getMeasurement(index).t; }

  virtual void deleteUpToStartTimestamp(Timestamp start) = 0;
  virtual void deleteFromEndTimestamp(Timestamp end) = 0;

  virtual size_t getSize() const = 0;
  bool isEmpty() const { return getSize() == 0; }

  Timestamp getCenterTimestamp() const {
    return minTimestamp / Timestamp(2.0) + maxTimestamp / Timestamp(2.0); //TODO A FIX : this could be bad for shortened point clouds!
  }

  Timestamp getDuration() const {
    return maxTimestamp - minTimestamp;
  }

  Timestamp getMaximalDuration() const {
    return maximalDuration;
  }

  void setMaximalDuration(Timestamp maximalDuration) {
    this->maximalDuration = maximalDuration;
  }

  Timestamp getMinTimestamp() const {
    return minTimestamp;
  }

  Timestamp getMaxTimestamp() const {
    return maxTimestamp;
  }

 protected:
  Timestamp minTimestamp = Timestamp::Zero();
  Timestamp maxTimestamp = Timestamp::Zero();
  Timestamp maximalDuration = 10.0;

  const NanPolicy nanPolicy_;
};

class EuclideanCloudMeasurements : public CloudMeasurements {
 public:
  using CloudMeasurements::CloudMeasurements;

  void deleteUpToStartTimestamp(Timestamp start) override;
  void deleteFromEndTimestamp(Timestamp end) override;

  void fillMeasurementsInto(const MeasurementTransformer& transformer, PointCloudView cloud, PointDescriptor::RowXpr indices) const override;

  const PointMeasurement& getMeasurement(const size_t index) const override;

  size_t getSize() const override { return measurements.size(); }

  int addData(const TimestampsInputVector& timestamps, const PointCloud& scan, size_t useFirstN);
  int addData(const std::vector<Timestamp>& timestamps, const PointCloud& scan, size_t useFirstN);

 protected:
  template <typename TimeFunctor, typename Scan>
  int addDataInternal(TimeFunctor timestamps, const Scan& scan, size_t size);

  std::vector<PointMeasurement> measurements;
};


class CloudBatch;

}
}

namespace std {
template <typename> class hash;
template <>
struct hash<reference_wrapper<const aslam::calibration::CloudBatch> > {
 public :
  size_t operator()(const reference_wrapper<const aslam::calibration::CloudBatch>&ref) const {
    return hash<size_t>()(reinterpret_cast<size_t>(&(ref.get())));
  }
};

inline bool operator == (reference_wrapper<const aslam::calibration::CloudBatch> a, reference_wrapper<const aslam::calibration::CloudBatch> b);
}


namespace aslam {
namespace calibration {

class CloudBatch{

      // TODO: Define the clean functions, if we want to implement a reoptimization,
      // - we need to save all the measurements, as they are, in a std::vector
      //
      // - for each run of the pipeline, building the DataPoints for pointmatcher
      //   WITHOUT deleting the std::vector of the measurements, penalty we have to
      //   refill it
      //
      // - when we finish the buildCloudProblem, we can delete the matrices, it causes
      //   an overhead but it is just related to the span of the buildCloudProblem function
      //   so we need a function to delete the matrices, as cleanPointCloudsPlugin()
      //
      // - when we add the error terms and we need to clean the measurements, we need another function
      //   in the cleanMeasurements in calibrator, that deletes the associations
      //
      // - we need to keep ALL the initial points, without filtering, since another spline
      //   will positionate the points in different parts, so we need to refilter all (it is good)
      //   We don't need to create another vector of indices since the first (referring to the whole)
      //   cloud is sufficient,
      //
      // - Find a way to estimate the covariance of the normals, and add in a vector (if we want it spherical)
      //
      // - Add accessor functions to get the minimun angle, timestamp, ecc. as done
      //   in Jerome library

      // TODO: Adapt the structure to accept intensity measurement, for the future
      // integration with the cameras
     public:
      typedef PM::IntMatrix::Scalar Index;
      typedef TP Transformation;

      typedef cloud_batch::SensorId SensorId;

      typedef std::pair<Index, Index> IndexPair;

      struct MatchData {
        PmScalar dist;
        PmScalar weight;
        MatchData(double d, double w):dist(d),weight(w){}
      };

      struct Association {
        IndexPair matchIndixPair;
        MatchData data;
      };

      typedef std::vector<Association> Associations;
      struct AssociationsAndTransformation {
        Associations associations;
        Transformation T_ref_read;
        size_t size() const { return associations.size(); }
      };

      typedef std::unordered_map<IndexPair, MatchData, pair_hash<IndexPair>> MatchingPair2DataMap;
      struct AssociationsMapAndTransformation { //TODO O avoid that intermediate step of AssociationsAndTransformation
        MatchingPair2DataMap matchingPair2DataMap;
        Transformation T_ref_read;
        size_t size() const { return matchingPair2DataMap.size(); }

        AssociationsMapAndTransformation& operator = (const AssociationsAndTransformation& other);
      };


      typedef calibration::TimestampsInputVector TimestampsInputVector; // TODO C remove for compatibility
      typedef calibration::PointCloud PointCloud;

      CloudBatch(const PointCloudSensor& sensor, CloudId id);
      CloudBatch(CloudBatch &&) = default;
      virtual ~CloudBatch();

      void clearAssociations();
      void createCloud();

      void transformMeasurementsIntoCloud(const MeasurementTransformer& transformer);

      void saveFilteredCloud(const std::string& fileName) const;

      void saveAssociations(const std::string& fileName) const;

      void saveAlignedCloud(const std::string& fileName) const;

      size_t getMeasurementIndexFromFilteredIndex(const size_t indexFiltered) const;
      Timestamp getMeasurementTimestamp(const size_t indexFiltered) const;

      const PmVector3&getMeasurementFeature(const size_t index) const;
      const PointMeasurement& getMeasurementByFilteredIndex(const size_t index) const;

      PmVector3 getNormal(const size_t index) const;
      PmVector3 getEigenValues(const size_t index) const;

      bool hasMatchesTo(const CloudBatch& other) const {
        return matches.count(other) > 0;
      }
      const CloudBatch::AssociationsMapAndTransformation& getMatchesTo(const CloudBatch& other) const {
        return matches.at(other);
      }
      CloudBatch::AssociationsMapAndTransformation& getMatchesTo(const CloudBatch& other){
        return matches.at(other);
      }
      CloudBatch::AssociationsMapAndTransformation& getOrCreateMatchesTo(const CloudBatch& other){
        return matches[other];
      }


      void updateIndices();

      size_t getFilteredSize() const {
        return cloudScan.features.cols();
      }

      bool hasNormals() const;

      Timestamp getCenterTimestamp() const {
        return getMeasurements().getCenterTimestamp();
      }
      Timestamp getDuration() const {
        return getMeasurements().getDuration();
      }
      Timestamp getMinTimestamp() const {
        return getMeasurements().getMinTimestamp();
      }
      Timestamp getMaxTimestamp() const {
        return getMeasurements().getMaxTimestamp();
      }
      Timestamp getMaximalDuration() const {
        return getMeasurements().getMaximalDuration();
      }
      void setMaximalDuration(Timestamp maximalDuration) {
        getMeasurements().setMaximalDuration(maximalDuration);
      }
      bool isEmpty() const {
        return getMeasurements().isEmpty();
      }

      bool isClosed() const {
        return closed;
      }
      void close() {
        this->closed = true;
      }

      const PointCloudSensor& getSensor() const {
        return sensor;
      }

      CloudMeasurements& getMeasurements() {
        return *measurements;
      }
      const CloudMeasurements& getMeasurements() const {
        return *measurements;
      }

      template <typename CloudMeasurementsImpl>
      CloudMeasurementsImpl& getMeasurements() {
        static_assert(std::is_base_of<CloudMeasurements, CloudMeasurementsImpl>::value, "");
        return static_cast<CloudMeasurementsImpl&>(*measurements);
      }
      template <typename CloudMeasurementsImpl>
      const CloudMeasurementsImpl& getMeasurements() const {
        static_assert(std::is_base_of<CloudMeasurements, CloudMeasurementsImpl>::value, "");
        return static_cast<const CloudMeasurementsImpl&>(*measurements);
      }

      const DP& getCloud() const { return cloudScan; }

      template <typename F>
      void applyFilter(F f){ f(cloudScan); updateCloudInfo(); }

      typedef std::unordered_map<std::reference_wrapper<const CloudBatch>, AssociationsMapAndTransformation> SensorCloud2AssociationsMap;

      const SensorCloud2AssociationsMap& getSensorCloud2AssociationsMap() const { return matches; }

      friend std::ostream& operator <<(std::ostream& out, const CloudBatch& c); //TODO C move operator <<(std::ostream& out, const CloudBatch& c) implementation here or merge otherwise

      CloudId getId() const { return id; }
      operator CloudId () const { return id; }

      bool operator == (const CloudBatch& other) const {
        return this == &other;
      }

     private:
      void updateCloudInfo();
      unsigned getDescriptorIndexIfExists(const std::string& descName) const;

      const PointCloudSensor& sensor;
      CloudId id;
      bool created = false;
      bool closed = false;

      DP cloudScan;
      SensorCloud2AssociationsMap matches;
      std::vector<size_t> indicesMeasCloud; // Vector of auxiliary indices to relate filtered cloud to initial one

      int indexRow = -1;
      int normalsRow = -1;
      int eigenValuesRow = -1;

      std::unique_ptr<CloudMeasurements> measurements;
};

}
}

namespace std {
bool operator == (reference_wrapper<const aslam::calibration::CloudBatch> a, reference_wrapper<const aslam::calibration::CloudBatch> b){
  return a.get().operator == (b);
}
}

#endif /* CLOUDBATCH_H_ */
