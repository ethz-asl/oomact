#include <aslam/calibration/tools/Parallelizer.hpp>

#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <sstream>

using aslam::calibration::Parallelizer;

TEST(Parallelizer, testParallel) {
  static const int WaitMillis = 5;
  std::stringstream s;
  std::mutex m;

  int N = 3;
  Parallelizer p(N);
  for(int i = 0 ; i < N + 1; i++)
  p.add([&](){
    {
      std::lock_guard<std::mutex> mlock(m);
      s << "s";
    }
      std::this_thread::sleep_for(std::chrono::milliseconds(WaitMillis));
    {
      std::lock_guard<std::mutex> mlock(m);
      s << "e";
    }
      std::this_thread::sleep_for(std::chrono::milliseconds(WaitMillis));
  });

  p.doAndWait();

  ASSERT_EQ(std::string(N, 's') + std::string(N, 'e') + "se", s.str());
}
