#include <glog/logging.h>
#include <gtest/gtest.h>

void setupGoogleStderrLogging(bool verbose){
  google::SetStderrLogging(verbose ? google::INFO : google::WARNING);
}

void initGloogleLogging(unsigned verbosity) {
  fLB::FLAGS_colorlogtostderr = true;
  if(verbosity > 0){
    fLI::FLAGS_v = verbosity - 1;
  }
  google::InitGoogleLogging("");

  setupGoogleStderrLogging(verbosity > 0);

  LOG(INFO) << "Set GLOG verbosity to " << fLI::FLAGS_v << ".";
}

int main(int argc, char** argv) {
  int verbosity = 0;
  if(argc > 1 && std::string("-v") == argv[1]){
    verbosity = 2;
  }
  initGloogleLogging(verbosity);  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
