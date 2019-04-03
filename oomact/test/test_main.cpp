#include <regex>

#include <glog/logging.h>
#include <gtest/gtest.h>

void setupGoogleStderrLogging(bool verbose){
  google::SetStderrLogging(verbose ? google::INFO : google::WARNING);
}

void initGloogleLogging(unsigned verbosity) {
  fLB::FLAGS_colorlogtostderr = true;
  fLB::FLAGS_logtostderr = verbosity > 0;
  if(verbosity > 0){
    fLI::FLAGS_v = verbosity - 1;
  }
  google::InitGoogleLogging("");

  setupGoogleStderrLogging(verbosity);

  LOG(INFO) << "Set GLOG verbosity to " << fLI::FLAGS_v << ".";
}

int main(int argc, char** argv) {
  int verbosity = 0;
  const std::regex number_pattern("[0-9]+", std::regex_constants::egrep);
  const std::regex verbose_pattern("-v([0-9]+|v*)", std::regex_constants::egrep);
  for(int i = 1; i < argc; i++){
    std::cmatch m;
    if (std::regex_match(argv[i], m, verbose_pattern)) {
      int removeArgs = 1;
      const auto& vArg = m[1];
      if (vArg.length() == 0) {
        if (i < argc - 1 && std::regex_match(argv[i + 1], number_pattern)) {
          verbosity += std::stoi(argv[i + 1]);
          removeArgs++;
        } else {
          verbosity += 1;
        }
      } else if (m.str(1)[0] != 'v') {
        verbosity += std::stoi(vArg);
      } else {
        verbosity += vArg.length() + 1;
      }
      std::copy(argv + i + removeArgs, argv + argc + 1, argv + i);
      argc -= removeArgs;
      i --;
    }
  }
  initGloogleLogging(verbosity);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
