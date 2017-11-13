#include <aslam/calibration/tools/Tree.h>

#include <gtest/gtest.h>
#include <sstream>

typedef aslam::calibration::Tree<std::string, std::string> IntTree;

std::string a = "a";
std::string b = "b";
std::string i = "i";
std::string j = "j";
std::string k = "k";
std::string ij = "ij";
std::string jk = "jk";
std::string aj = "aj";
std::string bk = "bk";

TEST(Tree, getClosestCommonAncestor) {
  IntTree tree;
  EXPECT_TRUE(!tree.isInitialized());

  tree.add(i, j, ij);
  tree.add(j, k, jk);
  tree.add(a, j, aj);
  tree.add(b, k, bk);

  EXPECT_TRUE(!tree.isInitialized());

  tree.init();
  EXPECT_FALSE(!tree.isInitialized());

  EXPECT_EQ(i, tree.getClosestCommonAncestor(i, i));
  EXPECT_EQ(j, tree.getClosestCommonAncestor(j, j));
  EXPECT_EQ(k, tree.getClosestCommonAncestor(k, k));
  EXPECT_EQ(j, tree.getClosestCommonAncestor(a, i));
  EXPECT_EQ(j, tree.getClosestCommonAncestor(i, a));
  EXPECT_EQ(j, tree.getClosestCommonAncestor(i, j));
  EXPECT_EQ(j, tree.getClosestCommonAncestor(j, i));
  EXPECT_EQ(k, tree.getClosestCommonAncestor(b, i));
  EXPECT_EQ(k, tree.getClosestCommonAncestor(i, b));
  EXPECT_EQ(k, tree.getClosestCommonAncestor(k, i));
  EXPECT_EQ(k, tree.getClosestCommonAncestor(i, k));
}


std::string operator ! (std::string a){
  return "!" + a;
}

TEST(Tree, walkPath) {
  IntTree tree;
  tree.add(i, j, ij);
  tree.add(j, k, jk);
  tree.add(a, j, aj);
  tree.add(b, k, bk);
  tree.init();

  typedef std::vector<std::string> PLS;
  PLS payloads;
  auto storePailoads = [&](std::string payload, const bool inverse){
    if(inverse)
      payload = ! payload;
    payloads.push_back(payload);
  };


  payloads.clear();
  tree.walkPath(a, a, storePailoads);
  EXPECT_EQ(PLS{}, payloads);
  payloads.clear();

  tree.walkPath(i, j, storePailoads);
  EXPECT_EQ(PLS{ij}, payloads);
  payloads.clear();

  tree.walkPath(i, k, storePailoads);
  EXPECT_EQ((PLS{ij, jk}), payloads);
  payloads.clear();

  tree.walkPath(j, k, storePailoads);
  EXPECT_EQ(PLS{jk}, payloads);
  payloads.clear();

  tree.walkPath(j, i, storePailoads);
  EXPECT_EQ(PLS{!ij}, payloads);
  payloads.clear();

  tree.walkPath(k, j, storePailoads);
  EXPECT_EQ(PLS{!jk}, payloads);
  payloads.clear();

  tree.walkPath(k, i, storePailoads);
  EXPECT_EQ((PLS{!jk, !ij}), payloads);
  payloads.clear();

  tree.walkPath(b, i, storePailoads);
  EXPECT_EQ((PLS{bk, !jk, !ij}), payloads);
  payloads.clear();

  tree.walkPath(a, i, storePailoads);
  EXPECT_EQ((PLS{aj, !ij}), payloads);
  payloads.clear();

  tree.walkPath(i, a, storePailoads);
  EXPECT_EQ((PLS{ij, !aj}), payloads);
  payloads.clear();
}
