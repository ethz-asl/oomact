#ifndef H45524307_F3B1_4796_B68A_9C62139560F1
#define H45524307_F3B1_4796_B68A_9C62139560F1
#include <algorithm>
#include <stdexcept>

#include <iostream>

#include <glog/logging.h>

namespace aslam {
namespace calibration {


template <typename N, typename EP>
class Tree {
 public:
  class TreeFailure : public std::runtime_error {
    using std::runtime_error::runtime_error;
  };

  struct Edge;

  struct Node {
    Node(N node) : node(node){}
    N node;
    Node * parent = nullptr;
    Edge * toParentEdge = nullptr;
  };

  struct Edge {
    N from, to;
    EP payload;
  };

  void add(N from, N to, EP payload){
    CHECK(!isInitialized()) << "Adding after init is not supported!";
    edges.emplace_back(Edge{from, to, payload});
  }

  void init() {
    nodes.reserve(edges.size() * 2); // prevent reallocation which otherwise would ruin the addresses.
    for(auto & e : edges){
      auto toNode = getNode(e.to, true);
      auto fromNode = getNode(e.from, true);
      CHECK(toNode);
      CHECK(fromNode);
      CHECK(fromNode->parent == nullptr) << "This is not a tree because" << fromNode << " has multiple parents!";
      fromNode->parent = toNode;
      fromNode->toParentEdge = &e;
    }

    auto isRoot = [&](const Node & n){
      return n.parent == nullptr;
    };
    CHECK(std::count_if(nodes.begin(), nodes.end(), isRoot) == 1) << "This is not a tree!";
    root = &(* std::find_if(nodes.begin(), nodes.end(), isRoot));
  }

  N getClosestCommonAncestor(N from, N to){
    CHECK(isInitialized()) << "Did you forget to call init() after adding?";
    if(from == to) return from;
    auto pathFromToRoot = getPathToRoot(getNode(from));
    auto pathToToRoot = getPathToRoot(getNode(to));
    int fi = pathFromToRoot.size() - 1;
    for(auto it = pathToToRoot.rbegin(); it != pathToToRoot.rend(); it++){
      if(fi < 0 || pathFromToRoot[fi] != *it){
        if(fi >= 0) CHECK((*it)->to == pathFromToRoot[fi]->to) << "No common ancestor found!";
        return (*it)->to;
      }
      fi--;
    }
    CHECK(fi >= 0);
    return pathFromToRoot[fi]->to;
  }

  template <typename F>
  void walkPath(N from, N to, F f){
    CHECK(isInitialized()) << "Did you forget to call init() after adding?";
    if(from == to) return;
    auto pathFromToRoot = getPathToRoot(getNode(from));
    auto pathToToRoot = getPathToRoot(getNode(to));
    int fi = pathFromToRoot.size() - 1;
    bool foundClosestCommonAncestor = false;

    auto goForward = [&](){
      for(int i = 0; i <= fi; i ++){
        f(pathFromToRoot[i]->payload, false);
      }
    };
    for(auto it = pathToToRoot.rbegin(); it != pathToToRoot.rend(); it++){
      if(foundClosestCommonAncestor){
        f((*it)->payload, true);
      } else if(fi < 0 || pathFromToRoot[fi] != *it){
        goForward();
        f((*it)->payload, true);
        foundClosestCommonAncestor = true;
      }
      fi--;
    }
    if(!foundClosestCommonAncestor){
      goForward();
    }
  }

  friend std::ostream & operator <<(std::ostream & o, const Tree & t){
    for(const Node & n : t.nodes){
      if (n.parent) {
        CHECK(n.toParentEdge);
        o << "(" << n.node << "->" << n.toParentEdge->payload << "->" << n.parent->node << ")";
      } else {
        o << "(ROOT:" << n.node <<")";
      }
    }
    return o;
  }

  bool isInitialized() {
    return root != nullptr;
  }
 private:
  Node * getNode(N node, bool create = false){
    auto it = std::find_if(nodes.begin(), nodes.end(), [&](const Node & n){
      return n.node == node;
    });
    if(it == nodes.end()){
      if(!create){
        LOG(FATAL) << node << " not in the tree!";
      }
      nodes.emplace_back(Node{node});
      return &nodes.back();
    } else {
      return &(*it);
    }
  }

  std::vector<Edge*> getPathToRoot(const Node * np) {
    std::vector<Edge*> ret;
    while(np->parent){
      ret.emplace_back(np->toParentEdge);
      np = np->parent;
    }
    return ret;
  }

  std::vector<Edge> edges;
  std::vector<Node> nodes;
  Node * root = nullptr;
};

}
}

#endif /* H45524307_F3B1_4796_B68A_9C62139560F1 */
