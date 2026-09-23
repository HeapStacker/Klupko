#include "Graph.hpp"

#include <iostream>
#include <string>

struct City {
    std::string name;
};

int main() {
    graph::Graph<City> g;

    auto zg = g.addNode({"Zagreb"});
    auto ri = g.addNode({"Rijeka"});
    auto sp = g.addNode({"Split"});

    g.addConnection(zg, ri, 160.0);
    g.addConnection(ri, sp, 350.0);

    auto path = g.findShortestPath(zg, sp);
    std::cout << "Path: " << path.total_weight << "\n";
    return 0;
}
