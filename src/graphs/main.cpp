#include "Graph.hpp"

struct City {
    std::string name;
};

int main() {
    // Graph s defaultnim ArcWeight = double
    graph::Graph<City> g;

    auto zg = g.addNode({"Zagreb"});
    auto ri = g.addNode({"Rijeka"});
    auto sp = g.addNode({"Split"});

    // Izravno prosljeđivanje brojeva bez potrebe za ArcProps strukturom
    g.addConnection(zg, ri, 160.0);
    g.addConnection(ri, sp, 350.0);

    // Dohvaćanje matrice bez pisanja ikakvih lambda funkcija!
    auto adj = g.getAdjacencyMatrix();

    // Pronalaženje puta bez definiranja ekstra weight_fn-a
    auto path = g.findShortestPath(zg, sp);

    std::cout << "Path: " << path.total_distance << std::endl;

    return 0;
}