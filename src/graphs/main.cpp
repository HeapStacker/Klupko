#include <cmath>
#include <iostream>
#include <string>


// Raylib zamjenjuje GLFW, OpenGL, ImGui i ImNodes
#include <raylib.h>

#include <lemon/list_graph.h>
#include <lemon/smart_graph.h>
#include <lemon/static_graph.h>

using namespace lemon;

// --- STRUKTURA PODATAKA (ZADRŽAN LEMON GRAPH) ---

struct DeliveryNetwork {
    ListDigraph graph;

    ListDigraph::NodeMap<std::string> nodeName;
    ListDigraph::NodeMap<Vector2> nodePos; // Koristimo Raylib-ov Vector2 umjesto ImVec2

    ListDigraph::ArcMap<double> arcWeight;
    ListDigraph::ArcMap<bool> isSymmetric;

    DeliveryNetwork()
        : nodeName(graph),
          nodePos(graph),
          arcWeight(graph),
          isSymmetric(graph) {}

    ListDigraph::Node addLocation(const std::string &name, Vector2 pos) {
        ListDigraph::Node u = graph.addNode();
        nodeName[u] = name;
        nodePos[u] = pos;
        return u;
    }

    ListDigraph::Arc addOneWayRoad(ListDigraph::Node u, ListDigraph::Node v, double weight) {
        ListDigraph::Arc a = graph.addArc(u, v);
        arcWeight[a] = weight;
        isSymmetric[a] = false;
        return a;
    }

    std::pair<ListDigraph::Arc, ListDigraph::Arc> addTwoWayRoad(ListDigraph::Node u, ListDigraph::Node v, double weight) {
        ListDigraph::Arc a1 = graph.addArc(u, v);
        ListDigraph::Arc a2 = graph.addArc(v, u);

        arcWeight[a1] = weight;
        arcWeight[a2] = weight;

        isSymmetric[a1] = true;
        isSymmetric[a2] = true;

        return {a1, a2};
    }
};

namespace StaticMapModule {

    struct HighwayStaticNetwork {
        StaticDigraph graph;
        StaticDigraph::NodeMap<std::string> name;
        StaticDigraph::ArcMap<double> speedLimit;

        HighwayStaticNetwork()
            : name(graph), speedLimit(graph) {}
    };

    void buildHighwayNetwork() {
        SmartDigraph smartGraph;
        SmartDigraph::NodeMap<std::string> smartName(smartGraph);

        SmartDigraph::Node zagreb = smartGraph.addNode();
        SmartDigraph::Node split = smartGraph.addNode();
        smartName[zagreb] = "Zagreb (Autocesta)";
        smartName[split] = "Split (Autocesta)";

        SmartDigraph::Arc A1 = smartGraph.addArc(zagreb, split);

        std::cout << "[StaticMapModule] SmartDigraph uspješno inicijaliziran s "
                  << countNodes(smartGraph) << " čvora i "
                  << countArcs(smartGraph) << " luka.\n";
    }
} // namespace StaticMapModule

// --- POMOĆNE FUNKCIJE ZA RENDERING S RAYLIB-OM ---

// Funkcija za crtanje strelice za jednosmjerne ceste
void DrawDirectedArc(Vector2 start, Vector2 end, Color color, float thickness = 3.0f) {
    DrawLineEx(start, end, thickness, color);

    // Izračun kuta usmjerenja
    float angle = atan2f(end.y - start.y, end.x - start.x);
    float arrowLength = 14.0f;
    float arrowAngle = 0.4f; // radijani (~23 stupnja)

    // Odmak od centra čvora kako vrh strelice ne bi bio sakriven ispod kruga čvora
    Vector2 arrowHead = {
        end.x - 28.0f * cosf(angle),
        end.y - 28.0f * sinf(angle)};

    Vector2 p1 = {
        arrowHead.x - arrowLength * cosf(angle - arrowAngle),
        arrowHead.y - arrowLength * sinf(angle - arrowAngle)};
    Vector2 p2 = {
        arrowHead.x - arrowLength * cosf(angle + arrowAngle),
        arrowHead.y - arrowLength * sinf(angle + arrowAngle)};

    DrawTriangle(arrowHead, p1, p2, color);
}

// Renderiranje mreže dostave u Raylib 2D okruženju
void RenderDeliveryNetworkInRaylib(DeliveryNetwork &net) {
    const float nodeRadius = 25.0f;

    // 1. Iscrtavanje Veza / Lukova
    for (ListDigraph::ArcIt a(net.graph); a != INVALID; ++a) {
        ListDigraph::Node u = net.graph.source(a);
        ListDigraph::Node v = net.graph.target(a);

        Vector2 startPos = net.nodePos[u];
        Vector2 endPos = net.nodePos[v];

        if (net.isSymmetric[a]) {
            // Za dvosmjerne ceste crtamo samo jednu ravnu liniju (gdje je u.id < v.id)
            if (net.graph.id(u) < net.graph.id(v)) {
                DrawLineEx(startPos, endPos, 4.0f, GREEN);

                // Prikaz težine/udaljenosti na sredini ceste
                Vector2 midPos = {(startPos.x + endPos.x) / 2.0f, (startPos.y + endPos.y) / 2.0f};
                std::string weightText = std::to_string(net.arcWeight[a]).substr(0, 3) + " km";
                DrawText(weightText.c_str(), (int)midPos.x - 15, (int)midPos.y - 10, 14, RAYWHITE);
            }
        } else {
            // Jednosmjerna cesta (narančasta strelica)
            DrawDirectedArc(startPos, endPos, ORANGE, 3.0f);

            Vector2 midPos = {(startPos.x + endPos.x) / 2.0f, (startPos.y + endPos.y) / 2.0f};
            std::string weightText = std::to_string(net.arcWeight[a]).substr(0, 3) + " km";
            DrawText(weightText.c_str(), (int)midPos.x - 15, (int)midPos.y - 10, 14, YELLOW);
        }
    }

    // 2. Iscrtavanje Čvorova (Lokacija)
    for (ListDigraph::NodeIt n(net.graph); n != INVALID; ++n) {
        Vector2 pos = net.nodePos[n];

        // Crtanje čvora (krug s okvirom)
        DrawCircleV(pos, nodeRadius, DARKBLUE);
        DrawCircleLinesV(pos, nodeRadius, SKYBLUE);

        // Prikaz ID-a čvora u središtu
        std::string idStr = std::to_string(net.graph.id(n));
        DrawText(idStr.c_str(), (int)pos.x - 4, (int)pos.y - 8, 16, WHITE);

        // Prikaz naziva lokacije iznad čvora
        DrawText(net.nodeName[n].c_str(), (int)pos.x - 40, (int)pos.y - 42, 16, LIGHTGRAY);
    }
}

// --- MAIN FUNKCIJA ---

int main() {
    // 1. Inicijalizacija Raylib prozora (zamjenjuje GLFW i OpenGL init)
    const int screenWidth = 1280;
    const int screenHeight = 720;
    InitWindow(screenWidth, screenHeight, "Multi-Slojni Grad (Raylib)");
    SetTargetFPS(60);

    // Testiranje sporednog modula
    StaticMapModule::buildHighwayNetwork();

    // 2. Priprema grafa u memoriji
    DeliveryNetwork cityNetwork;
    auto depot = cityNetwork.addLocation("Centralno Skladiste", {200.0f, 180.0f});
    auto hubA = cityNetwork.addLocation("Dostavni Hub A", {700.0f, 180.0f});
    auto hubB = cityNetwork.addLocation("Dostavni Hub B", {200.0f, 520.0f});
    auto zoneC = cityNetwork.addLocation("Krajnja Zona C", {700.0f, 520.0f});

    cityNetwork.addTwoWayRoad(depot, hubA, 5.2);
    cityNetwork.addOneWayRoad(hubA, zoneC, 2.1);
    cityNetwork.addTwoWayRoad(depot, hubB, 3.0);
    cityNetwork.addOneWayRoad(hubB, zoneC, 4.5);

    // 3. Glavna render-petlja (izvršava se dok se ne pritisne ESC ili zatvori prozor)
    while (!WindowShouldClose()) {

        // --- CRTANJE OKVIRA ---
        BeginDrawing();
        ClearBackground(GetColor(0x181818FF)); // Tamna pozadina

        // Naslov i legenda
        DrawText("Karta Dostavne Mreze", 20, 20, 22, RAYWHITE);
        DrawText("Zeleno = Dvosmjerna cesta | Narancasto = Jednosmjerna cesta", 20, 50, 16, GRAY);

        // Crtanje mreže na ekranu
        RenderDeliveryNetworkInRaylib(cityNetwork);

        EndDrawing();
    }

    // 4. Čišćenje resursa
    CloseWindow();

    return 0;
}