// application.cpp <Starter Code>
// <Shaan Kohli>
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <queue>
#include <cassert>
#include <stack>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;

// REDEFINES INFINITY
const double INF = numeric_limits<double>::max();

// NEEDED FOR DIJKSTRA'S ALGORITHM
class prioritize {
    public:
        bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const {
            return p1.second > p2.second;
        }
};

// helper function to search for buildings based off a given query
// returns the building if it is valid, otherwise returns default constructor
BuildingInfo searchBuilding(const string& query, const vector<BuildingInfo>& buildings) {
    // loops through all of them buildings
    for (const auto& building : buildings) {
        // returns the building if the query matches the abbreviation
        // this is for an exact match
        if (building.Abbrev == query) {
            return building;
        }
    }

    // if no exact match is found, this searches for a partial match in names
    for (const auto& building : buildings) {
        if (building.Fullname.find(query) != string::npos) {
            return building;
        }
    }

    // if no match is found, returns empty BuildingInfo object
    return BuildingInfo();
}

// finds and returns the nearest building to the midpoint coordinate found
BuildingInfo findNearestBuilding(const Coordinates& midpoint, const vector<BuildingInfo>& buildings) {
    BuildingInfo nearestBuilding;
    double minDist = INF;

    // loops through each building
    for (const auto& building : buildings) {

        // gets the distance between the midpoint and each building
        double dist = distBetween2Points(midpoint.Lat, midpoint.Lon, building.Coords.Lat, building.Coords.Lon);

        // finds the building with the shortest distance
        if (dist < minDist) {
            minDist = dist;
            nearestBuilding = building;
        }
    }

    // returns the closest building to the midpoint
    return nearestBuilding;
}

// helper function to find the nearest nodes
Coordinates findNearestNode(const BuildingInfo& building, const map<long long, Coordinates>& Nodes, vector<FootwayInfo> Footways) {
    // initializes the closest node ID and the minimum distance
    Coordinates nearestNodeID;
    double minDist = INF;

    for (const auto& foot : Footways) {
        // loops through each node pair
        for (int i = 0; i < foot.Nodes.size(); i++) 
        {

            // calculates the distance between the building coordinates and each node's coordinates
            double dist = distBetween2Points(building.Coords.Lat, building.Coords.Lon, Nodes.at(foot.Nodes.at(i)).Lat, Nodes.at(foot.Nodes.at(i)).Lon);
            
            // finds the node id with the smallest distance
            if (dist < minDist) {
                minDist = dist;
                nearestNodeID = Nodes.at(foot.Nodes.at(i));
            }
        }
    }

    // returns the id of the nearest node to the building
    return nearestNodeID;
}

// finds the shortest path using dijkstra's algorithm
void dijkstraAlg(long long startV, graph<long long, double>& G, map<long long, double>& distances, map<long long, long long>& predecessors) {
    // stores vertices, the pair composed of a vertex and its distance from startV
    priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisited;
    set<long long> visited;

    // initializes the distances and predecessors for each vertex
    for (const auto& vertex : G.getVertices()) {
        distances[vertex] = INF;
        predecessors[vertex] = 0;
        unvisited.push(make_pair(vertex, INF));
    }

    // sets distance of start vertex to 0 and pushes it into priority queue
    distances[startV] = 0;
    unvisited.push(make_pair(startV, 0));

    // processes vertices until the queue is empty
    while (!unvisited.empty()) {
        // gets the vertex with the minimum distance from the start vertex
        long long currV = unvisited.top().first;
        unvisited.pop();

        // breaks out of the loop if the vertex is unreachable
        if (distances[currV] == INF) {
            break;
        }
        else if (visited.find(currV) != visited.end()) {
            continue;
        }

        // adds currV to visited set
        visited.insert(currV);

        // for each adjacent vertex of the current vertex
        for (const auto& adjV : G.neighbors(currV)) {
            double edgeWeight;

            // gets the weight of the edge from currV to adjV
            if (G.getWeight(currV, adjV, edgeWeight)) {
                double altPathDist = distances[currV] + edgeWeight;

                // gets the minimum distance and pushes the adjacent vertex with the update distance to the priority queue
                if (altPathDist < distances[adjV]) {
                    distances[adjV] = altPathDist;
                    predecessors[adjV] = currV;
                    unvisited.push(make_pair(adjV, altPathDist));
                }
            }
        }
    }
}

// find and returns the path based off the user input
vector<long long> getPath(map<long long, long long>& predecessors, long long endV) {

    stack<long long> reversePath;
    vector<long long> path;

    // start from the end vertex and push the current vertex on the stack and moves to predecessor fo the current vertex
    long long currV = endV;
    while (predecessors[currV] != 0) {
        reversePath.push(currV);
        currV = predecessors[currV];
    }

    reversePath.push(currV);

    // makes sure path is in correct order
    while (!reversePath.empty()) {
        currV = reversePath.top();
        reversePath.pop();
        path.push_back(currV);
    }

    return path;
}

//
// Implement your standard application here
//
void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
    string person1Building, person2Building;

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    while (person1Building != "#") {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);

        // Searches Buildings 1 and 2
        // checks if the buildings are valid
        BuildingInfo building1 = searchBuilding(person1Building, Buildings);
        BuildingInfo building2 = searchBuilding(person2Building, Buildings);
        if (building1.Fullname.empty()) {
            cout << "Person 1's building not found" << endl;
        }
        if (building2.Fullname.empty()) {
            cout << "Person 2's building not found" << endl;
        }

        // initializes the path found to be false and loops until the path is found
        bool pathFound = false;
        
        // loops as long as buildings 1 and 2 are valid and the path is not found
        while (!pathFound && !building1.Fullname.empty() && !building2.Fullname.empty()) {
            // gets the midpoint of the 2 buildings
            Coordinates midpoint = centerBetween2Points(building1.Coords.Lat, building1.Coords.Lon, building2.Coords.Lat, building2.Coords.Lon);
            
            // finds the closest building to the midpoint
            BuildingInfo centerBuilding = findNearestBuilding(midpoint, Buildings);

            // prints out all information on person 1 and person 2 and their destination building
            cout << endl << "Person 1's point:" << endl;
            cout << " " << building1.Fullname << endl << " (" << building1.Coords.Lat << ", " << building1.Coords.Lon << ")" << endl;
            cout << "Person 2's point:" << endl;
            cout << " " << building2.Fullname << endl << " (" << building2.Coords.Lat << ", " << building2.Coords.Lon << ")" << endl;
            cout << "Destination Building:" << endl;
            cout << " " << centerBuilding.Fullname << endl << " (" << centerBuilding.Coords.Lat << ", " << centerBuilding.Coords.Lon << ")" << endl;

            // fins Nearest Nodes from buildings 1, 2 & Center
            Coordinates node1 = findNearestNode(building1, Nodes, Footways);
            Coordinates node2 = findNearestNode(building2, Nodes, Footways);
            Coordinates node3 = findNearestNode(centerBuilding, Nodes, Footways);

            // prints out all informatioin on the 3 nodes
            cout << endl << "Nearest P1 node:" << endl;
            cout << " " << node1.ID << endl << " (" << node1.Lat << ", " << node1.Lon << ")" << endl;
            cout << "Nearest P2 node:" << endl;
            cout << " " << node2.ID << endl << " (" << node2.Lat << ", " << node2.Lon << ")" << endl;
            cout << "Nearest destination node:" << endl;
            cout << " " << node3 .ID << endl << " (" << node3.Lat << ", " << node3.Lon << ")" << endl;
            
            // creates the distance and predecessor maps for node1
            map<long long, double> distances1;
            map<long long, long long> predecessors1;

            // calls dijkstra's algorithm and the getPath function for node1 to node3
            dijkstraAlg(node1.ID, G, distances1, predecessors1);
            vector<long long> path1 = getPath(predecessors1, node3.ID);
            
            // checks to make sure that distance is reachable from node1 to node3
            if (distances1[node2.ID] >= INF) {
                cout << "Sorry, destination unreachable." << endl;
                break;
            }

            // prints out path and distance from node1 to node3
            cout << "Person 1's distance to dest: " << distances1[node3.ID] << " miles" << endl;
            cout << "Path: ";
            for (int i = 0; i < path1.size(); i++) {
                cout << path1[i];
                if (i < path1.size() - 1)
                {
                    cout << "->";
                }
            }
            cout << endl << endl;

            // creates the distance and predecessor maps for node2
            map<long long, double> distances2;
            map<long long, long long> predecessors2;

            // calls dijkstra's algorithm and the getPath function for node2 to node3
            dijkstraAlg(node2.ID, G, distances2, predecessors2);
            vector<long long> path2 = getPath(predecessors2, node3.ID);

            // checks if one path is invalid
            if (distances1[node3.ID] >= INF || distances2[node3.ID] >= INF) {
                cout << "At least one person was unable to reach the destination building. Finding next cloest building..." << endl;
            }

            // prints out path and distance from node2 to node3
            cout << "Person 2's distance to dest: " << distances2[node3.ID] << " miles" << endl;
            cout << "Path: ";
            for (int i = 0; i < path2.size(); i++) {
                cout << path2[i];
                if (i < path2.size() - 1)
                {
                    cout << "->";
                }
            }
            cout << endl << endl;
            
            pathFound = true;

        }

    //
    // another navigation?
    //
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
    }     
}

int main() {
    graph<long long, double> G;

    // maps a Node ID to it's coordinates (lat, lon)
    map<long long, Coordinates>  Nodes;
    // info about each footway, in no particular order
    vector<FootwayInfo>          Footways;
    // info about each building, in no particular order
    vector<BuildingInfo>         Buildings;
    XMLDocument                  xmldoc;

    cout << "** Navigating UIC open street map **" << endl;
    cout << endl;
    cout << std::setprecision(8);

    string def_filename = "map.osm";
    string filename;

    cout << "Enter map filename> ";
    getline(cin, filename);

    if (filename == "") {
        filename = def_filename;
    }

    //
    // Load XML-based map file
    //
    if (!LoadOpenStreetMap(filename, xmldoc)) {
        cout << "**Error: unable to load open street map." << endl;
        cout << endl;
        return 0;
    }

  //
  // Read the nodes, which are the various known positions on the map:
  //
    int nodeCount = ReadMapNodes(xmldoc, Nodes);

    //
    // Read the footways, which are the walking paths:
    //
    int footwayCount = ReadFootways(xmldoc, Footways);

    //
    // Read the university buildings:
    //
    int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

    //
    // Stats
    //
    assert(nodeCount == (int)Nodes.size());
    assert(footwayCount == (int)Footways.size());
    assert(buildingCount == (int)Buildings.size());

    cout << endl;
    cout << "# of nodes: " << Nodes.size() << endl;
    cout << "# of footways: " << Footways.size() << endl;
    cout << "# of buildings: " << Buildings.size() << endl;

    // adds vertices to the graph
    for (const auto& nodePair : Nodes) {
        // gets the key in the Nodes map which is used as the vertex
        long long nodeID = nodePair.first;

        // adds each vertex to the graph
        G.addVertex(nodeID);
    }

    // adds edges to the graph
    for (const auto& footway : Footways) {
        // each footway has vector of node IDs
        const vector<long long>& nodes = footway.Nodes;

        // adds edge between each pair of consecutive nodes in the footway
        for (size_t i = 0; i < nodes.size() - 1; i++) {
            long long fromNode = nodes[i];
            long long toNode = nodes[i + 1];
            
            // calculates distance between the 2 nodes as the weight of the edge
            double weight = distBetween2Points(Nodes[fromNode].Lat, Nodes[fromNode].Lon, Nodes[toNode].Lat, Nodes[toNode].Lon);

            // adds bidirectional edge to the graph
            G.addEdge(fromNode, toNode, weight);
            G.addEdge(toNode, fromNode, weight);
        }
    }

    cout << "# of vertices: " << G.NumVertices() << endl;
    cout << "# of edges: " << G.NumEdges() << endl;
    
    cout << endl;

    // Execute Application
    application(Nodes, Footways, Buildings, G);

    //
    // done:
    //
    cout << "** Done **" << endl;
    return 0;
}
