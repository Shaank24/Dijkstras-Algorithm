// graph.h <Starter Code>
// <Shaan Kohli>
//
// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
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

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <unordered_map>

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
  private:
    // each vertex maps to vector of pairs
    // each pair contains neighbor vertex and weight
    unordered_map<VertexT, vector<pair<VertexT, WeightT>>> AdjList;


  public:
    // constructor:
    // Constructs an empty graph where n is the max # of vertices
    // you expect the graph to contain.
    graph() {}

    // destructor to clean up dynamically allocated memory
    ~graph() {
      clear();
    }

    // clears the unordered map to remove all vertices-edges pairings from adjacency list
    void clear() {
      AdjList.clear();
    }

    // copy constructor
    // initializes new graph with adjacency list of the other graph.
    // copies over each pair of vertices-edges 
    graph(const graph<VertexT, WeightT>& other) {
      AdjList = other.AdjList;
    }

    // copy assingment operator
    // allows for adjacency lists to be set equal to one another
    graph<VertexT, WeightT>& operator=(const graph<VertexT, WeightT>& other) {
      // makes sure this object and the other are not already the same
      if (this != &other) {
        AdjList = other.AdjList;
      }

      return *this;
    }

    // NumVertices
    // Returns the # of vertices currently in the graph.
    int NumVertices() const {
      return AdjList.size();
    }

    // NumEdges
    // Returns the # of edges currently in the graph.
    int NumEdges() const {
      int count = 0;

      // loop through each pair in the adjacency list and counts how many
      // edges currently exist:
      for (const auto& pair : AdjList) {
        count += pair.second.size();
      }
      return count;
    }

    // addVertex
    // Adds the vertex v to the graph if there's room, and if so
    // returns true.  If the graph is full, or the vertex already
    // exists in the graph, then false is returned.
    // adds new vertex to adjacency list and pairs the vertex v with empty list of edges
    bool addVertex(VertexT v) {
      
      return AdjList.insert({v, {}}).second;
    }

    // addEdge
    // Adds the edge (from, to, weight) to the graph, and returns
    // true.  If the vertices do not exist or for some reason the
    // graph is full, false is returned.
    // NOTE: if the edge already exists, the existing edge weight
    // is overwritten with the new edge weight.
    bool addEdge(VertexT from, VertexT to, WeightT weight) {
      // makes sure both vertces from and to exist in graph and returns false if either vertex does not exist
      if (AdjList.find(from) == AdjList.end() || AdjList.find(to) == AdjList.end()) {
        return false;
      }

      // checks if the edge already exists and updates weight
      auto &edges = AdjList[from];
      for (auto& edge : edges) {
        if (edge.first == to) {
          edge.second = weight;
          return true;
        }
      }

      // if the edge doesn't exist, add it
      edges.emplace_back(to, weight);
      return true;
    }

    // getWeight
    // Returns the weight associated with a given edge.  If
    // the edge exists, the weight is returned via the reference
    // parameter and true is returned.  If the edge does not
    // exist, the weight parameter is unchanged and false is
    // returned.
    bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
      // finds the from vertex in adjacency list
      auto currFrom = AdjList.find(from);

      // makes sure from vertex is in graph
      if (currFrom != AdjList.end()) {

        // iterates over al edges of from vertex
        for (const auto& edge : currFrom->second) {
          // checks if current edge connects to the to vertex
          if (edge.first == to) {
            // sets weight to weight of the current edge
            weight = edge.second;
            return true;
          }
        }
      }

      // if from vertex is not found or no edge to the to vertex exists
      return false;
    }

    // neighbors
    // Returns a set containing the neighbors of v, i.e. all
    // vertices that can be reached from v along one edge.
    // Since a set is returned, the neighbors are returned in
    // sorted order; use foreach to iterate through the set.
    set<VertexT> neighbors(VertexT v) const {
      set<VertexT> neighborsSet;

      // finds the vertex v in the adjacency list
      auto vFound = AdjList.find(v);

      // checks if v is found in the graph
      if (vFound != AdjList.end()) {

        // iterates over all edges of v
        // for pair, first is the neighbor vertex and second is weight
        for (const auto& edge : vFound->second) {
          // inserts neighbor vertex into set
          neighborsSet.insert(edge.first);
        }
      }

      return neighborsSet;

    }

    // getVertices
    // Returns a vector containing all the vertices currently in
    // the graph.
    vector<VertexT> getVertices() const {
      vector<VertexT> vertices;

      // iterates over adjacency list where first is a vertex and second is the vector of its edges
      for (const auto& pair : AdjList) {
        // adds vertex to vector of vertices
        vertices.push_back(pair.first);
      }

      // returns vector containing all vertices in the graph
      return vertices;
    }

    // dump
    // Dumps the internal state of the graph for debugging purposes.
    // Example:
    //    graph<string,int>  G(26);
    //    ...
    //    G.dump(cout);  // dump to console
    void dump(ostream& output) const {
      // gets all vertices and sorts them
      vector<VertexT> sortedVertices = getVertices();
      sort(sortedVertices.begin(), sortedVertices.end());

      output << "***************************************************" << endl;
      output << "********************* GRAPH ***********************" << endl;

      // outputs number of vertices and edges in graph
      output << "**Num vertices: " << NumVertices() << endl;
      output << "**Num edges: " << NumEdges() << endl;

      // outputs list of vertices with the vertice's index
      output << endl << "**Vertices:" << endl;
      for (int i = 0; i < sortedVertices.size(); i++) {
        output << " " << i << ". " << sortedVertices[i] << endl;
      }

      // outputs edges
      output << endl << "**Edges:" << endl;

      // iterates over each vertex in the graph
      for (const auto& from : sortedVertices) {
        output << " row " << from << ": ";

        // for each vertex, iterate over all vertices to check for edges
        for (const auto& to : sortedVertices) {
          int weight;
          // checks if there is an edge from "from" to "to" and it is not a self loop
          if (from != to && getWeight(from, to, weight)) {
            output << "(T, " << weight << ") ";
          }
          // no edge exists
          else {
            output << "F ";
          }
        }
        output << endl;
      }

      output << "**************************************************" << endl;
    }
};

