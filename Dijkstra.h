
#ifndef Dijkstra_h
#define Dijkstra_h

// --------
// includes
// --------

#include <iostream>
#include <string>
#include <utility>
#include <list>
#include <vector>

using namespace std;

class VertexLength;

typedef std::list<VertexLength> VertexLengthList;
typedef std::vector<VertexLengthList> Graph;
typedef std::list<int> VertexList;

// ------------
// dijkstra_eval
// ------------

VertexList dijkstra_eval (const Graph& graph, int start_vertex, int end_vertex);

// -------------
// dijkstra_print
// -------------

void dijkstra_print (ostream& w, const VertexList& answer);

// -------------
// dijkstra_solve
// -------------

void dijkstra_solve (istream& r, ostream& w);

#endif