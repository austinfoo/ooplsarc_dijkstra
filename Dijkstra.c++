// ----------------------------
// projects/twobuttons/Twobuttons.c++
// Copyright (C) 2015
// Glenn P. Downing
// ----------------------------

// --------
// includes
// --------

#include <cassert>  // assert
#include <iostream> // endl, istream, ostream
#include <sstream>  // istringstream
#include <string>   // getline, string
#include <queue>
#include <utility>

#include "Dijkstra.h"

using namespace std;

// ------------
// dijkstra_eval
// ------------

class VertexLength
{
public:
  VertexLength (int vertex_, int length_) :
    vertex (vertex_),
    length (length_)
  {}

  int vertex = 0;
  int length = 0;
};

class QueueEntry
{
public:
  QueueEntry(int vertex_, int cumm_length_, const VertexList& path_) :
    vertex (vertex_),
    cumm_length (cumm_length_),
    path (path_)
  {}

  int vertex = 0;
  int cumm_length = 0;
  VertexList path;
};

typedef std::queue<QueueEntry> Queue;

class VisitEntry
{
public:
  bool visited = false;
  int cumm_length = 0;
};

class Solution {
public:
  int cumm_length = -1;
  VertexList path;
};

VertexList dijkstra_eval (const Graph& graph, int start_vertex, int end_vertex)
{
  // Create the queue and seed it with the initial value
  Queue q;
  VertexList vl;
  vl.push_back(start_vertex);
  QueueEntry qe (start_vertex, 0, vl);
  q.push(qe);

  // Create visit vector
  std::vector<VisitEntry> v (graph.size());

  // Create something to hold the solution
  Solution solution;

  // Do the search
  while (!q.empty()) {
    
    // Get the entry from the head of the queue
    const QueueEntry qe = q.front();
    q.pop();
    
    // If we found a solution keep it if it is shorter than any previous solution
    if (qe.vertex == end_vertex) 
    {
      if (qe.cumm_length < solution.cumm_length) {
	solution.cumm_length = qe.cumm_length;
	solution.path = qe.path;
      }
    }

    else 
    {
      for (const VertexLength& vl : graph[qe.vertex]) {

	int cumm_length = qe.cumm_length + vl.length;
     
	// If we have seen this vertex before, don't keep it if it is longer than a previous visit
	if (v[vl.vertex].visited && (cumm_length > v[vl.vertex].cumm_length)) {
	  // Throw it away
	}

	else {
	  q.emplace (vl.vertex, cumm_length, qe.path);
	  q.back().path.push_back(vl.vertex);
	  assert (v[vl.vertex].visited == false);
	  v[vl.vertex].visited = true;
	  v[vl.vertex].cumm_length = cumm_length;
	}
      }
    }
  }

  return solution.path;
}

// -------------
// dijkstra_print
// -------------

void dijkstra_print (ostream& w, const VertexList& answer) {
  if (answer.size() == 0) {
    w << "-1" << std::endl;
  } else {
    for (int vertex : answer) {
      w << vertex << " ";
    }
    w << std::endl;
  }
}

// -------------
// dijkstra_solve
// -------------

void dijkstra_solve (istream& r, ostream& w) {
  std::string s;

  int num_vertices = 0;
  int num_edges = 0;
  if (getline(r, s))
  {
    std::istringstream sin(s);
    sin >> num_vertices >> num_edges;
  }

  Graph graph (num_vertices);
  for (int i = 0; i < num_edges; ++i) {
    getline(r, s);
    std::istringstream sin(s);
    int vertex1 = 0;
    int vertex2 = 0;
    int length = 0;
    sin >> vertex1 >> vertex2 >> length;
    graph[vertex1].emplace_back(vertex2, length);
    graph[vertex2].emplace_back(vertex1, length);
  }

  const VertexList answer = dijkstra_eval(graph, 0, num_vertices-1);
  dijkstra_print(w, answer);
}