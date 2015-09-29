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

#include "TwoButtons.h"

using namespace std;

typedef std::pair<int, int> TBEntry;
typedef std::queue<TBEntry> TBQueue;

const bool enable_greater_opt = true;
const bool enable_visit_opt = true;
const int max_input_val = 10000;

// ------------
// twobuttons_read
// ------------

std::pair<int, int> twobuttons_read (const std::string& s) {
  std::istringstream sin(s);
  int n = 0;
  int m = 0;
  sin >> n >> m;
  return std::make_pair(n, m);
}

// ------------
// twobuttons_eval
// ------------


class VertexLength
{
public:
  VertexLength (int vertex_, int length_) :
    vertex (vertex_),
    length (length_)
  {}

private:
  int vertex = 0;
  int length = 0;
};

typedef std::vector<std::list<VertexLength>> Graph;

class QueueEntry
{
public:
  int dest_vertex = 0;
  int total_length = 0;
  std::list<int> path;
};

typedef std::queue<QueueEntry> Queue;

class VisitEntry
{
  bool visited = false;
  int total_length = 0;
}

int dijkstra_eval (const Graph& graph)
{
  // Create the queue and seed it with the initial value
  Queue q {1, 0, std::list<int> {1}};
  
  // Create visit vector
  std::vector<VisitEntry> v (graph.size());

  // Do the search
  while (!q.empty()) {

    // Get the entry from the head of the queue
    const QueueEntry entry = q.front();
    q.pop();
    
    const std::list<VertexEntry>& vlist = graph[entry.dest_vertex];
    
    for (const VertexEntry& itr : vlist) {
      
    }

  assert (false);
  return 0;
}

// -------------
// twobuttons_print
// -------------

void twobuttons_print (ostream& w, const int answer) {
  w << answer << std::endl;
}

// -------------
// twobuttons_solve
// -------------

void twobuttons_solve (istream& r, ostream& w) {
  std::string s;

  int num_vertices = 0;
  int num_edges = 0;
  if (getline(r, s))
  {
    std::istringstream sin(s);
    sin >> num_vertices >> num_edges;
  }

  Graph graph (num_vertices);
  while (getline(r, s)) 
  {
    std::istringstream sin(s);
    int start_vertex = 0;
    int end_vertex = 0;
    int length = 0;
    sin >> start_vertex >> end_vertex >> length;
    graph[start_vertex].emplace(end_vertex, length);
  }

  const int answer = dijkstra_eval(graph);
  twobuttons_print(w, answer);
}
