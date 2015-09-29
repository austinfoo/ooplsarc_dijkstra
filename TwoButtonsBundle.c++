
#ifndef TwoButtons_h
#define TwoButtons_h

// --------
// includes
// --------

#include <iostream>
#include <string>
#include <utility>

using namespace std;

// ------------
// twobuttons_read
// ------------

/**
 * read a string
 * @param s a string
 * @return the input string
 */
std::pair<int, int> twobuttons_read (const string& s);

// ------------
// twobuttons_eval
// ------------

/**
 * @param s the input string
 * @return the answer
 */
int twobuttons_eval (const int n, const int m);

// -------------
// twobuttons_print
// -------------

/**
 * print three ints
 * @param w an ostream
 * @param out the answer
 */
void twobuttons_print (ostream& w, const int answer);

// -------------
// twobuttons_solve
// -------------

/**
 * @param r an istream
 * @param w an ostream
 */
void twobuttons_solve (istream& r, ostream& w);

#endif
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

int twobuttons_eval (int n, int m)
{
  // Create the queue and seed it with the initial value
  TBQueue q;
  q.push(std::make_pair(n, 0));
  
  // Create visit vector
  std::vector<bool> v;
  v.assign(2*max_input_val, false);

  while (!q.empty()) {

    // Get the entry from the head of the queue
    const TBEntry entry = q.front();
    q.pop();
    
    // Check for termination condition
    if (entry.first == m) {
      return entry.second;
    }

    // The red button doubles the value
    // If the greater optimizaiton is enabled, prune the node if the value is greater than m
    if (!enable_greater_opt || entry.first <= m ) {
      int red_val = 2 * entry.first;
      // If the visit optimization is enabled, prune the node if we have seen the value before 
      if (!enable_visit_opt || !v[red_val]) {
	q.push(std::make_pair(red_val, entry.second + 1));
	if (enable_visit_opt) v[red_val] = true;
      }
    }
    
    // The blue button subtracts one from the value
    int blue_val = entry.first - 1;
    // Prune the node if the value is negative
    if (blue_val >= 0) {
      if (!enable_visit_opt || !v[blue_val]) {
	q.push(std::make_pair(blue_val, entry.second  + 1));
	if (enable_visit_opt) v[blue_val] = true;
      }
    }
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
  while (getline(r, s)) {
    const std::pair<int, int> val = twobuttons_read(s);
    const int answer = twobuttons_eval(val.first, val.second);
    twobuttons_print(w, answer);
  }
}

#include <iostream>



// ----
// main
// ----

int main () {
    using namespace std;
    twobuttons_solve(cin, cout);
    return 0;
}
