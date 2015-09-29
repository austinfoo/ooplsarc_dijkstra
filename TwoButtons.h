
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
