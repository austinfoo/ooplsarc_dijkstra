
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "TwoButtons.h"

using namespace std;

// -----------
// TestTwoButtons
// -----------

// ----
// read
// ----

TEST(TwoButtonsFixture, read) {
    string s ("5 10\n");
    const std::pair<int, int> val = twobuttons_read(s);
    EXPECT_EQ(5, val.first);
    EXPECT_EQ(10, val.second);}

// ----
// eval
// ----

TEST(TwoButtonsFixture, eval_1) {
  const int val = twobuttons_eval(4, 6);
    EXPECT_EQ(2, val);}

TEST(TwoButtonsFixture, eval_2) {
  const int val = twobuttons_eval(10, 1);
    EXPECT_EQ(9, val);}

// -----
// print
// -----

TEST(TwoButtonsFixture, print) {
    ostringstream val;
    twobuttons_print(val, 9);
    EXPECT_EQ("9\n", val.str());}

// -----
// solve
// -----

TEST(TwoButtonsFixture, solve) {
    istringstream r("4 6\n10 1\n");
    ostringstream w;
    twobuttons_solve(r, w);
    EXPECT_EQ("2\n9\n", w.str());}

