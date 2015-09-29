INCDIRS:=/Users/dmielke/Documents/oopl/trees/googletest/googletest/include
LIBDIRS:=/Users/dmielke/Documents/oopl/trees/googletest/googletest/make

FILES :=                              \
    .travis.yml                       \
    twobuttons-tests/EID-RunTwoButtons.in   \
    twobuttons-tests/EID-RunTwoButtons.out  \
    twobuttons-tests/EID-TestTwoButtons.c++ \
    twobuttons-tests/EID-TestTwoButtons.out \
    TwoButtons.c++                       \
    TwoButtons.h                         \
    TwoButtons.log                       \
    html                              \
    RunTwoButtons.c++                    \
    RunTwoButtons.in                     \
    RunTwoButtons.out                    \
    TestTwoButtons.c++                   \
    TestTwoButtons.out                   \
    ColalatzBundle.c++

# Call gcc and gcov differently on Darwin
ifeq ($(shell uname), Darwin)
  CXX      := g++
  GCOV     := gcov
  VALGRIND := echo Valgrind not available on Darwin
else
  CXX      := g++-4.8
  GCOV     := gcov-4.8
  VALGRIND := valgrind
endif

CXXFLAGS   := -pedantic -std=c++11 -Wall -I$(INCDIRS)
LDFLAGS    := -lgtest -lgtest_main -pthread -L$(LIBDIRS)
GCOVFLAGS  := -fprofile-arcs -ftest-coverage
GPROF      := gprof
GPROFFLAGS := -pg

clean:
	rm -f *.gcda
	rm -f *.gcno
	rm -f *.gcov
	rm -f RunTwoButtons
	rm -f RunTwoButtons.tmp
	rm -f TestTwoButtons
	rm -f TestTwoButtons.tmp
	rm -f TwoButtonsBundle

config:
	git config -l

bundle:
	cat TwoButtons.h TwoButtons.c++ RunTwoButtons.c++ | sed -e "s/#include \"TwoButtons.h\"//g" > TwoButtonsBundle.c++
	$(CXX) $(CXXFLAGS) $(GCOVFLAGS) TwoButtonsBundle.c++ -o TwoButtonsBundle

scrub:
	make  clean
	rm -f  TwoButtons.log
	rm -rf twobuttons-tests
	rm -rf html
	rm -rf latex

status:
	make clean
	@echo
	git branch
	git remote -v
	git status

test: RunTwoButtons.tmp TestTwoButtons.tmp

RunTwoButtons: TwoButtons.h TwoButtons.c++ RunTwoButtons.c++
	$(CXX) $(CXXFLAGS) $(GCOVFLAGS) TwoButtons.c++ RunTwoButtons.c++ -o RunTwoButtons

RunTwoButtons.tmp: RunTwoButtons
	./RunTwoButtons < RunTwoButtons.in > RunTwoButtons.tmp
	diff RunTwoButtons.tmp RunTwoButtons.out

TestTwoButtons: TwoButtons.h TwoButtons.c++ TestTwoButtons.c++
	$(CXX) $(CXXFLAGS) $(GCOVFLAGS) TwoButtons.c++ TestTwoButtons.c++ -o TestTwoButtons $(LDFLAGS)

TestTwoButtons.tmp: TestTwoButtons
	./TestTwoButtons                                                   >  TestTwoButtons.tmp 2>&1
	$(VALGRIND) ./TestTwoButtons                                       >> TestTwoButtons.tmp
	$(GCOV) -b TwoButtons.c++     | grep -A 5 "File 'TwoButtons.c++'"     >> TestTwoButtons.tmp
	$(GCOV) -b TestTwoButtons.c++ | grep -A 5 "File 'TestTwoButtons.c++'" >> TestTwoButtons.tmp
	cat TestTwoButtons.tmp
