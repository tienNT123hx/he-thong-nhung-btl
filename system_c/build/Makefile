SYSTEMC_HOME = C:/systemc-3.0.1/systemc-3.0.1
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall \
    -I$(SYSTEMC_HOME)/include \
    -I../

LDFLAGS = -L$(SYSTEMC_HOME)/lib -lsystemc

SRC = ../top/sc_main.cpp
OUT = systemc_sim

all:
	$(CXX) $(CXXFLAGS) $(SRC) -o $(OUT) $(LDFLAGS)

clean:
	rm -f $(OUT)
