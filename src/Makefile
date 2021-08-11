ODIR=obj

IDIR=../include
IFRANKA =/home/${USER}/libfranka/include
IEIGEN =/usr/include/eigen3

LFRANKA=/home/${USER}/libfranka/build

CC=g++
CXXFLAGS=-std=c++14 -I$(IDIR) -I$(IFRANKA) -I$(IEIGEN) -L$(LFRANKA)

LIBS=-lfranka

_DEPS = position_controller.h parse_confs.h position_control_utils.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = panda_test.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CXXFLAGS)

panda_test: $(OBJ)
	$(CC) -o $@ $^ $(CXXFLAGS) $(LIBS)

_OBJ = calibration.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

calibration: $(OBJ)
	$(CC) -o $@ $^ $(CXXFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 
