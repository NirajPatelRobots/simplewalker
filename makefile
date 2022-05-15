eigen = -I ~/eigen-3.4.0
rapidxml = -I ~/rapidxml-1.13
incLocal = -I./mainsrc -I./communication -I./state_estimation
build = ./build/
CFLAGS = -std=c++17 -Wall -Wextra -O
objnames = simplewalker.o maincomp_comm.o state_estimation.o robot_state.o sensorBoss.o logger.o
objects := $(addprefix $(build),$(objnames))
	
simplewalker: $(objects)
	g++ $(incLocal) $(rapidxml) -o $(build)$@ $^ -lwiringPi -lpthread

localize_test: $(build)*state_estimation.o $(build)robot_state.o
	g++ -I/state_estimation $(eigen) -o $(build)$@ $^

$(build)%.o : ./*/%.cpp
	g++ -c $(incLocal) $(eigen) $(rapidxml) $^  $(CFLAGS) -o $@

clean : 
	rm -f $(build)*.o $(build)simplewalker $(build)localize_test
