eigen = -I ~/eigen-3.4.0
incLocal = -I./mainsrc -I./communication -I./state_estimation
build = ./build/
CFLAGS = -std=c++17 -Wall -Wextra -O
objnames = simplewalker.o maincomp_comm.o state_estimation.o robot_state.o sensorBoss.o logger.o
objects := $(addprefix $(build),$(objnames))
	
simplewalker: $(objects)
	g++ $(incLocal) -o $(build)$@ $^ -lwiringPi -lpthread

localize_test: $(build)*state_estimation.o $(build)robot_state.o
	g++ -I/state_estimation $(eigen) -o $(build)$@ $^

$(build)%.o : ./*/%.cpp
	g++ -c $(incLocal) $(eigen) $^  $(CFLAGS) -o $@

clean : 
	rm -f $(build)*.o $(build)simplewalker $(build)localize_test
