eigen = -I ../eigen-3.4.0
rapidxml = -I ../rapidxml
incLocal = -I./mainsrc -I./communication -I./state_estimation -I./test -I./physics -I./display
build = ./build/
CFLAGS = -std=c++17 -Wall -Wextra -O
objnames = state_estimation.o robot_state.o sensorBoss.o logger.o leg_kinematics.o walkerUtils.o
objects := $(addprefix $(build),$(objnames))
	
simplewalker: $(build)simplewalker.o $(build)maincomp_comm.o $(objects)
	g++ $(incLocal) $(rapidxml) -o $(build)$@ $^ -lwiringPi -lpthread

test_localization: $(build)test_localization.o $(objects)
	g++ -I/state_estimation -I/mainsrc -I/physics $(eigen) $^ -o $(build)$@

unittests: $(build)unittests.o $(objects)
	g++ $(incLocal) $^ -o $(build)$@

$(build)%.o : ./*/%.cpp
	g++ -c $(incLocal) $(eigen) $(rapidxml) $^  $(CFLAGS) -o $@

clean : 
	rm -f $(build)*.o $(build)simplewalker $(build)test_localization $(build)unittests
