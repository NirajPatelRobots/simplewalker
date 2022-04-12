eigen = -I ~/eigen-3.4.0
incLocal = -I./communication -I./state_estimation
state_depnd = ./state_estimation/state_estimation.cpp ./state_estimation/robot_state.cpp

localize_test: ./state_estimation/test_state_estimation.cpp $(state_depnd)
	g++ $(eigen) -I/state_estimation -o ./build/$@ $^ -std=c++17 -Wall -Wextra

simplewalker: simplewalker.cpp communication/maincomp_comm.cpp $(state_depnd)
	g++ $(eigen) $(incLocal)  -o ./build/$@ $^ -lwiringPi -lpthread -std=c++17 -Wall -Wextra
