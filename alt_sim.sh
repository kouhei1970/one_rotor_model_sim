g++-12 one_rotor_model_sim.cpp pid.cpp alt_kalman.cpp -lm
./a.out > log/data00.log
gnuplot "alt_sim.plt"
