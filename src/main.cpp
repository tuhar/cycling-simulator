#include <iostream>
#include <simulator/CyclingSimulator.hpp>

int main() {
  CyclingSimulator simulator;

  try {
    simulator.init();
    simulator.run();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
