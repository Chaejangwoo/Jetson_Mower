#include <iostream>

#include "mower_map/grid_map.hpp"

int main() {
  mower_map::GridMap map(9, 9, 0.20, {37.0, 127.0});
  map.placeMower({4, 4});
  std::cout << map.render();
}
