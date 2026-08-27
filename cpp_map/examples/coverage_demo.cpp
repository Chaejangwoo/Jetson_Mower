#include <iostream>

#include "mower_map/grid_map.hpp"

int main() {
  mower_map::GridMap map(100, 100, 0.20, {37.0, 127.0});

  // Example obstacles supplied by a future map/GPS obstacle input.
  for (std::size_t row = 35; row < 51; ++row) {
    for (std::size_t column = 42; column < 58; ++column) {
      map.setObstacle({row, column});
    }
  }

  for (const auto cell : map.coveragePath()) {
    std::cout << cell.row << ',' << cell.column << '\n';
  }
}
