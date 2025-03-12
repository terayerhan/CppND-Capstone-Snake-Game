#pragma once

#include <utility>
#include "sdl_point_operators.h"  

class Grid {
 public:
  Grid(std::size_t width, std::size_t height) : _width(width), _height(height) {
    // Initialize grid width for hash function
    SDLPointHash::grid_width = width;
  }

  inline std::size_t GetWidth() const { return _width; }
  inline std::size_t GetHeight() const { return _height; }

  inline std::pair<float, float> WrapPosition(float x, float y) const {
    // Wraps coordinates around the grid boundaries (toroidal wrapping).
    if (x < 0) x += _width;
    if (x >= _width) x -= _width;
    if (y < 0) y += _height;
    if (y >= _height) y -= _height;
    return {x, y};
  }

  inline std::pair<int, int> WrapPosition(int x, int y) const {
    if (x < 0) x += _width;
    if (x >= _width) x -= _width;
    if (y < 0) y += _height;
    if (y >= _height) y -= _height;
    return {x, y};
  }

 private:    
  std::size_t _width;
  std::size_t _height;
};