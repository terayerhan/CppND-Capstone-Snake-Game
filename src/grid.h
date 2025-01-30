#pragma once

#include <utility>

class Grid {
 public:
    Grid(size_t width, size_t height);

    size_t GetWidth() const { return _width; }
    size_t GetHeight() const { return _height; }

    // Wraps floating point coordinates
    std::pair<float, float> WrapPosition(float x, float y) const;

    // Wrap integer position
    std::pair<int, int> WrapPosition(int x, int y) const;

 private:    
    /* data */
    size_t _width;
    size_t _height;
    
};


