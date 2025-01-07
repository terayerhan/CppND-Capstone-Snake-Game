#pragma once

#include <utility>

class Grid {
 public:
    Grid(int width, int height);

    int GetWidth() const { return _width; }
    int GetHeight() const { return _height; }

    // Wraps floating point coordinates
    std::pair<float, float> WrapPosition(float x, float y) const;

    // Wrap integer position
    std::pair<int, int> WrapPosition(int x, int y) const;

 private:    
    /* data */
    int _width;
    int _height;
    
};


