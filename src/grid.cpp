#include "grid.h"
#include "sdl_point_operators.h"

Grid::Grid(size_t width, size_t height) : _width(width), _height(height) {
    // Initialize grid width for hash function
    SDLPointHash::grid_width = width;
}

std::pair<float, float> Grid::WrapPosition(float x, float y) const {
    // Wraps coordinates around the grid boundaries (toroidal wrapping).
    if (x < 0) x += _width;
    if (x >= _width) x -= _width;
    if (y < 0) y += _height;
    if (y >= _height) y -= _height;

    return {x, y};
}


std::pair<int, int> Grid::WrapPosition(int x, int y) const {
    if (x < 0) x += _width;
    if (x >= _width) x -= _width;
    if (y < 0) y += _height;
    if (y >= _height) y -= _height;
    
    return {x, y};
}