#include "snake.h"
#include <cmath>
#include <iostream>

void Snake::Update() {
  SDL_Point& previousHeadCell = _body_cells.back();  // We first capture the head's cell before updating.
  UpdateHead();
  SDL_Point currentHeadCell{
      static_cast<int>(_head_x),
      static_cast<int>(_head_y)};  // Capture the head's cell after updating.

  // Update all of the body vector items if the snake head has moved to a new
  // cell.
  if (currentHeadCell.x != previousHeadCell.x || currentHeadCell.y != previousHeadCell.y) {
    UpdateBody(currentHeadCell);
  }
}

void Snake::UpdateHead() {
  switch (_direction) {
    case Direction::kUp:
      _head_y -= _speed;
      break;

    case Direction::kDown:
      _head_y += _speed;
      break;

    case Direction::kLeft:
      _head_x -= _speed;
      break;

    case Direction::kRight:
      _head_x += _speed;
      break;
  }

  // Wrap the Snake around to the beginning if going off of the screen.  
  std::pair<float, float> head_xy = _grid.WrapPosition(_head_x, _head_y);
  _head_x = head_xy.first;
  _head_y = head_xy.second;
}

void Snake::UpdateBody(const SDL_Point &currentHeadCell) {
  // Add previous head location to vector
  _body_cells.push_back(currentHeadCell);

  if (!_growing) {
    // Remove the tail from the vector.
    _body_cells.erase(_body_cells.begin());
  } else {
    _growing = false;
    _size++;
  }

  // Check if the snake has died.
  if (_body_cells.size() > 1) { // If there's only one element or none, the head can't overlap with the body        
    // The head is the last element in the vector
    if(std::find_if(_body_cells.begin(), _body_cells.end() - 1, [&currentHeadCell](const SDL_Point& point) {
        return point.x == currentHeadCell.x && point.y == currentHeadCell.y; }) != (_body_cells.end() - 1)) {
          
      _alive = false;
    };
  }
}

void Snake::Grow() { _growing = true; }

// Inefficient method to check if cell is occupied by snake.
bool Snake::SnakeCell(int x, int y) {
  if (x == static_cast<int>(head_x) && y == static_cast<int>(head_y)) {
    return true;
  }
  for (auto const &item : body) {
    if (x == item.x && y == item.y) {
      return true;
    }
  }
  return false;
}