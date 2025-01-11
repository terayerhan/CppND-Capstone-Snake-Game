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
  
}


bool Snake::HasSelfCollision() {
  /* Head cell is part of Snake's body, so only search the body cells for head collision with itself
     if snake's size is greater than 1.
  */ 
  if (_body_cells.size() <= 1) {
      return false; // No self-collision: Snake is only one cell, hence it cannot collide with itself.
  }

  SDL_Point currentHead{static_cast<int>(_head_x), static_cast<int>(_head_y)};
  
  // Use "_body_cells.end() - 1" because head cell is last element in _body_cell vector.
  return std::any_of(_body_cells.begin(), _body_cells.end() - 1,
      [&currentHead](const auto& cell) {
          return cell.x == currentHead.x && cell.y == currentHead.y;
      });
}


void Snake::Grow() { _growing = true; }


// Check if cell with coordinate (x, y) is part of this Snake's body.
bool Snake::IsSnakeCell(const int x, const int y) {
  if(std::find_if(_body_cells.begin(), _body_cells.end(), [&x, &y](const SDL_Point& cell) {
      return cell.x == x && cell.y == y; }) != _body_cells.end()) {

    return true; //Cell is part of this Snake's body.
  };

  return false; // Cell is not part of this Snake's body.
}