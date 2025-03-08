#include "snake.h"
#include <cmath>
#include <iostream>

void Snake::Update() {
  SDL_Point& previousHeadCell = _body_cells.front();  // Capture the head's cell before updating.
  UpdateHead();
  SDL_Point currentHeadCell{
      static_cast<int>(_head_x),
      static_cast<int>(_head_y)};  // Capture the head's cell after updating.

  // Update all of the body deque items if the snake head has moved to a new cell.
  if (currentHeadCell != previousHeadCell) {
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
  // Add previous head location to the front of the body cells deque
  _body_cells.push_front(currentHeadCell);

  if (!_growing) {
    // Remove the tail from the back of the deque.
    _body_cells.pop_back();
  } else {
    _growing = false;
    //_size++;
  }
  
}


bool Snake::HasSelfCollision() {
  /* Head cell is part of Snake's body, so only search the body cells for head collision with itself
     if snake's size is greater than 1.
  */ 
  if (_body_cells.size() <= 1) {
      return false; // No self-collision: Snake is only one cell, hence it cannot collide with itself.
  }

  // Get head position from front of deque
  const SDL_Point& head = _body_cells.front();
  
   // Use a reverse iterator to check from the tail towards the head, stopping early if found.
   // This is because self-collision is most likely to happen at or closer to tail than head.
   return std::find(_body_cells.rbegin(), _body_cells.rend() - 1, head) != _body_cells.rend() - 1;
}


// Check if cell with coordinate (x, y) is part of this Snake's body.
bool Snake::IsHitBy(const SDL_Point& targetCell) {
  return std::find(_body_cells.begin(), _body_cells.end(), targetCell) != _body_cells.end();
}

// Check if snake is hit by a cell(headCell of another snake) from nake to tail.
bool Snake::IsHitBelowHeadBy(const SDL_Point& offendingCell) {
  if (_body_cells.size() <= 1) return false;

  return std::find(std::next(_body_cells.begin()), _body_cells.end(), offendingCell) != _body_cells.end();
}
