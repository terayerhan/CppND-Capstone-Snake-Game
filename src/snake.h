#pragma once

#include <vector>
#include "Entity.h"

class Snake : public Entity {
 public:
  enum class Direction { kUp, kDown, kLeft, kRight }; // You may want to move this to Type.h

  Snake(int grid_width, int grid_height)
      : grid_width(grid_width),
        grid_height(grid_height),
        head_x(grid_width / 2),
        head_y(grid_height / 2) {}

  void Update();

  void GrowBody();

  // checks if coordinates collide with snake's body. You may want to change the name
  bool SnakeCell(int x, int y); 

  const std::vector<SDL_Point>& GetBodyCells() const;
  SDL_Point GetHeadCell() const;
  float GetHeadX() const;
  float GetHeadY() const;
  Direction GetDirection() const;
  bool IsActive() const override;
  float GetSpeed() const;
  int GetHealth() const;

  virtual void Die();
  virtual void Grow();
  virtual void Shrink(int amount = 1);
  virtual void ReduceHealth(int amount);
  virtual void SlowDown(float factor);
  virtual void SpeedUp(float factor);
  

 protected:
  void UpdateHead();
  void UpdateBody(SDL_Point &current_cell, SDL_Point &prev_cell);

  Direction _direction = Direction::kUp;

  float _speed{0.1f};
  int _size{1};
  bool _alive{true};
  float _head_x;
  float _head_y;
  std::vector<SDL_Point> _body_cells;
  bool _growing{false};
  int grid_width;  // You may want to move the grid to the Entity class.
  int grid_height;
};

