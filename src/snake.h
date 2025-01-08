#pragma once

#include <vector>
#include "Entity.h"
#include "grid.h"

class Snake : public Entity {
 public:
  enum class Direction { kUp, kDown, kLeft, kRight }; // You may want to move this to Type.h

  Snake(const Grid& grid)
      : _grid(grid) {}

  void Update();

  // checks if coordinates collide with snake's body. You may want to change the name
  bool SnakeCell(int x, int y); 

  const std::vector<SDL_Point>& GetBodyCells() const;
  SDL_Point GetHeadCell() const;
  float GetHeadX() const {return _head_x;}
  float GetHeadY() const {return _head_y;}
  Direction GetDirection() const{return _direction;}
  bool IsActive() const override {return _alive;}
  float GetSpeed() const {return _speed;}
  int GetHealth() const {return _health;}

  virtual void Die();
  virtual void Grow();
  virtual void Shrink(int amount = 1);
  virtual void ReduceHealth(int amount);
  virtual void SlowDown(float factor);
  virtual void SpeedUp(float factor);
  

 protected:
  void UpdateHead();
  void UpdateBody(SDL_Point &current_cell, SDL_Point &prev_cell);

  Direction _direction; //Game should control direction initial value

  float _speed{0.1f};
  int _size;            //Game should control size initial value
  bool _alive{true};
  float _head_x;
  float _head_y;
  std::vector<SDL_Point> _body_cells;
  bool _growing{false};
  int _health;          //Review the need for _health or how to implement it.
  const Grid& _grid;
};

