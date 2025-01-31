#pragma once

#include <vector>
#include <algorithm> // For std::clamp

#include "Entity.h"
#include "grid.h"
#include "sdl_point_operators.h"

class Snake : public Entity {
 public:
  Snake(const Grid& grid, float initialSpeed, float deltaSpeedLimit)
      : _grid(grid),
        // Clamp deltaSpeed during initialization
        _delta_speed(std::clamp(deltaSpeedLimit, 0.001f, 0.01f)) 
  {
        SetSpeed(initialSpeed);  // Clamp speed during initialization
  } 

  void Update();
  bool HasSelfCollision();

  // checks if coordinates collide with snake's body. You may want to change the name
  bool IsSnakeCell(const int x, const int y); 
  const std::vector<SDL_Point>& GetBodyCells() const {return _body_cells;}

  SDL_Point GetHeadCell() const {
    return SDL_Point{
        static_cast<int>(_head_x),
        static_cast<int>(_head_y)
    };
  }

  float GetHeadX() const {return _head_x;}
  float GetHeadY() const {return _head_y;}
  //Direction GetDirection() const{return _direction;} //For now _direction is public so no need.
  bool IsActive() const override {return _alive;}
  //float GetSpeed() const {return _speed;}
  int GetHealth() const {return _health;}

  /* Collision Matrix Implications*/
  virtual void Die();
  virtual void Grow() { _growing = true; }
  virtual void Shrink(int amount = 1);
  virtual void ReduceHealth(int amount);
  
  ~Snake() override = default;

  // Inline getter for speed
  inline float GetSpeed() const { return _speed; }

  // Inline getter for deltaSpeed
  inline float GetDeltaSpeed() const { return _delta_speed; }

  // Accelerate: Increases speed by deltaSpeed, clamped to 0.999
  inline void Accelerate() { SetSpeed(_speed + _delta_speed); }

  inline void Accelerate(float increment) { SetSpeed(_speed + increment); }

  // Decelerate: Decreases speed by deltaSpeed, clamped to 0.01
  inline void Decelerate() { SetSpeed(_speed - _delta_speed); }
  
  inline void Decelerate(float decrement) { SetSpeed(_speed - decrement); }

  /* I have left _direction public for now since I am concerned about input latency.
     I could change it later to protected with public getter and setters
  */
  Direction _direction; //Game should control direction initial value
  

 protected:
  void UpdateHead();
  virtual void UpdateBody(const SDL_Point &currentHeadCell);

  // Inline setter for speed with clamping logic
  inline void SetSpeed(float newSpeed) {
      /* Clamp speed to the range [0.01, 0.999] to prevent snakes heads from
         jumping cells which could lead to snakes phasing through obstacles 
         when they have speed of 1 or above. This top limit can be remove when 
         mechanisms are put in place to handle detecting collision when snakes 
         heads can move 1 cell or more per step or game update.
      */ 
      _speed = std::clamp(newSpeed, 0.01F, 0.999F);
  }

   
  int _size;            //Game should control size initial value
  bool _alive{true};
  float _head_x;
  float _head_y;
  std::vector<SDL_Point> _body_cells;
  bool _growing{false};
  int _health;          //Review the need for _health or how to implement it.
  const Grid& _grid;  

 private:
  float _speed;               // Current speed of the snake
  const float _delta_speed;   // Immutable rate of change of speed
};

