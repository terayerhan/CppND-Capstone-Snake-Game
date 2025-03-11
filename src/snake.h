#pragma once

// Forward declarations
class Game;

#include <vector>
#include <algorithm> // For std::clamp
#include <deque>

#include "Entity.h"
#include "grid.h"
#include "sdl_point_operators.h"

class Snake : public Entity {
 public:
  // Constructor.
  Snake(
    const Grid& grid, 
    float initialSpeed, 
    float deltaSpeedLimit, 
    Direction direction, 
    const SDL_Point& head
  )
    : _direction(direction),
      _head_x(static_cast<float>(head.x)),
      _head_y(static_cast<float>(head.y)),
      _grid(grid),
      _delta_speed(std::clamp(deltaSpeedLimit, 0.001f, 0.01f))
  {
      // Place the head cell into the body container (head is at the front)
      _body_cells.push_front(head);
      SetSpeed(initialSpeed);  // Clamps and sets the initial speed.
  }

  // Copy Assignment Operator
  Snake& operator=(const Snake& other) {
    if (this != &other) {
      // _grid and _delta_speed are not assigned because they are const.
      // Note: _grid and _delta_speed remain unchanged.
      _head_x = other._head_x;
      _head_y = other._head_y;
      _body_cells = other._body_cells;
      _growing = other._growing;
      _health = other._health;
      _speed = other._speed;
      _direction = other._direction;
      _alive = other._alive;      
    }
    return *this;
  }

  void Update();
  bool HasSelfCollision();

  // checks if coordinates collide with snake's body. You may want to change the name
  bool IsHitBy(const SDL_Point& targetCell); 
  bool IsHitBelowHeadBy(const SDL_Point& offendingCell);
  float GetDistanceInHeadCell() const;
  const std::deque<SDL_Point>& GetBodyCells() const {return _body_cells;}

  // Getter for the Snake's head cell position.
  SDL_Point GetPosition() const override {
    return SDL_Point{
        static_cast<int>(_head_x),
        static_cast<int>(_head_y)
    };
  }

  // Makes Game a friend class, giving it access to private members. Game is trusted to 
  // modify this object appropriatly and responsibly.
  friend class Game; 

  // Getter for direction which is private but Game is friends with this class so it is
  // the only Object that can set the direction of snakes.  Other snakes should not be able to 
  // change the direction of others apart from thier own.
  inline Direction GetDirection() const {return _direction; }

  int GetCellX() const override {return static_cast<int>(_head_x);}  // Return the snake's head cell's x-coordinate.
  int GetCellY() const override {return static_cast<int>(_head_y);}  // Return the snake's head cell's y-coordinate.

  float GetHeadX() const {return _head_x;}
  float GetHeadY() const {return _head_y;}
  //Direction GetDirection() const{return _direction;} //For now _direction is public so no need.
  bool IsActive() const override {return _alive;}
  //float GetSpeed() const {return _speed;}
  int GetHealth() const {return _health;}

  /* Collision Matrix Implications*/
  //virtual void Die();
  virtual void Grow() { _growing = true; }
  //virtual void Shrink(int amount = 1);
  virtual void ReduceHealth(int amount) { _health - amount; };
  
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

  Direction _direction;
  bool _alive{true};
  float _head_x;
  float _head_y;
  std::deque<SDL_Point> _body_cells;
  bool _growing{false};
  int _health;          //Review the need for _health or how to implement it.
  const Grid& _grid;  

 private:
  float _speed;               // Current speed of the snake
  const float _delta_speed;   // Immutable rate of change of speed
};

