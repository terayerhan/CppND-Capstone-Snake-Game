#pragma once

#include "Entity.h"
#include "grid.h"
#include "sdl_point_operators.h"

class Food : public Entity {
 public:
  ~Food() override = default;
   EntityType GetType() const override { return EntityType::Food; }
   SDL_Point GetPosition() const override { return _position; }
   int GetCellX() const override {return _position.x;}
   int GetCellY() const override {return _position.y;}
   bool IsActive() const override { return active; }
   void Consume() { active = false; }

   // Makes Game a friend class, giving it access to private members (position in particular)
   friend class Game;  
   
 private:
  bool active;
  SDL_Point _position;
};