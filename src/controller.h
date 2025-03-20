#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "snakes.h"

class Controller {
 public:
  bool HandleInput(bool &running, PlayerSnake &playerSnake) const;

 private:
  bool ChangeDirection(PlayerSnake &playerSnake, Direction input, Direction opposite) const;
};

#endif