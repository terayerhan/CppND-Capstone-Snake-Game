#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "snakes.h"

class Controller {
 public:
  void HandleInput(bool &running, PlayerSnake &playerSnake) const;

 private:
  void ChangeDirection(PlayerSnake &playerSnake, Direction input, Direction opposite) const;
};

#endif