#include "controller.h"
#include <iostream>
#include "SDL.h"
#include "snake.h"

bool Controller::ChangeDirection(PlayerSnake &playerSnake, Direction input, Direction opposite) const {
  Direction previousDirection = playerSnake._direction;
  if (playerSnake._direction != opposite || playerSnake.GetSize() == 1) playerSnake._direction = input;

  /* Check if the playerSnake's direction has changed and return the result.
  
     This is for use to let the AISnake know that the playerSnake has changed
     its direction since unlike the playerSnake that depends on the user's 
     eyes to detect when the AISnake or ObstacleSnakes have changed direction,
     the AISnake uses this mechanism as its sensor and the fact that the 
     ObstacleSnakes move in predefined directions. 
  */
  return previousDirection != playerSnake._direction ? true : false;
}

void Controller::HandleInput(bool &running, PlayerSnake &playerSnake) const {
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    if (e.type == SDL_QUIT) {
      running = false;
    } else if (e.type == SDL_KEYDOWN) {
      switch (e.key.keysym.sym) {
        case SDLK_UP:
          ChangeDirection(playerSnake, Direction::kUp, Direction::kDown);
          break;

        case SDLK_DOWN:
          ChangeDirection(playerSnake, Direction::kDown, Direction::kUp);
          break;

        case SDLK_LEFT:
          ChangeDirection(playerSnake, Direction::kLeft, Direction::kRight);
          break;

        case SDLK_RIGHT:
          ChangeDirection(playerSnake, Direction::kRight, Direction::kLeft);
          break;
      }
    }
  }
}