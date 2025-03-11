#include "controller.h"
#include <iostream>
#include "SDL.h"
#include "snake.h"

void Controller::ChangeDirection(PlayerSnake &playerSnake, Direction input, Direction opposite) const {
  if (playerSnake._direction != opposite || playerSnake.GetSize() == 1) playerSnake._direction = input;
  return;
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