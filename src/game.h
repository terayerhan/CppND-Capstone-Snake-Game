#ifndef GAME_H
#define GAME_H

#include <random>
#include "SDL.h"
#include "controller.h"
#include "renderer.h"
#include "snake.h"
#include "snakes.h"

class Game {
 public:
  Game(std::size_t grid_width, std::size_t grid_height);

  void Run(Controller const &controller, Renderer &renderer,
           std::size_t target_frame_duration);
           
  int GetScore() const;
  int GetSize() const;

 private:
  std::vector<ObstacleSnake> _obstacles;
  PlayerSnake _playerSnake;
  Food _food;

  std::vector<Snake*> _allSnakes_ptrs;

  std::random_device dev;
  std::mt19937 engine;
  std::uniform_int_distribution<int> random_w;
  std::uniform_int_distribution<int> random_h;

  int score{0};

  SDL_Point GetEmptyCell();

  void PlaceFood();
  void Update();
};

#endif