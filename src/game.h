#ifndef GAME_H
#define GAME_H

#include <random>
#include "SDL.h"
#include "controller.h"
#include "renderer.h"
#include "snake.h"
#include "snakes.h"
#include "top_score_manager.h"

class Game {
 public:
  Game(Grid& grid);

  void Run(Controller const &controller, Renderer &renderer,
           std::size_t target_frame_duration);

  int GetScore() const;
  int GetSize() const;

 private:
  Food _food;
  std::vector<ObstacleSnake> _obstacles;
  PlayerSnake _playerSnake;  
  AISnake _aiSnake;

  std::vector<Snake*> _allSnakes_ptrs;

  std::random_device dev;
  std::mt19937 engine;
  std::uniform_int_distribution<int> random_w;
  std::uniform_int_distribution<int> random_h;

  int score{0};
  SDL_Point _previouFoodPosition = _food._position; // For detecting when food has been replaced.
  TopScoreManager _topScoreManager;
  int _topScore{0};

  SDL_Point GetEmptyCell();
  void CheckCollisions();

  void PlaceFood();
  void Update();
};

#endif