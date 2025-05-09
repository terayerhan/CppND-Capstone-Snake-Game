#ifndef RENDERER_H
#define RENDERER_H

#include <vector>
#include "SDL.h"
#include "snakes.h"

class Renderer {
 public:
  Renderer(const std::size_t screen_width, const std::size_t screen_height,
           const std::size_t grid_width, const std::size_t grid_height);
  ~Renderer();

  void Render(
    std::vector<ObstacleSnake> const &obstacles, 
    PlayerSnake const &playerSnake,
    AISnake const &aiSnake, Food const &food
  );

  void UpdateWindowTitle(int score, int topScore, int fps);
  void UpdateWindowTitle(int score, int topScore, int fps, const std::string &message);

 private:
  SDL_Window *sdl_window;
  SDL_Renderer *sdl_renderer;

  const std::size_t screen_width;
  const std::size_t screen_height;
  const std::size_t grid_width;
  const std::size_t grid_height;
};

#endif