#include "renderer.h"
#include <iostream>
#include <string>

Renderer::Renderer(const std::size_t screen_width,
                   const std::size_t screen_height,
                   const std::size_t grid_width, const std::size_t grid_height)
    : screen_width(screen_width),
      screen_height(screen_height),
      grid_width(grid_width),
      grid_height(grid_height) {
  // Initialize SDL
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cerr << "SDL could not initialize.\n";
    std::cerr << "SDL_Error: " << SDL_GetError() << "\n";
  }

  // Create Window
  sdl_window = SDL_CreateWindow("Snake Game", SDL_WINDOWPOS_CENTERED,
                                SDL_WINDOWPOS_CENTERED, screen_width,
                                screen_height, SDL_WINDOW_SHOWN);

  if (nullptr == sdl_window) {
    std::cerr << "Window could not be created.\n";
    std::cerr << " SDL_Error: " << SDL_GetError() << "\n";
  }

  // Create renderer
  sdl_renderer = SDL_CreateRenderer(sdl_window, -1, SDL_RENDERER_ACCELERATED);
  if (nullptr == sdl_renderer) {
    std::cerr << "Renderer could not be created.\n";
    std::cerr << "SDL_Error: " << SDL_GetError() << "\n";
  }
}

Renderer::~Renderer() {
  SDL_DestroyWindow(sdl_window);
  SDL_Quit();
}

void Renderer::Render(
  std::vector<ObstacleSnake> const &obstacles, 
  PlayerSnake const &playerSnake,
  AISnake const &aiSnake, Food const &food
) {
  SDL_Rect block;
  block.w = screen_width / grid_width;
  block.h = screen_height / grid_height;

  // Clear screen
  SDL_SetRenderDrawColor(sdl_renderer, 0x1E, 0x1E, 0x1E, 0xFF);
  SDL_RenderClear(sdl_renderer);

  // Render food
  SDL_SetRenderDrawColor(sdl_renderer, 0xFF, 0xCC, 0x00, 0xFF);
  SDL_Point foodPosition = food.GetPosition(); // cache food position.
  block.x = foodPosition.x * block.w;
  block.y = foodPosition.y * block.h;
  SDL_RenderFillRect(sdl_renderer, &block);

  // Render playerSnake
  // Render playerSnake's body
  SDL_SetRenderDrawColor(sdl_renderer, 0xFF, 0xFF, 0xFF, 0xFF);
  for (SDL_Point const &point : playerSnake.GetBodyCells()) {
    block.x = point.x * block.w;
    block.y = point.y * block.h;
    SDL_RenderFillRect(sdl_renderer, &block);
  }

  // Render playerSnake's head
  SDL_Point const & playerSnakeHead = playerSnake.GetPosition();
  block.x = playerSnakeHead.x * block.w;
  block.y = playerSnakeHead.y * block.h;
  if (playerSnake.IsActive()) {
    SDL_SetRenderDrawColor(sdl_renderer, 0x00, 0x7A, 0xCC, 0xFF);
  } else {
    SDL_SetRenderDrawColor(sdl_renderer, 0xFF, 0x00, 0x00, 0xFF);
  }
  SDL_RenderFillRect(sdl_renderer, &block);


  // Render AISnake
  // Render AISnake's body
  SDL_SetRenderDrawColor(sdl_renderer, 0xEE, 0xEE, 0xEE, 0xFF);
  for (SDL_Point const &point : aiSnake.GetBodyCells()) {
    block.x = point.x * block.w;
    block.y = point.y * block.h;
    SDL_RenderFillRect(sdl_renderer, &block);
  }

  // Render AISnake's head
  SDL_Point const & aiSnakeHead = aiSnake.GetPosition();
  block.x = aiSnakeHead.x * block.w;
  block.y = aiSnakeHead.y * block.h;
  SDL_SetRenderDrawColor(sdl_renderer, 0x00, 0x7A, 0xBB, 0xFF);
  SDL_RenderFillRect(sdl_renderer, &block);


  // Render ObstacleSnakes
  for(ObstacleSnake const &obstacle : obstacles) {
    // Render ObstacleSnake
    // Render ObstacleSnake's body
    SDL_SetRenderDrawColor(sdl_renderer, 0xCC, 0xCC, 0xCC, 0xFF);
    for (SDL_Point const &point : obstacle.GetBodyCells()) {
      block.x = point.x * block.w;
      block.y = point.y * block.h;
      SDL_RenderFillRect(sdl_renderer, &block);
    }

    // Render ObstacleSnake's head
    SDL_Point const & obstacleHead = obstacle.GetPosition();
    block.x = obstacleHead.x * block.w;
    block.y = obstacleHead.y * block.h;
    SDL_SetRenderDrawColor(sdl_renderer, 0x00, 0x7A, 0xAA, 0xFF);
    SDL_RenderFillRect(sdl_renderer, &block);
  }

  // Update Screen
  SDL_RenderPresent(sdl_renderer);
}

void Renderer::UpdateWindowTitle(int score, int fps) {
  std::string title{"Snake Score: " + std::to_string(score) + " FPS: " + std::to_string(fps)};
  SDL_SetWindowTitle(sdl_window, title.c_str());
}
