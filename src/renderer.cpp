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

  // Helper function to render a single cell at a position
  auto renderCell = [this, &block](const SDL_Point& position, Uint8 r, Uint8 g, Uint8 b) {
    block.x = position.x * block.w;
    block.y = position.y * block.h;
    SDL_SetRenderDrawColor(sdl_renderer, r, g, b, 0xFF);
    SDL_RenderFillRect(sdl_renderer, &block);
  };

  // Helper function to render any snake's body
  auto renderSnakeBody = [this, &block](const auto& snake, Uint8 r, Uint8 g, Uint8 b) {
    SDL_SetRenderDrawColor(sdl_renderer, r, g, b, 0xFF);
    for (SDL_Point const &point : snake.GetBodyCells()) {
      block.x = point.x * block.w;
      block.y = point.y * block.h;
      SDL_RenderFillRect(sdl_renderer, &block);
    }
  };

  // Render food
  renderCell(food.GetPosition(), 0xFF, 0xCC, 0x00);

  // Render playerSnake's body
  renderSnakeBody(playerSnake, 0xFF, 0xFF, 0xFF);

  // Render playerSnake's head with conditional coloring
  if (playerSnake.IsActive()) {
    renderCell(playerSnake.GetPosition(), 0x00, 0x7A, 0xCC);
  } else {
    renderCell(playerSnake.GetPosition(), 0xFF, 0x00, 0x00);
  }

  // Render AISnake
  renderSnakeBody(aiSnake, 0xEE, 0xEE, 0xEE);
  renderCell(aiSnake.GetPosition(), 0x00, 0x7A, 0xBB);

  // Render ObstacleSnakes
  for(ObstacleSnake const &obstacle : obstacles) {
    renderSnakeBody(obstacle, 0xCC, 0xCC, 0xCC);
    renderCell(obstacle.GetPosition(), 0x00, 0x7A, 0xAA);
  }

  // Update Screen
  SDL_RenderPresent(sdl_renderer);
}

void Renderer::UpdateWindowTitle(int score, int fps) {
  std::string title{"Snake Score: " + std::to_string(score) + " FPS: " + std::to_string(fps)};
  SDL_SetWindowTitle(sdl_window, title.c_str());
}
