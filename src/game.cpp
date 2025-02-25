#include "game.h"
#include <iostream>
#include "SDL.h"

Game::Game(Grid& grid)
  : 
  engine(dev()),
  random_w(0, static_cast<int>(grid.GetWidth() - 1)),
  random_h(0, static_cast<int>(grid.GetHeight() - 1)),

  // Could use a settings_parameter to initialze entities but for now just hard-coding the parameters.

  // Initialize obstacles using list-initialization.
  _obstacles{
    ObstacleSnake(grid, 0.03f, 0.003f, Direction::kDown, SDL_Point{10, 10}, 5),
    ObstacleSnake(grid, 0.03f, 0.003f, Direction::kUp,   SDL_Point{20, 20}, 5),
    ObstacleSnake(grid, 0.03f, 0.003f, Direction::kLeft, SDL_Point{30, 30}, 5)
  },

  // Initialize the player snake at the center of the grid.
  _playerSnake(
    grid, 0.05f, 0.005f, Direction::kUp,
    SDL_Point{ 
      static_cast<int>(grid.GetWidth() / 2),
      static_cast<int>(grid.GetHeight() / 2) 
    }
  ),

  // Default-construct Food. Its position will be set in PlaceFood().
  _food(),

  // Initialize the AI snake.
  _aiSnake(
    grid, 0.04f, 0.004f, Direction::kUp,
    SDL_Point{ 
      static_cast<int>(grid.GetWidth() / 2 + 1),  // place aiSnake one cell to the right of playerSnake
      static_cast<int>(grid.GetHeight() / 2)
    }, // initial head for the AI snake.
    _obstacles, _playerSnake, _food
  )
{
  // Populate the vector of snake pointers for collision checks, etc.
  for (auto& obs : _obstacles) {
    _allSnakes_ptrs.push_back(&obs);
  }

  _allSnakes_ptrs.push_back(&_playerSnake);
  _allSnakes_ptrs.push_back(&_aiSnake);

  // Place food in an empty cell.
  PlaceFood();
}


void Game::Run(Controller const &controller, Renderer &renderer,
               std::size_t target_frame_duration) {
  Uint32 title_timestamp = SDL_GetTicks();
  Uint32 frame_start;
  Uint32 frame_end;
  Uint32 frame_duration;
  int frame_count = 0;
  bool running = true;

  while (running) {
    frame_start = SDL_GetTicks();

    // Input, Update, Render - the main game loop.
    controller.HandleInput(running, snake);
    Update();
    renderer.Render(snake, food);

    frame_end = SDL_GetTicks();

    // Keep track of how long each loop through the input/update/render cycle
    // takes.
    frame_count++;
    frame_duration = frame_end - frame_start;

    // After every second, update the window title.
    if (frame_end - title_timestamp >= 1000) {
      renderer.UpdateWindowTitle(score, frame_count);
      frame_count = 0;
      title_timestamp = frame_end;
    }

    // If the time for this frame is too small (i.e. frame_duration is
    // smaller than the target ms_per_frame), delay the loop to
    // achieve the correct frame rate.
    if (frame_duration < target_frame_duration) {
      SDL_Delay(target_frame_duration - frame_duration);
    }
  }
}

SDL_Point Game::GetEmptyCell() {
  SDL_Point targetCell;
  while (true) {
      // Get a random location
      targetCell.x = random_w(engine);
      targetCell.y = random_h(engine);

      // Check that the location is not occupied by any snake.
      // Only return the cell if every snake does NOT hit it.
      if (std::all_of(_allSnakes_ptrs.begin(), _allSnakes_ptrs.end(),
                      [&targetCell](Snake* snake) {
                          return !snake->IsHitBy(targetCell);
                      })) {
          return targetCell;
      }
  }
}


void Game::PlaceFood() {
  _food._position = GetEmptyCell();
  _food.active = true;        // Optionally, ensure food is marked active.  
}

void Game::Update() {
  if (!snake.alive) return;

  snake.Update();

  int new_x = static_cast<int>(snake.head_x);
  int new_y = static_cast<int>(snake.head_y);

  // Check if there's food over here
  if (food.x == new_x && food.y == new_y) {
    score++;
    PlaceFood();
    // Grow snake and increase speed.
    snake.GrowBody();
    snake.speed += 0.02;
  }
}

int Game::GetScore() const { return score; }
int Game::GetSize() const { return snake.size; }