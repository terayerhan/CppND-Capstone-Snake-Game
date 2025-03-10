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


void Game::CheckCollisions() {
  // Check self-collision of playerSnake and AISnake.
  if (_playerSnake.HasSelfCollision()) { 
    _playerSnake.Decelerate();
    _playerSnake.ReduceHealth(1); 
    //_playerSnake.Shrink(1);
  }

  if (_aiSnake.HasSelfCollision()) { 
    _aiSnake.Decelerate();
    //_aiSnake.Shrink();
  }

  // Get the head of playerSnake and AISnake.
  SDL_Point playerHeadCell = _playerSnake.GetPosition();
  SDL_Point aiSnakeHeadCell = _aiSnake.GetPosition();

  // Create a vector of snake pointers to store any snake playerSnake, aiSnake or obstacleSnake
  // collides with food cell.
  std::vector<Snake*> snakesAtFoodCellPtrs;

  // Check food collision of playerSnake and AISnake.
  SDL_Point foodCell = _food.GetPosition(); // cash food position.
  if (playerHeadCell == foodCell) { snakesAtFoodCellPtrs.push_back(&_playerSnake); }
  if (aiSnakeHeadCell == foodCell) { snakesAtFoodCellPtrs.push_back(&_aiSnake); }

  // Check head-to-head collision of playerSnake and aiSnake and if there isn't, chech playerSnake
  // collision against aiSnake and aiSnake collision against playerSnake.
  if (playerHeadCell == aiSnakeHeadCell) {
    // Determine the snake that got to the cell last and penalize it.
    _aiSnake.GetDistanceInHeadCell() > _playerSnake.GetDistanceInHeadCell() ? 
      _playerSnake.ReduceHealth(1)  : _aiSnake.Decelerate();
  }
  else { 
    // Check playerSnake collision against aiSnake's rest of body.
    if ( _aiSnake.IsHitBelowHeadBy(playerHeadCell)) {
      _playerSnake.Decelerate();
      _playerSnake.ReduceHealth(1);
    }

    // Check aiSnake collision against playerSnake's rest of body. (may not be needed if aiSnake path planner works as intented)
    if ( _playerSnake.IsHitBelowHeadBy(aiSnakeHeadCell)) {
      _aiSnake.Decelerate();
    }
  }

  // Check obstacleSnakes' collisions against playerSnake and aiSnake.
  for (auto& obstacleSnake : _obstacles) {
    // cash obstacleSnake's head.
    SDL_Point obstacleSnakeHeadCell = obstacleSnake.GetPosition();
    // Check if obstacles snake can eat the food then add it to the list of snake trying to eat the food.
    if (obstacleSnakeHeadCell == foodCell) { snakesAtFoodCellPtrs.push_back(&obstacleSnake); }

    // Check if any cell in the body of the obstacleSnake collides with a playerSnake and aiSnake.
    for (auto& obstacleCell : obstacleSnake._body_cells) {
      if (_playerSnake.IsHitBy(obstacleCell)) { 
        _playerSnake.Decelerate();
        _playerSnake.ReduceHealth(1);
      }

      if (_aiSnake.IsHitBy(obstacleCell)) {
        _aiSnake.Decelerate();
      }
    }
  }

  // Grow the snake that reached the food cell first.
  std::vector<std::size_t> distancesOfSnakesAtFoodCell; // longest distance will be the for snake that got to cell first.
  
  if (snakesAtFoodCellPtrs.size() == 1) {
    // Grow the only snake that got to the food cell.
    snakesAtFoodCellPtrs.back()->Grow();
  }
  else if (snakesAtFoodCellPtrs.size() > 1) {
    // grow the first snake to reach the food cell.
    for ( auto& snakePtr : snakesAtFoodCellPtrs) {
      distancesOfSnakesAtFoodCell.push_back(snakePtr->GetDistanceInHeadCell());
    }

    // Find iterator to max element
    auto max_it = std::max_element(distancesOfSnakesAtFoodCell.begin(), distancesOfSnakesAtFoodCell.end());
    // Convert iterator to index
    std::size_t max_index = std::distance(distancesOfSnakesAtFoodCell.begin(), max_it);
    snakesAtFoodCellPtrs[max_index]->Grow(); // Grow the snake.
  }

  // Check if playerSnake's health is zero and end game if it is.
  if (_playerSnake._health == 0) {_playerSnake._alive = false;}

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