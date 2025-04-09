#include "game.h"
#include <iostream>
#include "SDL.h"

Game::Game(Grid& grid)
  : 
  engine(dev()),
  random_w(0, static_cast<int>(grid.GetWidth() - 1)),
  random_h(0, static_cast<int>(grid.GetHeight() - 1)),

  // Could use a settings_parameter to initialze entities but for now just hard-coding the parameters.

  // Default-construct Food. Its position will be set in PlaceFood().
  _food(),

  // Initialize obstacles using list-initialization.
  _obstacles{
    ObstacleSnake(grid, _food, 0.03f, 0.003f, Direction::kDown, SDL_Point{10, 10}, 5),
    ObstacleSnake(grid, _food, 0.03f, 0.003f, Direction::kUp,   SDL_Point{20, 20}, 5),
    //ObstacleSnake(grid, _food, 0.03f, 0.003f, Direction::kLeft, SDL_Point{30, 30}, 5)
  },

  // Initialize the player snake at the center of the grid.
  _playerSnake(
    grid, _food, 0.05f, 0.005f, Direction::kUp,
    SDL_Point{ 
      static_cast<int>(grid.GetWidth() / 2),
      static_cast<int>(grid.GetHeight() / 2) 
    }
  ),  

  // Initialize the AI snake.
  _aiSnake(
    grid, _food, 0.04f, 0.004f, Direction::kUp,
    SDL_Point{ 
      static_cast<int>(grid.GetWidth() / 2 + 1),  // place aiSnake one cell to the right of playerSnake
      static_cast<int>(grid.GetHeight() / 2)
    }, // initial head for the AI snake.
    _obstacles, _playerSnake
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
    /* Handle get user direction input and detect if player snake has changed direction. 
       The detection of playerSnake direction change and food position change serve as sensor
       outputs for the AISnake just like the playerSnake uses the User's eyes to detect all these 
       changes for deciding how to navigate the grid. 
    */
    bool IsPlayerSnakeDirChanged = controller.HandleInput(running, _playerSnake);

    // Check if previous food position has changed to conditionally update AISnake path.
    bool IsFoodPositionChanged = _previouFoodPosition != _food._position;

    if (IsFoodPositionChanged) {
      _aiSnake.SetDirection(IsPlayerSnakeDirChanged, IsFoodPositionChanged);
      _previouFoodPosition = _food._position; // Update _previousFoodPosition after detection of change.
    }
    else {
      // Set aiSnake's direction base on food position not changed and whether playerSnake position 
      // has changed
      _aiSnake.SetDirection(IsPlayerSnakeDirChanged, IsFoodPositionChanged);
    }
    
    Update();
    renderer.Render(_obstacles, _playerSnake, _aiSnake, _food);

    frame_end = SDL_GetTicks();

    // Keep track of how long each loop through the input/update/render cycle
    // takes.
    frame_count++;
    frame_duration = frame_end - frame_start;

    // After every second, update the window title.
    if (frame_end - title_timestamp >= 1000) {
      renderer.UpdateWindowTitle(GetScore(), frame_count);
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


/**
 * @brief Handles all collision detection and resolution in the game
 * 
 * This method checks for multiple types of collisions and applies appropriate consequences:
 * - Self collisions for both player and AI snakes
 * - Food collisions for all snakes (player, AI, and obstacles)
 * - Head-to-head collisions between player and AI snakes
 * - Body collisions between all snakes
 * 
 * When multiple snakes collide with food simultaneously, only the snake that has
 * progressed furthest into the cell will consume it and grow.
 */
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
    _aiSnake.IsInCollision = true;
    std::cout << " Self_Collision Detected!!"<< std::endl;
  }

  // Cache the head of playerSnake and AISnake.
  SDL_Point playerHeadCell = _playerSnake.GetPosition();
  SDL_Point aiSnakeHeadCell = _aiSnake.GetPosition();

  // Create a vector of snake pointers to store any snake playerSnake, aiSnake or obstacleSnake
  // collides with food cell.
  std::vector<Snake*> snakesAtFoodCellPtrs;

  // Check food collision of playerSnake and AISnake.
  SDL_Point foodCell = _food.GetPosition(); // cache food position.
  if (playerHeadCell == foodCell) { snakesAtFoodCellPtrs.push_back(&_playerSnake); }
  if (aiSnakeHeadCell == foodCell) { snakesAtFoodCellPtrs.push_back(&_aiSnake); }

  // Check head-to-head collision of playerSnake and aiSnake and if there isn't, chech playerSnake
  // collision against aiSnake and aiSnake collision against playerSnake.
  if (playerHeadCell == aiSnakeHeadCell) {
    // Determine the snake that got to the cell last and penalize it.
    if (_aiSnake.GetDistanceInHeadCell() > _playerSnake.GetDistanceInHeadCell()) {
      _playerSnake.ReduceHealth(1); 
    }
    else {
      _aiSnake.Decelerate();
      _aiSnake.IsInCollision = true;
    }
      
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
      _aiSnake.IsInCollision = true;
    }
  }

  // Check obstacleSnakes' collisions with food,  playerSnake and aiSnake and collision of playerSnake and 
  // aiSnake with the obstacles' snakes. The game logic dictate that if a playerSnake or aiSnake comes in 
  // contact with any obstacleSnake, they are penelized even if the contact is initiated by the obstacle
  // snake. The obstacleSnakes are not penelized for any contact, only the AISnake and PlayerSnake.
  for (auto& obstacleSnake : _obstacles) {
    // cache obstacleSnake's head.
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
        _aiSnake.IsInCollision = true;
        std::cout << "ObstacleSnake_Collision Detected"<< std::endl;
      }
    }
  }

  // Check if playerSnake's health is zero and end game if it is.
  if (_playerSnake._health < 1) {
    _playerSnake._alive = false;
    return; // if player is no longer alive, no need to continue.
  }

  // Grow the snake that reached the food cell first.
  std::vector<std::size_t> distancesOfSnakesAtFoodCell; // longest distance will be the for snake that got to cell first.

  if (snakesAtFoodCellPtrs.size() == 1) {
    // Grow the only snake that got to the food cell.
    auto& growingSnake = *(snakesAtFoodCellPtrs.back());
    growingSnake.Grow();
    std::cout << "Food eatten by a Snake"<< std::endl;
    growingSnake.Accelerate();
    PlaceFood();
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

    // Accelerate snake that got to the food cell first but remove the tail of snakes that got to the food
    // cell subsequently.
    for (std::size_t i = 0; i < snakesAtFoodCellPtrs.size(); i++) {
      if (i == max_index) {
        auto& growingSnake = *(snakesAtFoodCellPtrs[max_index]);
        growingSnake.Grow();
        growingSnake.Accelerate();
      }
      else {
        // Remove the tails of snakes that got to the food cell after the first snake. This is because of 
        // how the tails of any snake that is in the food cell after an update are not removed in case there
        // are multiple snakes that also got to the food cell after that same update. This is to wait till
        // a tie break is applied here in Game::CheckCollision().
        snakesAtFoodCellPtrs[i]->_body_cells.pop_back(); 
      }
    }
    
    PlaceFood();
  }

}


void Game::PlaceFood() {
  _food._position = GetEmptyCell();
  _food.active = true;        // Optionally, ensure food is marked active.  
}

void Game::Update() {
  // Check if playerSnake is alive.
  if (!_playerSnake._alive) return;  // AKA GAME OVER : For now all snakes just stop moving.

  // Update snakes.
  for(Snake* snakePtr : _allSnakes_ptrs) {
    snakePtr->Update();
  }

  // Check snakes collisions.
  CheckCollisions(); 
  
}

int Game::GetScore() const { return _playerSnake.GetBodyCells().size() - 1; } // For now.
int Game::GetSize() const { return _playerSnake.GetBodyCells().size(); }