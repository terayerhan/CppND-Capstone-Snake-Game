#pragma once

#include "snake.h"

// PlayerSnake: Snake Controlled by the User.
class PlayerSnake : public Snake {
 public:
    PlayerSnake(const Grid& grid, float initialSpeed, float deltaSpeedLimit) 
        : Snake(grid, initialSpeed, deltaSpeedLimit) {    
    }

    ~PlayerSnake() override = default;
        
    EntityType GetType() const override { return EntityType::PlayerSnake;}
 
};


/* ObstacleSnake: Snake controlled by the Game/computer that act as mobile walls.
   If Some snakes like PlayerSnake or AISnake run into the ObstacleSnake or if ObstacleSnake
   collide with those other snakes, they are penelized still.
*/ 
class ObstacleSnake : public Snake {
 public:
    ObstacleSnake(const Grid& grid, float initialSpeed, float deltaSpeedLimit) 
        : Snake(grid, initialSpeed, deltaSpeedLimit) {    
    }

    ~ObstacleSnake() override = default;

    EntityType GetType() const override { return EntityType::ObstacleSnake;}

 protected:
    // Custom UpadateBody for ObstacleSnake that is slightly different from default Snake.
    void UpdateBody(const SDL_Point &currentHeadCell) override;
};


/* AISnake: Snake controlled by the Game/computer that competes for the food with the 
   PlayerSnake. 

   AISnake uses a custom A* Search algorithm to find the optimal path to the Food while
   avoiding other Snakes in the game including Obstacles and PlayerSnake.
*/
class  AISnake : public Snake {
 public:
    AISnake(const Grid& grid, float initialSpeed, float deltaSpeedLimit) 
        : Snake(grid, initialSpeed, deltaSpeedLimit) {    
    }

    ~AISnake() override = default;

    EntityType GetType() const override { return EntityType::AISnake;}
    
 protected:
  

};
    