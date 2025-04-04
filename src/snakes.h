#pragma once

#include "snake.h"

#include <unordered_set>
#include <unordered_map>
#include <memory>

struct Node {
   SDL_Point cell_;
   std::size_t gCost_;             // TimeSteps to reach node.
   std::size_t fCost_;             // fCost_ = gCost_ + CalculateHeuristic().

   // Actual position of Snake's head within a the cell_.
   float headX_;
   float headY_; 

   Direction direction_; // Direction used to move from parent-Node to this Node[point with headX_, headY_].

   std::shared_ptr<Node> parent_;

   // Node constructor.
   Node(
      SDL_Point cell, std::size_t gCost, std::size_t fCost, float headX, float headY, Direction direction,
      std::shared_ptr<Node> parent
   ) : cell_(cell), gCost_(gCost), fCost_(fCost), headX_(headX), headY_(headY), direction_(direction),
       parent_(parent) 
   {}      
};

struct NodeCompare {
   bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
      return a->fCost_ > b->fCost_;  // Inverted for min-heap priority_queue.
   }
};

// PlayerSnake: Snake Controlled by the User.
class PlayerSnake : public Snake {
 public:
   PlayerSnake(
      const Grid& grid, const Food& food, float initialSpeed, float deltaSpeedLimit, Direction direction,
      const SDL_Point& head 
   ) 
      : Snake(grid, food, initialSpeed, deltaSpeedLimit, direction, head) {    
   }

   ~PlayerSnake() override = default;
      
   EntityType GetType() const override { return EntityType::PlayerSnake;}
 
};

class Food : public Entity {
 public:
   ~Food() override = default;

   EntityType GetType() const override { return EntityType::Food; }
   SDL_Point GetPosition() const override { return _position; }
   int GetCellX() const override {return _position.x;}
   int GetCellY() const override {return _position.y;}
   bool IsActive() const override { return active; }
   void Consume() { active = false; }
   
   // Makes Game a friend class, giving it access to private members (position in particular)
   friend class Game;  

 private:
   bool active;
   SDL_Point _position;
};


/* ObstacleSnake: Snake controlled by the Game/computer that act as mobile walls.
   If Some snakes like PlayerSnake or AISnake run into the ObstacleSnake or if ObstacleSnake
   collide with those other snakes, they are penelized still.
*/ 
class ObstacleSnake : public Snake {
 public:
   ObstacleSnake(
      const Grid& grid, const Food& food, float initialSpeed, float deltaSpeedLimit, Direction direction,
      const SDL_Point& head, std::size_t initialLength
   ) 
   : Snake(grid, food, initialSpeed, deltaSpeedLimit, direction, head) {

      InitializeBody(initialLength);         
   }

   ~ObstacleSnake() override = default;

   EntityType GetType() const override { return EntityType::ObstacleSnake;}

 private:
   void InitializeBody(std::size_t initialLength);
    
};


/* AISnake: Snake controlled by the Game/computer that competes for the food with the 
   PlayerSnake. 

   AISnake uses a custom A* Search algorithm to find the optimal path to the Food while
   avoiding other Snakes in the game including Obstacles and PlayerSnake.
*/
class  AISnake : public Snake {
 public:
   AISnake(
      const Grid& grid, const Food& food, float initialSpeed, float deltaSpeedLimit, Direction direction,
      const SDL_Point& head, const std::vector<ObstacleSnake>& obstacles, 
      const PlayerSnake& playerSnake
   )
   : Snake(grid, food, initialSpeed, deltaSpeedLimit, direction, head),
      _obstacles(obstacles),                    // Read-only reference; no modifications allowed.
      _playerSnake(playerSnake),                // Read-only reference.      
      _predictedObstacles(_obstacles),          // Direct copy for prediction.
      _predictedPlayerSnake(_playerSnake)       // Direct copy for prediction.
   { }

   ~AISnake() override = default;

   EntityType GetType() const override { return EntityType::AISnake;}

   void FindPath();
   void SetDirection(bool IsPlayerSnakeChanged, bool IsFoodChanged);

 private:
   const std::vector<ObstacleSnake>& _obstacles;
   const PlayerSnake& _playerSnake;
   
   std::vector<ObstacleSnake> _predictedObstacles;
   PlayerSnake _predictedPlayerSnake ;

   // Path related variables
   std::vector<Direction> _pathDirections;   // For use in the SetDirection() to guide AISnake to food cell.
   std::vector<SDL_Point> _pathCells;        // Intended for use to review path when playerSnake changes direction.

   // Predicted unordered_map of time_steps to unordered_sets of cells that will be blocked by Obstacle snakes
   std::unordered_map<size_t, std::unordered_set<SDL_Point, SDLPointHash>> _predictedObstaclesBlockedCells;

   // Predicted unordered_map of time_steps to unordered_sets of cells that will be blocked by Player snake
   std::unordered_map<size_t, std::unordered_set<SDL_Point, SDLPointHash>> _predictedPlayerBlockedCells;

   /* The food in this current implementation remains in one place. Hence, there is no need to predict its
      future positions. 
   */

   size_t CalculateHeuristic(
      const float headX, const float headY, const float& speed,
      const int goalX, const int goalY, const int gridWidth, const int gridHeight
   ) const;

   void PredictObstacleBlockedCells(std::size_t initialTimeStep, std::size_t maxTimeStep);
   void PredictPlayerBlockedCells(std::size_t initialTimeStep, std::size_t maxTimeStep);

   std::shared_ptr<Node> AddNode( 
      std::shared_ptr<Node> current, Direction nextDirection, 
      std::deque<SDL_Point> currentBodyCells
   );

   
   //void ReviewPath();
  

};
    