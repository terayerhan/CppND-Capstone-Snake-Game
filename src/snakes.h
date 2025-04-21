#pragma once

#include "snake.h"

#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <future>
#include <queue>

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

// Define a NodeState struct containing the essential unique properties of a node
struct NodeState {
   SDL_Point cell;        // Cell position (x,y)   
   size_t timeStep;       // gCost - representing time steps taken

   // Constructor
   NodeState(SDL_Point c, size_t t) 
       : cell(c), timeStep(t) {}

   // Equality operator for unordered_set
   bool operator==(const NodeState& other) const {
       return cell == other.cell && timeStep == other.timeStep;
   }
};

// Hash function for NodeState
struct NodeStateHash {
   std::size_t operator()(const NodeState& state) const {
      std::size_t hash = SDLPointHash()(state.cell);
      // Better bit mixing
      //hash ^= static_cast<int>(state.direction) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
      hash ^= state.timeStep + 0x9e3779b9 + (hash << 6) + (hash >> 2);
      return hash;
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
      const PlayerSnake& playerSnake,
      std::size_t aggressionLevel
   )
   : Snake(grid, food, initialSpeed, deltaSpeedLimit, direction, head),
      _obstacles(obstacles),                    // Read-only reference; no modifications allowed.
      _playerSnake(playerSnake),                // Read-only reference.      
      _predictedObstacles(_obstacles),          // Direct copy for prediction.
      _predictedPlayerSnake(_playerSnake),       // Direct copy for prediction.
      _aggressionLevel(aggressionLevel)
   { }

   ~AISnake() override = default;

   friend class Game;

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
   std::size_t _aggressionLevel;  // determines if the snake will be more cautious or take higher risk path to food.
   bool _IsGuaranteedPathFound = false;  // Is there an overly cautious guaranteed collision free path to food?
   bool _IsInCollision = false; // Is there a cell in this snake currently colliding with an obstacle or self?
   SDL_Point _pathReCalcPoint; // A cell closer to goal where a pathfinding will be redone to see if a safer path has opened up.
   std::size_t _MaxAggressionLevel = 10;
   std::size_t _tailToPastGoalTime; //future timeSteps that it will take for the tail cell to reach the food.

   // Predicted unordered_map of time_steps to unordered_sets of cells that will be blocked by Obstacle snakes
   std::unordered_map<size_t, std::unordered_set<SDL_Point, SDLPointHash>> _predictedObstaclesBlockedCells;

   // Predicted unordered_map of time_steps to unordered_sets of cells that will be blocked by Player snake
   std::unordered_map<size_t, std::unordered_set<SDL_Point, SDLPointHash>> _predictedPlayerBlockedCells;

   /* The food in this current implementation remains in one place. Hence, there is no need to predict its
      future positions. 
   */

   size_t CalculateHeuristic(
      const float headX, const float headY, const float speed,
      const int goalX, const int goalY, const int gridWidth, const int gridHeight
   ) const;

   void PredictObstacleBlockedCells(std::size_t initialTimeStep, std::size_t maxTimeStep);
   void PredictPlayerBlockedCells(std::size_t initialTimeStep, std::size_t maxTimeStep);

   void AddNode( 
      std::shared_ptr<Node> current, Direction nextDirection, 
      const std::deque<SDL_Point>& currentNodeBodyCells,
      std::priority_queue<std::shared_ptr<Node>,
         std::vector<std::shared_ptr<Node>>,
         NodeCompare>& openList,
      std::mutex& openListMutex
   );

   void ReconstructPath(std::shared_ptr<Node> current);
   void ReconstructPartialPath(std::shared_ptr<Node> partialNode);
   void PredictSnakesBlockedCells(std::size_t playerMaxTimeStep, std::size_t obstacleMaxTimeStep);
   std::mutex _pathMutex;

   
   //void ReviewPath();
  

};
    