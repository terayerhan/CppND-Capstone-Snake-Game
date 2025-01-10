#pragma once

#include "snake.h"

class PlayerSnake : public Snake {
 public:
    PlayerSnake(const Grid& grid, float initialSpeed, float deltaSpeedLimit) 
        : Snake(grid, initialSpeed, deltaSpeedLimit) {    
    }

    ~PlayerSnake() override = default;
        
    EntityType GetType() const override { return EntityType::PlayerSnake;}
 
};