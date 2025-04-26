#pragma once
#include <cstddef>
#include <utility>

enum class EntityType {
    PlayerSnake,
    AISnake,
    ObstacleSnake,
    Food
};

enum class Direction { kUp, kDown, kLeft, kRight }; 

/**
 * @brief Returns the opposite direction of the given direction.
 *
 * @param d The input direction.
 * @return Direction The direction opposite to d.
 */
inline Direction OppositeDirection(Direction d) {
    switch(d) {
        case Direction::kUp:    return Direction::kDown;
        case Direction::kDown:  return Direction::kUp;
        case Direction::kLeft:  return Direction::kRight;
        case Direction::kRight: return Direction::kLeft;
        default:                return d; // Alternatively, throw an exception or assert.
    }
}

