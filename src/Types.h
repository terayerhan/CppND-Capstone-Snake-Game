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


// Hash function for pairs
struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        // Hash each element in the pair
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);

        // Combine the two hashes
        return hash1 ^ (hash2 << 1);  // XOR and bit-shifting to combine
    }
};