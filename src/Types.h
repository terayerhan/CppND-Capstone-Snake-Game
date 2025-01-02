#pragma once
#include <cstddef>
#include <utility>

enum class EntityType {
    PlayerSnake,
    AISnake,
    ObstacleSnake,
    Food
};

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