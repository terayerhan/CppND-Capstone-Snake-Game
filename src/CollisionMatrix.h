#pragma once
#include "Types.h"
#include <unordered_map>
#include <functional>
#include <utility>

class Snake;
class Food;

using SnakeCollisionHandler = std::function<void(Snake&, Snake&)>;
using FoodCollisionHandler = std::function<void(Snake&, Food&)>;

class CollisionMatrix {
public:
    void RegisterSnakeCollision(EntityType type1, EntityType type2, SnakeCollisionHandler handler);
    void RegisterFoodCollision(EntityType snakeType, FoodCollisionHandler handler);
    void HandleCollision(Snake& snake1, Snake& snake2);
    void HandleFoodCollision(Snake& snake, Food& food);

private:
    std::unordered_map<std::pair<EntityType, EntityType>, SnakeCollisionHandler, PairHash> snake_collisions;
    std::unordered_map<EntityType, FoodCollisionHandler> food_collisions;
};