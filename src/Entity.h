#pragma once
#include "Types.h"
#include <SDL2/SDL.h>

class Entity {
public:
    virtual ~Entity() = default;
    virtual EntityType GetType() const = 0;
    virtual SDL_Point GetPosition() const = 0;
    virtual bool IsActive() const = 0;
    virtual void Update() = 0;
    virtual void Render() = 0;
};