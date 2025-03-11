#pragma once

#include <SDL2/SDL.h>

// Equality operator (needed for unordered_set)
inline bool operator==(const SDL_Point& lhs, const SDL_Point& rhs) {
   return (lhs.x == rhs.x && lhs.y == rhs.y);
}

// Inequality operator 
inline bool operator!=(const SDL_Point& lhs, const SDL_Point& rhs) {
   return !(lhs == rhs);
}

// Less than operator (needed for ordered containers and std::sort)
inline bool operator<(const SDL_Point& lhs, const SDL_Point& rhs) {
    return (lhs.x < rhs.x) || (lhs.x == rhs.x && lhs.y < rhs.y);
}

/* // Greater than operator
inline bool operator>(const SDL_Point& lhs, const SDL_Point& rhs) {
   return rhs < lhs;
}

// Less than or equal operator
inline bool operator<=(const SDL_Point& lhs, const SDL_Point& rhs) {
   return !(rhs < lhs);
}

// Greater than or equal operator
inline bool operator>=(const SDL_Point& lhs, const SDL_Point& rhs) {
   return !(lhs < rhs);
} */ // I don't think I need these once for now.

// Hash function optimized for grid-based games (needed for unordered_set)
struct SDLPointHash {
    static std::size_t grid_width;
    
    std::size_t operator()(const SDL_Point& p) const {
        return p.x * grid_width + p.y;
    }
};