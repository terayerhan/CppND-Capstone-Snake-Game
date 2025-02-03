#include "snakes.h"

std::shared_ptr<Node> AISnake::AddNode( std::shared_ptr<Node> current, Direction nextDirection, 
    std::vector<SDL_Point>& currentBodyCells
    ) {
    
    // Initialize the nextNode variables
    float nextHeadX = current->headX_;         // Use this to update nextHeadCellX
    float nextHeadY = current->headY_;

    SDL_Point currentCell = current->cell_;  // Cach the current node's cell.
    SDL_Point nextHeadCell = currentCell;    // initialize the nextHeadCell.

    /* Initialize the next node's time step from the current node's gCost. gCost the number of (time)steps 
       it will take the head of a snake to reach a position (float) not just the Cell position. 
    */
    size_t nextTimeStep = current->gCost_;   

    float speed = GetSpeed();  // Use the Snake's speed to determine the distance traveled by snake per step.

    // While still in the current nodes's cell, keep stepping forward till you reach the next cell.
    while(nextHeadCell == currentCell) {
        // Move one step in the nextDirection.
        switch (nextDirection) {
            case Direction::kUp:
                nextHeadY -= speed;
                break;

            case Direction::kDown:
                nextHeadY += speed;
                break;

            case Direction::kLeft:
                nextHeadX -= speed;
                break;

            case Direction::kRight:
                nextHeadX += speed;
                break;
        }

        // Wrap the Snake around to the beginning if going off of the screen.  
        std::pair<float, float> head_xy = _grid.WrapPosition(nextHeadX, nextHeadY);
        nextHeadX = head_xy.first;
        nextHeadY = head_xy.second;

        nextHeadCell.x = static_cast<int>(nextHeadX);
        nextHeadCell.y = static_cast<int>(nextHeadY);

        // Increment time steps by 1.
        nextTimeStep++;

        /* Since  head is moving till it reaches the next cell, check if this simulatd snake head
           has reached the next cell so that its bodycells can be updated before checking for 
           collision because when the head moves to a new cell the tail will vacat its previous 
           position hence, this can affect the result o f the collision check. I the cells are not
           moved appropraitely, more optimal paths could be blocked.
        */

        // Check if the snake will reach the next cell after the step.
        if (nextHeadCell != currentCell) {
            // Snake has arrived in next cell.//

            /* Tail is at the back of currentBodyCells to avoid the need to reverse it to match
               back to front nature of the body cells vector of Snake hence, remove it first to 
               reduce the amount of elements that will be shifted when the head is inserted at 
               the front of currrentBody vector.
            */
            currentBodyCells.pop_back();
            currentBodyCells.insert(currentBodyCells.begin(), nextHeadCell);

            // Check for collision with playerSnake and self.//

            /* Check if nextTimeStep is a key in  the unordered_map of  _predictedPlayerBlockedCells
               (which also implies in same for _predictedObstacleBlockedCells since the max time step 
               of prediction is same for both). If the time step is not in the unordered_map, G
               Generate more predicted blocked cells for both playerSnake and obstacle snakes up to
               the heuristic of the nextCell. 
            */
            if (_predictedPlayerBlockedCells.count(nextTimeStep)) {
                /* nextTimeStep exist in the blockedCells unordered_maps, check for collision
                   playerSnake and self. 
                */
                if (
                    _predictedPlayerBlockedCells[nextTimeStep].count(nextHeadCell) ||
                    std::find(
                        currentBodyCells.begin() + 1, currentBodyCells.end(), nextHeadCell)
                        != currentBodyCells.end()
                    ) {
                    // Taking this direction from the current node will result in a collision with either 
                    // player sanke or self. Return nullptr to indicate this.
                    return nullptr;
                }
            }
            else {
                /* nextTimeStep does not exist Generate more time steps enough to reach the goal(food) */
                GenerateBlockedCells(
                    nextTimeStep,
                    CalculateHeuristic(
                        nextHeadX, nextHeadY, speed, _food.GetCellX(), _food.GetCellY(),
                        _grid.GetWidth(), _grid.GetHeight()
                    )
                );

                /* nextTimeStep Should Now exist in the blockedCells unordered_maps, check for collision
                   playerSnake and self. 
                */
                if (_predictedPlayerBlockedCells[nextTimeStep].count(nextHeadCell) ||
                    std::find(currentBodyCells.rbegin() + 1, currentBodyCells.rend(), nextHeadCell)
                      != currentBodyCells.rend()
                    ) {
                    return nullptr;
                }
            }

        }  // End checking if snake will reach next cell.

        // Since head will still be in the current cell, only check collision with Obstacles.
        if (_predictedObstacleBlockedCells.count(nextTimeStep)) {
                /* nextTimeStep exist in the blockedCells unordered_maps, check if any ObstacleSnakes will
                   collide with this snake or if this snake will collide with any obstacleSnake.
                */
                if (_predictedObstacleBlockedCells[nextTimeStep].count(nextHeadCell) ) {
                    /* Taking this direction from the current node will result in a collision with an 
                       ObstacleSnake or an ObstacleSnake will collide with this snake. Return nullptr 
                       to indicate this. 
                    */
                    return nullptr;
                }
        }
        else {
            /* nextTimeStep does not exist Generate more time steps enough to reach the goal(food) */
            GenerateBlockedCells(
                nextTimeStep,
                CalculateHeuristic(
                    nextHeadX, nextHeadY, speed, _food.GetCellX(), _food.GetCellY(),
                    _grid.GetWidth(), _grid.GetHeight()
                )
            );

            /* nextTimeStep Should Now exist in the blockedCells unordered_maps, check if any 
                ObstacleSnakes will collide with this snake or if this snake will collide with any 
                obstacleSnake.
            */
            if (_predictedObstacleBlockedCells[nextTimeStep].count(nextHeadCell)) { return nullptr; }
        }
    } // End of while loop moving snake and checking collisions.

    /* Snake will arrive in next cell with no collision againt or by ObstacleSnake and no collision with
       playerSnake or self when it take nextDirection from current position in current cell. return 
       shared_ptr to resulting Node.
    */
    return std::make_shared<Node>(
        nextHeadCell,
        nextTimeStep,
        nextTimeStep + CalculateHeuristic(
            nextHeadX, nextHeadY, speed, _food.GetCellX(), _food.GetCellY(), 
            _grid.GetWidth(), _grid.GetHeight()
        ),
        nextHeadX,
        nextHeadY,
        nextDirection,
        current
    );
    
}