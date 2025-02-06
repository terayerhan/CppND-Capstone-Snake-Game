#include "snakes.h"

#include <queue>

/**
 * @brief Calculates the heuristic (Manhattan distance in steps) from the snake's head to a goal cell,
 *        accounting for grid wrap-around.
 *
 * This method computes the minimum number of game update steps required for the snake to reach the goal cell.
 * It does so by calculating the required number of steps along the x-axis and y-axis and then summing them.
 *
 * The grid is defined using half-open intervals: each cell along the x- or y-axis spans the interval [n, n+1),
 * where n is an integer cell index. In other words, a coordinate is considered to be in cell n if it satisfies:
 *
 *      n <= coordinate < n+1.
 *
 * For example, a coordinate of 1.99999 lies in cell 1, while a coordinate of 2.0000 lies in cell 2.
 *
 * The method also accounts for grid wrap-around (toroidal behavior). When the snake moves off one edge,
 * it reappears on the opposite side. This behavior is handled by comparing both the direct (positive) and 
 * wrap-around (negative) distances along each axis.
 *
 * @param headX      The x-coordinate of the snake's head (fractional position).
 * @param headY      The y-coordinate of the snake's head (fractional position).
 * @param speed      The essential distance per step (game update) along one axis that the snake moves.
 *                   For example, if speed is 1, then each game update moves the snake exactly one cell.
 * @param goalX      The x-index of the goal cell.
 * @param goalY      The y-index of the goal cell.
 * @param gridWidth  The total number of cells along the x-axis in the grid.
 * @param gridHeight The total number of cells along the y-axis in the grid.
 *
 * @return The Manhattan distance in steps (the sum of the minimum steps along both the x-axis and y-axis)
 *         from the snake's current head position to the goal cell.
 */
size_t AISnake::CalculateHeuristic( 
    const float headX, const float headY, const float& speed,
    const int goalX, const int goalY, const int gridWidth, const int gridHeight
) const {
    // Determine the current grid cell of the snake's head.
    // Casting the head coordinates to int effectively applies the floor function for non-negative values,
    // which aligns with our half-open interval definition: a coordinate x is in cell n if n <= x < n+1.
    int headCellX = static_cast<int>(headX);
    int headCellY = static_cast<int>(headY);

    std::size_t xSteps = 0;
    std::size_t ySteps = 0;

    // Calculate the number of steps along the x-axis.
    // The grid wraps around, so we consider two distances:
    //   - pdx: the positive-direction distance to the goal cell.
    //   - ndx: the negative-direction distance to the goal cell.
    if (goalX > headCellX) {
        float pdx = goalX - headX;                   // Positive direction: distance from head to goal.
        float ndx = headX - goalX - 1 + gridWidth;   // Negative direction: wrapping around the grid.
        
        // When pdx equals ndx, the right cell edge exclusion (due to the half-open interval)
        // makes the effective negative distance larger. Hence, we add 1 step if needed i.e[
        // for ndx, if snake lands exactly on the right-edge, i.e actual distance == ndx, then it
        // will need one more step - no matter how small (snakes can't move more than one cell
        // distance per step) to reach the goal].
        xSteps = pdx <= ndx ? ceil(pdx / speed) : static_cast<int>((ndx / speed) + 1); 
    }
    else if (goalX < headCellX) {
        float pdx = goalX - headX + gridWidth;        // Positive direction considering wrap-around.
        float ndx = headX - goalX - 1;                // Negative direction: direct movement.
        
        xSteps = pdx <= ndx ? ceil(pdx / speed) : static_cast<int>((ndx / speed) + 1);
    }

    // Calculate the number of steps along the y-axis using similar logic to x-axis.
    if (goalY > headCellY) {
        float pdy = goalY - headY;                     
        float ndy = headY - goalY - 1 + gridHeight;     
        
        ySteps = pdy <= ndy ? ceil(pdy / speed) : static_cast<int>((ndy / speed) + 1); 
    }
    else if (goalY < headCellY) {
        float pdy = goalY - headY + gridHeight;        
        float ndy = headY - goalY - 1;                   
        
        ySteps = pdy <= ndy ? ceil(pdy / speed) : static_cast<int>((ndy / speed) + 1);
    }

    // The heuristic is the Manhattan distance: the sum of the minimum number of steps
    // along both the x-axis and y-axis to reach the goal.
    return xSteps + ySteps;
}



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
                    std::find(currentBodyCells.begin() + 1, currentBodyCells.end(), nextHeadCell)
                      != currentBodyCells.end()
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



void AISnake::FindPath() {
    // Priority queue for open set.
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare> openList;

    /*  To enable backtracking by snake size[- when snake size is greater than 1 otherwise there is no need] 
        to reconstruct the cells of the snake, create dummy Node shared_ptrs from the body cells that are not 
        the head; linking them from the tail as base parent up to the neck which will then serve as the parent 
        to the head based Node that will be placed on the openList as the start Node for the A*Search based 
        path planning. The parent of the tail based node will be nullptr which will be the first element in
        the closedList of Node pointer.
    */
    std::vector<std::shared_ptr<Node>> closedList{nullptr};  // vector to store the dummy node pointers.
    std::size_t fCost_Max = std::numeric_limits<std::size_t>::max();  // for use as fCost of all dummy nodes.

    // Check if size of snake's body Cells is greater than one otherwise don't create dummy node pointers.
    std::size_t snakeSize = _body_cells.size();
    if (snakeSize > 1) {
        for (std::size_t i=0; i<snakeSize; i++) {
            if (i > 0) {
                // Add non-tail based dummy nodes pointers.
                closedList.push_back(
                    std::make_shared<Node>(
                        _body_cells[i], 0, fCost_Max, 0.0f, 0.0f, _direction,
                        closedList[i-1] // Previous Node(next one closer to tial Node) is the node's parent. 
                    )
                );
            }
            else {
                // Add node pointer base on the tail cell of the snake.
                closedList.push_back(
                    std::make_shared<Node>(
                        _body_cells[i], 0, fCost_Max, 0.0f, 0.0f, _direction,
                        nullptr  // Only the tail cell based node has a nullptr parent.
                    )
                );
            }
        }
    }

    SDL_Point goal = _food.GetPosition(); // Cach the food location as goal.

    // Add the start node to the open list
    openList.push(
        std::make_shared<Node>(
            _body_cells.back(), 
            0, 
            CalculateHeuristic(
                _head_x, _head_y, GetSpeed(), goal.x, goal.y, _grid.GetWidth(), _grid.GetHeight()
            ),
            _head_x,
            _head_y,
            _direction,
            closedList[snakeSize-1]
        )
    );


    while (!openList.empty()) {
        std::shared_ptr<Node> current = openList.top();  // Get the node ptr with the least fCost.
        openList.pop();                                  // Remove the node ptr from the openList.

        // Check if goal has been reached.
        if (current->cell_ == goal) {
            // Reconstruct the path cells and the directions to take at each step.
            _pathDirections.clear();   // Clear the vector before adding the new directions.
            _pathCells.clear();        // Clear the vector before adding the new cells.

            // Backtrack till start node (first node going backwards that have gCost == 0) is reached.
            std::shared_ptr<Node> nodePtr = current;
            while (nodePtr->gCost_ > 0) {
                std::shared_ptr<Node> parentPtr = nodePtr->parent_; 
                Direction directionToNode = nodePtr->direction_;   // Direction the snake will need to take to reach node.

                // Calculate the steps needed from parent node to this node using via direction to Node.
                std::size_t stepsInDirection = nodePtr->gCost_ - parentPtr->gCost_;

                // Append directionToNode stepsInDirection times to the _path directions vector.
                for ( std::size_t i=0; i<stepsInDirection; i++) {
                    _pathDirections.push_back(directionToNode);
                }

                // Append the cell(of nodePtr) the snake will arrive at when it take those steps.
                _pathCells.push_back(nodePtr->cell_);

                nodePtr = parentPtr;   // make parent pointer the current node ptr.
            }

            return;  // Stop the search. //

        } // end of check for reaching goal.


        /* Reconstruct the snake's body when its head is at the current node's cell. For times when 
           the snake is longer than one cell and it head is not at a node with a cell position that
           is not more than the snake's body size cell moves from its search  starting head cell, 
           this is when the nodes constructed from the body cells come into use.

           The reconstructed body is then used to check for collisions with obstacle snakes, player
           snake, and the snakes head agaist its rest of its body when exploring nodes to add to the
           open list in the up, down, left and right directions. The reconstructed body is left with 
           the tail at the back of vector

           The game logic detect that a player or AI snake is penelized when it collides with an 
           obstacle snake or when the obstacle snake collides with them. For a player or ai snake 
           is only penelize if it runs into the other's body not when the other runs into its body. 
        */
        std::vector<SDL_Point> currentBody;  // Tail is at back of vector. vector is delibrately not reversed.
        std::shared_ptr<Node> nodePtr = current;

        for (std::size_t i=0; i<snakeSize; i++) {
            currentBody.push_back(nodePtr->cell_);
            nodePtr = nodePtr->parent_;
        }


        // Create a vector of possible Directions the snake can move in at any given cell position.
        std::vector<Direction> possibleDirections {
            Direction::kUp, Direction::kDown, Direction::kLeft, Direction::kRight
        };


        /* // Explore neighbors. //
           Explor moving to the next node through each direction from current node float head position.
        */
        for (const Direction nextDirection : possibleDirections) {
            // If the snake is longer than one segment, check if the next direction is
            // the same as the current node's direction (i.e., the direction of the snake's neck).
            // This check prevents the snake from reversing onto itself.
            if (snakeSize > 1 && nextDirection == current->direction_) { 
                continue; // Skip this direction.
            }

            // Check validity of nextDirection to add node.
            // Attempt to create a new node in the specified nextDirection
            std::shared_ptr<Node> nextNodePtr = AddNode(current, nextDirection, currentBody);

            // If the node was successfully created (i.e., not null), add it to the open list.
            if (nextNodePtr != nullptr) {
                // If the node is valid, push its pointer onto the open list (a priority queue).
                // The open list is used in pathfinding algorithms (e.g., A*) to store nodes that 
                // are pending exploration.
                openList.push(nextNodePtr); // Enqueue the node for further processing.
            } 
        }

    } 

    // std::cout << "Path NOT Found" << std::endl;

}