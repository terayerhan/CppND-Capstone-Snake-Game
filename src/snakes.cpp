#include "snakes.h"

#include <queue>
#include <iostream>

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
    const float headX, const float headY, const float speed,
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




void AISnake::PredictPlayerBlockedCells(std::size_t initialTimeStep, std::size_t maxTimeStep) {
    // Predict playerSnake blocked cells [potential loop for concurrency]
    for(std::size_t i = initialTimeStep; i < maxTimeStep; i++) {
        _predictedPlayerSnake.Update();   // simulate a single time step move.

        // insert the resulting body cells after the step into the unordered_set of player snake blocked cells 
        // at time step i in the unordered_map for the predictedPlayerBlockedCells
        _predictedPlayerBlockedCells[i].insert(
            _predictedPlayerSnake.GetBodyCells().begin(), _predictedPlayerSnake.GetBodyCells().end()
        );
    }
}



void AISnake::PredictObstacleBlockedCells(std::size_t initialTimeStep, std::size_t maxTimeStep) {
    // Add a buffer amount of timeSteps to allow the snake's body to pass in front of an obstacle snake after 
    // it eats the food. This is because if obstacleSnake blocked cells are only predicted up to the time the
    // head of the aiSnake reaches the food, the aiSnake will not accont the collision of the obstacles with its
    // body after it reaches the food.
    /* The buffer amount of timeSteps to add to the maxTimeStep for ObstacleBlockedCells prediction is 
       1 + the current size of the snake because the snake will increase in size by one when it eats the food
       divde by the speed the snake will have after it eats the food because when the snake eats the food, the 
       amount of timeSteps it will take for the body to clear the head of the obstacle snake will be dependent on
       the current speed plus the increase in speed after eating food which is constant(_delta_speed) for each snake.        
    */
    maxTimeStep += (_body_cells.size() + 1) / (GetSpeed() + GetDeltaSpeed());
    
    // Predict obstacle Snakes blocked cells
    for(ObstacleSnake& obstacle : _predictedObstacles) {
        // [potential loops for concurrency]
        for(std::size_t i = initialTimeStep; i < maxTimeStep; i++) {
            obstacle.Update();   // simulate a single time step move.

            // Insert the resulting body_cells after the step into the unordered_set of obstacle snakes
            // blocked cells at time step i in the unordered_map for the predictedObstaclesBlocked cells
            _predictedObstaclesBlockedCells[i].insert(
                obstacle.GetBodyCells().begin(), obstacle.GetBodyCells().end()
            );
        }
    }
}



std::shared_ptr<Node> AISnake::AddNode( std::shared_ptr<Node> current, Direction nextDirection, 
    std::deque<SDL_Point> currentBodyCells
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
    
    SDL_Point goal = _food.GetPosition();     // Cach food position.

    float speed = GetSpeed();  // Use the Snake's speed to determine the distance traveled by snake per step.

    // While still in the current nodes's cell, keep stepping forward till you reach the next cell.
    while(nextHeadCell == currentCell) {
        // Move one step in the nextDirection.
       /*  std::cout<< "Simulating AISnake movement"<< 
        "  nextHeadCell:"<< nextHeadCell.x << "  " << nextHeadCell.y <<
        "  currentCell:"<< currentCell.x << "  " << currentCell.y <<
        "  Speed" << speed  
        <<std::endl; */
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
           position hence, this can affect the result o f the collision check. If the cells are not
           moved appropraitely, more optimal paths could be blocked.
        */

        // Check if the snake will reach the next cell after the step.
        if (nextHeadCell != currentCell) {
            // Snake has arrived in next cell.//

            if ( nextHeadCell != goal) {
                // Tail is at the back of currentBodyCells.            
                currentBodyCells.pop_back();                 // Remove tail from back of deque.
            }

            currentBodyCells.push_front(nextHeadCell);   // Add head to front of deque.

            if (std::find(
                currentBodyCells.begin() + 1, currentBodyCells.end(), nextHeadCell)
                != currentBodyCells.end()
            ) {
                    std::cout << "Predicted Self_Collision before Checking with player "<< std::endl;
                    return nullptr;
            }

            // Check for collision with playerSnake 
            /* Check if nextTimeStep is a key in  the unordered_map of  _predictedPlayerBlockedCells
               (which also implies in same for _predictedObstacleBlockedCells since the max time step 
               of prediction is same for both). If the time step is not in the unordered_map, 
               Generate more predicted blocked cells for both playerSnake and obstacle snakes up to
               the heuristic of the nextCell. 
            */
            if (_predictedPlayerBlockedCells.count(nextTimeStep)) {
                /* nextTimeStep exist in the blockedCells unordered_maps, check for collision
                   playerSnake and self. 
                */
                if (_predictedPlayerBlockedCells[nextTimeStep].count(nextHeadCell) ) {
                    // Taking this direction from the current node will result in a collision with either 
                    // player sanke or self. Return nullptr to indicate this.
                    return nullptr;
                }
            }
            else {
                /* nextTimeStep does not exist Generate more time steps enough to reach the goal(food) */
                PredictObstacleBlockedCells(
                    nextTimeStep,
                    CalculateHeuristic(
                        nextHeadX, nextHeadY, speed, _food.GetCellX(), _food.GetCellY(),
                        _grid.GetWidth(), _grid.GetHeight()
                    )
                );

                PredictPlayerBlockedCells(
                    nextTimeStep,
                    CalculateHeuristic(
                        nextHeadX, nextHeadY, speed, _food.GetCellX(), _food.GetCellY(),
                        _grid.GetWidth(), _grid.GetHeight()
                    )
                );

                /* nextTimeStep Should Now exist in the blockedCells unordered_maps, check for collision
                   playerSnake. 
                */
                if (_predictedPlayerBlockedCells[nextTimeStep].count(nextHeadCell) ) {
                    std::cout << "Predicted Collision with PlayerSnake after adding more TimeSteps "<< std::endl;                                           
                    return nullptr;
                }
            }

        }  // End checking if snake will reach next cell.


        // Check obstacle collision with aiSnake and aiSnake collision with obstacles at nextTimeStep.
        for (const SDL_Point& cell : currentBodyCells) {
            if (_predictedObstaclesBlockedCells[nextTimeStep].count(cell) ) {
                /* Taking this direction from the current node will result in a collision with an 
                    ObstacleSnake or an ObstacleSnake will collide with this snake. Return nullptr 
                    to indicate this. 
                */
                return nullptr;
            }
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


/**
 * @brief Finds a path from the snake's head to the food using an A* search algorithm.
 *
 * This method uses a priority queue (open list) to perform an A* search over a grid that is
 * wrap-around (toroidal topology). 
 *
 * Key steps:
 *  - Build a closedList of dummy nodes representing the snake's initial body position on the grid.
 *  - Initialize the open list with a start node, based on the snake head linked to its reconstructed body.
 *  - Loop until the open list is empty or the goal (food) is reached.
 *  - For each node, if the goal is reached, backtrack to reconstruct the path directions and cells.
 *  - Otherwise, reconstruct the current body configuration and explore all possible directions.
 *  - Skip the direction that would reverse the snake (if snake size > 1) to avoid self-collision.
 *  - For each possible direction, attempt to add a new node; if successful, push it to the open list.
 *
 * The grid uses half-open intervals [n, n+1) for cell determination. For example, a coordinate
 * of 1.99999 is in cell 1, while a coordinate of 2.0000 is in cell 2.
 */
void AISnake::FindPath() {
    std::cout << "Started FindPath()"<< std::endl;
    // Priority queue for open set.
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare> openList;

    /*  To enable backtracking by snake size[- when snake size is greater than 1 otherwise there is no need] 
        to reconstruct the cells of the snake, create dummy Node shared_ptrs from the body cells that are not 
        the head; linking them from the tail as base parent up to the neck which will then serve as the parent 
        to the head based Node that will be placed on the openList as the start Node for the A*Search based 
        path planning. The parent of the tail based node will be nullptr.
    */
    std::vector<std::shared_ptr<Node>> closedList;  // vector to store the dummy node shared pointers.
    std::size_t fCost_Max = std::numeric_limits<std::size_t>::max();  // for use as fCost of all dummy nodes.

    // Check if size of snake's body Cells is greater than one otherwise don't create dummy node pointers.
    std::size_t snakeSize = _body_cells.size();
    if (snakeSize > 1) {
        closedList.reserve(snakeSize - 1); // Preallocate memory for all non-head nodes

        // Create nodes from tail to neck
        std::shared_ptr<Node> parent = nullptr; // Tail node will have nullptr as parent
        for (std::size_t i = snakeSize - 1; i > 0; i--) {  // Start from the back(tail) of the body (deque).
            closedList.emplace_back(
                std::make_shared<Node>(
                    _body_cells[i],     // The cell position.
                    0,                  // gCost is 0(same as start node) for dummy nodes.
                    fCost_Max,          // fCost is set to the maximum for dummy nodes. 
                    0.0f, 0.0f,         // Head position (dummy values here).
                    _direction,         // Initial Snake Direction is maintained by dummy nodes. Its irrelvant.
                    parent  
                )
            );

            parent = closedList.back(); // Update parent for next iteration            
        }
    }

    SDL_Point goal = _food.GetPosition(); // Cach the food location as goal cell.

    float speed = GetSpeed();
    std::cout << "Speed before caclulating initial time steps" << speed << std::endl;

    // Get the initial number of steps it will take to get to the goal if there were no blocked cells.
    std::size_t initialMaxTimeSteps = CalculateHeuristic(
        _head_x, _head_y, GetSpeed(), goal.x, goal.y, _grid.GetWidth(), _grid.GetHeight()
    );

    // Re-initialize the predicted snake object for simulating move steps to predict blocked cells.
    _predictedObstacles = _obstacles;
    _predictedPlayerSnake = _playerSnake;

    // Clear the maps of previouly predicted blocked cells and predict fresh ones.
    _predictedObstaclesBlockedCells.clear();
    _predictedPlayerBlockedCells.clear();

    PredictObstacleBlockedCells(0, initialMaxTimeSteps);
    PredictPlayerBlockedCells(0, initialMaxTimeSteps);

    // Create and add the start node to the open list.
    // The start node is based on the snake's head and uses the last dummy node from the body as its parent.
    openList.push(
        std::make_shared<Node>(
            _body_cells.front(), 
            0,                           // Inititial time step from when new path needs to be recalculated.
            initialMaxTimeSteps,
            _head_x,
            _head_y,
            _direction,
            closedList.empty() ? nullptr : closedList.back()  // The neck (or last dummy node) becomes the parent of the head node.
        )
    );

    bool notFirstIteration = false;

    // Begin A* search: process nodes until the open list is empty.
    while (!openList.empty()) {
        std::shared_ptr<Node> current = openList.top();  // Get the node ptr with the least fCost.
        openList.pop();                                  // Remove the node ptr from the openList.

        // Check if goal(food) has been reached.
        if (current->cell_ == goal) {
            // Reconstruct the path cells and the directions to take at each step in the game loop.//
            // Clear any previous path directions and cells.
            _pathDirections.clear();   
            _pathCells.clear();        

            // Backtrack till start node (first node going backwards that have gCost == 0) is reached.
            std::shared_ptr<Node> nodePtr = current;
            while (nodePtr->gCost_ > 0) {
                std::shared_ptr<Node> parentPtr = nodePtr->parent_; 
                Direction directionToNode = nodePtr->direction_;   // Direction from the parent to this node.

                // Determine the number of steps needed from parent node to this node via direction to Node.
                std::size_t stepsInDirection = nodePtr->gCost_ - parentPtr->gCost_;

                // Append the same direction repeatedly based on the number of steps.
                for ( std::size_t i=0; i<stepsInDirection; i++) {
                    _pathDirections.push_back(directionToNode);
                }

                // Append the cell(of nodePtr) the snake will arrive at when it take those steps.
                _pathCells.push_back(nodePtr->cell_);

                nodePtr = parentPtr;   // Move to the parent node to continue backtracking.
            }
            std::cout << "Path Found -- End FindPath()"<< std::endl;
            return;  // Path found, exit the function.

        } 


        /* Reconstruct the snake's body when its head is at the current node's cell. For times when 
           the snake is longer than one cell and it head is not at a node with a cell position that
           is not more than the snake's body size cell moves from its search  starting head cell, 
           this is when the nodes constructed from the body cells come into use.

           The reconstructed body is then used to check for collisions with obstacle snakes, player
           snake, and the snakes head agaist its rest of its body when exploring nodes to add to the
           open list in the up, down, left and right directions. The reconstructed body is left with 
           the tail at the back of the deque.

           The game logic dictect that a player or AI snake is penelized when it collides with an 
           obstacle snake or when the obstacle snake collides with them. For a player or ai snake 
           is only penelize if it runs into the other's body not when the other runs into its body. 
        */
        std::deque<SDL_Point> currentBodyCells;  // Tail is at back of deque. current.cell_ is head cell. 
        std::shared_ptr<Node> nodePtr = current;

        for (std::size_t i=0; i<snakeSize; i++) {
            currentBodyCells.push_back(nodePtr->cell_);
            nodePtr = nodePtr->parent_;
        }


        // Create a vector of possible Directions the snake can move in at any given cell position.
        std::vector<Direction> possibleDirections {
            Direction::kUp, 
            Direction::kDown, 
            Direction::kLeft, 
            Direction::kRight
        };


        /* Explore neighbor nodes in each possible direction. That is: explor moving to the next 
           node through each direction from current node float head position.
        */
        for (const Direction nextDirection : possibleDirections) {
            // If the snake is longer than one segment, check if the next direction is
            // the opposite of the current node's direction (i.e., the direction towards the snake's neck).
            // This check prevents the snake from reversing onto itself.
            if (
                (snakeSize > 1 && nextDirection == OppositeDirection(current->direction_)) || 
                (notFirstIteration && nextDirection == OppositeDirection(current->direction_)) ) { 
                continue; // Skip the reverse direction.
            }

            // Attempt to create a new node in the specified nextDirection
            std::cout<< "AddNode() started"<<"  "<< "  CurrentCell: "<< "  " <<
            current->cell_.x << "  "<< current->cell_.y << 
            "  gCost: " << current->gCost_ <<
            "  fCost: " << current->fCost_ <<
            "  Goal: " << goal.x << " " << goal.y<< 
            "  head: " << current->headX_ << "  "<< current->headY_ <<
            "  Speed: "<< GetSpeed() 
            //"  parentCell: " << current->parent_ != nullptr ? current->parent_->cell_.x :   << "  " <<current->parent_->cell_.y
            <<  "   NextDirection: "<< static_cast<int>(nextDirection)<< std::endl;
            std::cout << "  currentDirection: "<< static_cast<int>(current->direction_)<<
            "  oppositeDir: " << static_cast<int>(OppositeDirection(current->direction_)) << std::endl;
            std::shared_ptr<Node> nextNodePtr = AddNode(current, nextDirection, currentBodyCells);
            

            // If the node was successfully created (i.e., not null), add it to the open list.
            if (nextNodePtr != nullptr) {
                std::cout<< "AddNode() End    NextCell: "<< nextNodePtr->cell_.x<< "  " << nextNodePtr->cell_.y <<
                "  gCost: " << nextNodePtr->gCost_ << "  fCost: " << nextNodePtr->fCost_ <<
                "  head: " << nextNodePtr->headX_ << "  "<< nextNodePtr->headY_ 
                << std::endl;
                // If the node is valid, push its pointer onto the open list (a priority queue).
                // The open list is used in pathfinding algorithms (e.g., A*) to store nodes that 
                // are pending exploration.
                openList.push(nextNodePtr); // Enqueue the node for further processing.
            }
            else{
                std::cout<< "AddNode() End    NextCell:  InVAlid  "<< std::endl;
            }
        }

        notFirstIteration = true;

    } 

    // If the open list is exhausted without reaching the goal, no path was found.
    // (Optional: print or handle the "Path NOT Found" case.)
    // std::cout << "Path NOT Found" << std::endl;

}



void AISnake::SetDirection(bool IsPlayerSnakeChanged, bool IsFoodChanged) {
    // Check if state of food or playerSnake has change. That is food may have been eaten by player or
    // or obstacle snake or player snake may have changed direction and or eaten food. if any of these
    // happen, there is a chance that the current path has become invalidated, hence create new path plan.
    if(IsPlayerSnakeChanged || IsFoodChanged) {
        FindPath();            // RE-Calcualte path plan.
    }

    // Set Direction to the last direction in the _directions vector.
    _direction = _pathDirections.back();

    // Remove the current direction from the _pathDirections vector so that the next time this method is 
    // called, the last element will be direction the AISnake will take next.
    _pathDirections.pop_back();
}



void ObstacleSnake::InitializeBody(std::size_t initialLength) {
    // Clear any existing body cells.
    _body_cells.clear();

    // Create the head cell from the current head coordinates.
    SDL_Point head { static_cast<int>(_head_x), static_cast<int>(_head_y) };
    _body_cells.push_front(head);  // Head is at the front.

    // Create the rest of the body.
    // We'll place each new segment behind the head, depending on the starting direction.
    for (std::size_t i = 1; i < initialLength; ++i) {
        SDL_Point segment = head;
        switch (_direction) {
            case Direction::kUp:
                // If moving up, the body is below the head.
                segment.y += static_cast<int>(i);
                break;
            case Direction::kDown:
                // If moving down, the body is above the head.
                segment.y -= static_cast<int>(i);
                break;
            case Direction::kLeft:
                // If moving left, the body is to the right of the head.
                segment.x += static_cast<int>(i);
                break;
            case Direction::kRight:
                // If moving right, the body is to the left of the head.
                segment.x -= static_cast<int>(i);
                break;
        }
        // Add new segments at the back so that the head remains at the front.
        _body_cells.push_back(segment);
    }
}
