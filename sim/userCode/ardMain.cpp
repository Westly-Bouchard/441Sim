#include "mosscap/challenge5.h"

void setup() {
    // Mapping button
    pinMode(1, INPUT);

    // Planning button
    pinMode(2, INPUT);

    // Set initial position
    pose.setX(0.5);
    pose.setY(0.5);
}

AStarPlanner planner(map);

bool AStarPlanner::takeSolutionStep() {
    // Get the next node to check from the frontier
    Location current = popFromFrontier();

    // If we're at the goal, return true
    if (current == goal) {
        return true;
    }

    // For each of the node's neighbors, including diagonals
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx != 0 || dy != 0) {
                // Compute the location of the neighbor
                Location next(current.x + dx, current.y + dy);

                // If that location is within the bounds of our map
                if (next.x >= 0 && next.x < map.getNumElements() && next.y >= 0 && next.y < map.getNumElements()) {
                    // And, if there is no obstacle at that location
                    if (!map.get(next.x, next.y)) {
                        // Calculate the cost to this location
                        double newCost = costSoFar.at(current) + cost(current, next);

                        // If we've never visited this cell before, or if this cost is lower than the previous cost at this location
                        if (!costSoFar.contains(next) || newCost < costSoFar.at(next)) {
                            // Update the cost in our dictionary
                            costSoFar[next] = newCost;

                            // Calculate the priority for this node
                            double priority = newCost + heuristic(next, goal);

                            // Place the node into the frontier
                            frontier.emplace(priority, next);

                            // Update our `cameFrom` dictionary so that we know which node is this node's parent
                            cameFrom[next] = current;
                        }
                    }
                }
            }
        }
    }

    return false;
}

vector<Location> AStarPlanner::reconstructPath() {
    // If we didn't end up finding a valid path, we can just return an empty vector
    if (!cameFrom.contains(goal)) {
        return {};
    }

    // We trace our path backwards from the goal, so we start at the goal
    Location current = goal;

    // Stores our path
    vector<Location> path;

    // While we're not at the start
    while (current != start) {
        // Put the current node in our path
        path.push_back(current);

        // Update the current node to this node's parent
        current = cameFrom.at(current);
    }

    // Now we have to reverse the path so we can follow it from start to finish
    vector<Location>  pathReversed;

    for (int i = path.size() - 1; i >= 0; i--) {
        pathReversed.push_back(path[i]);
    }

    return pathReversed;
}

void loop() {
    // Wait for user to press the button
    while (!digitalRead(1)) {}
    while (digitalRead(1)) {}

    performMappingSweep();

    while (!digitalRead(2)) {}
    while (digitalRead(2)) {}

    Location current{5, 5};
    Location goal {18, 26};

    planner.reset(current, goal);
    auto _ = planner.solve();
}