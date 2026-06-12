//
// Created by west on 6/11/26.
//

#ifndef MOSSCAP_ASTARPLANNER_H
#define MOSSCAP_ASTARPLANNER_H

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <queue>
#include <unordered_map>

#include <util/Renderer.h>

#include "TelemetryMap.h"

struct Location {
    int x, y;

    bool operator==(const Location& other) const {
        return this->x == other.x && this->y == other.y;
    }
};

template <> struct std::hash<Location> {
    std::size_t operator()(const Location& id) const noexcept {
        return std::hash<int>()(id.x ^ (id.y << 16));
    }
};

class AStarPlanner : public Drawable {
public:
    AStarPlanner(const TelemetryMap& map)
        : Drawable(2), map(map) {}

    void reset(const Location s, const Location g) {
        std::lock_guard lk(drawMtx);

        // Clear frontier
        for (; !frontier.empty(); frontier.pop()) {}

        // Clear other tracking structures
        cameFrom.clear();
        costSoFar.clear();

        solved.store(false);
        path.clear();

        // Set up new search
        start = s;
        goal = g;

        frontier.emplace(0, start);

        cameFrom[start] = start;
        costSoFar[start] = 0;
    }

    std::vector<Location> solve() {
        while (!frontier.empty()) {
            // Wait until new frame
            std::unique_lock lk(drawMtx);
            cv.wait(lk, [this]{ return this->step; });

            if (takeSolutionStep()) {
                lk.unlock();
                break;
            }

            // Reset notification variable
            step = false;
            lk.unlock();
        }

        // Now we have to reconstruct the path
        path = reconstructPath();
        solved.store(true);

        return path;
    }

    bool takeSolutionStep();

    std::vector<Location> reconstructPath();

    void draw() const override {
        static int counter = 0;
        // Lock draw mutex
        std::unique_lock lk(drawMtx);

        // Copy data to draw
        auto f = frontier;

        step = true;

        // Unlock and notify solver to take another step
        lk.unlock();

        counter++;
        counter %= 5;

        if (counter == 0) {
            cv.notify_one();
        }

        // Draw the frontier in magenta
        for (; !f.empty(); f.pop()) {
            const auto [_, location] = f.top();
            const double rectX = location.x * map.getResolution() + (map.getResolution() / 2.0);
            const double rectY = location.y * map.getResolution() + (map.getResolution() / 2.0);
            Renderer::drawRect(rectX, rectY, 0.0, map.getResolution(), map.getResolution(), {1.0f, 0.0f, 1.0f, 0.33});
        }

        if (solved.load()) {
            for (const auto [x, y] : path) {
                const double rectX = x * map.getResolution() + (map.getResolution() / 2.0);
                const double rectY = y * map.getResolution() + (map.getResolution() / 2.0);
                Renderer::drawRect(rectX, rectY, 0.0, map.getResolution(), map.getResolution(), {0.0f, 1.0f, 1.0f, 0.33});
            }
        }
    }

private:
    const TelemetryMap& map;
    Location start{};
    Location goal{};

    typedef std::pair<double, Location> PQElement;

    struct PQComp {
        constexpr bool operator()(
            PQElement const& a,
            PQElement const& b
        ) const noexcept {
            return a.first > b.first;
        }
    };

    std::priority_queue<PQElement, std::vector<PQElement>, PQComp> frontier;

    std::unordered_map<Location, Location> cameFrom;
    std::unordered_map<Location, double> costSoFar;

    mutable std::condition_variable cv;
    mutable bool step{false};

    std::atomic_bool solved{false};
    std::vector<Location> path{};

    static constexpr std::array<Location, 8> neighbors = {{
        {1, 0},
        {1, 1},
        {0, 1},
        {-1, 1},
        {-1, 0},
        {-1, -1},
        {0, -1},
        {1, -1}
    }};

    bool isValidLocation(const Location l) const {
        return l.x >= 0 && l.x < map.getNumElements() && l.y >= 0 && l.y < map.getNumElements() && !map.get(l.x, l.y);
    }

    Location popFromFrontier() {
        const auto temp = frontier.top().second;
        frontier.pop();
        return temp;
    }

    double cost(const Location a, const Location b) const {
        // This function is assumed to be called on two cells that are neighbors.
        // Rather than doing a bunch of expensive square roots, we'll just take the cost
        // as the map resolution if it's a cardinal move and 1.5 * the map resolution if it's diagonal
        if (a.x == b.x || a.y == b.y) return map.getResolution();

        return 1.5 * map.getResolution();
    }

    static double heuristic(const Location a, const Location b) {
        // Here we kind of need the square root
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }
};

#endif //MOSSCAP_ASTARPLANNER_H
