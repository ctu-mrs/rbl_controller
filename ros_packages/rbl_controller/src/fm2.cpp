#include <vector>
#include <queue>
#include <cmath>
#include <utility>
#include <limits>
#include <algorithm>

class FastMarchingSquare {
public:
    FastMarchingSquare(int rows, int cols);
    void initializeCostMap(const std::vector<std::vector<double>> &cost_map);
    std::vector<std::pair<int, int>> computePath(const std::pair<int, int> &start,
                                                 const std::pair<int, int> &goal);

private:
    int rows_, cols_;
    std::vector<std::vector<double>> cost_map_;
    std::vector<std::vector<double>> distance_map_;
    std::vector<std::vector<bool>> visited_;

    struct Node {
        int x, y;
        double cost;

        bool operator>(const Node &other) const {
            return cost > other.cost;
        }
    };

    bool isValid(int x, int y);
    double getCost(int x, int y);
};

FastMarchingSquare::FastMarchingSquare(int rows, int cols)
    : rows_(rows), cols_(cols),
      cost_map_(rows, std::vector<double>(cols, 1.0)),
      distance_map_(rows, std::vector<double>(cols, std::numeric_limits<double>::infinity())),
      visited_(rows, std::vector<bool>(cols, false)) {}

void FastMarchingSquare::initializeCostMap(const std::vector<std::vector<double>> &cost_map) {
    cost_map_ = cost_map;
}

bool FastMarchingSquare::isValid(int x, int y) {
    return x >= 0 && x < rows_ && y >= 0 && y < cols_;
}

double FastMarchingSquare::getCost(int x, int y) {
    return isValid(x, y) ? cost_map_[x][y] : std::numeric_limits<double>::infinity();
}

std::vector<std::pair<int, int>> FastMarchingSquare::computePath(const std::pair<int, int> &start,
                                                                 const std::pair<int, int> &goal) {
    // Priority queue for wavefront propagation
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    // Initialize start node
    pq.push({start.first, start.second, 0.0});
    distance_map_[start.first][start.second] = 0.0;

    // Neighbor offsets for 8-connected grid
    std::vector<std::pair<int, int>> directions = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        // If the node is already visited, skip it
        if (visited_[current.x][current.y]) {
            continue;
        }
        visited_[current.x][current.y] = true;

        // If the goal is reached, stop the propagation
        if (current.x == goal.first && current.y == goal.second) {
            break;
        }

        // Propagate to neighbors
        for (const auto &[dx, dy] : directions) {
            int nx = current.x + dx;
            int ny = current.y + dy;

            if (isValid(nx, ny) && !visited_[nx][ny]) {
                double new_cost = distance_map_[current.x][current.y] + getCost(nx, ny);

                if (new_cost < distance_map_[nx][ny]) {
                    distance_map_[nx][ny] = new_cost;
                    pq.push({nx, ny, new_cost});
                }
            }
        }
    }

    // Traceback from goal to start to extract the path
    std::vector<std::pair<int, int>> path;
    int x = goal.first, y = goal.second;

    if (visited_[x][y]) { // Check if goal is reachable
        while (!(x == start.first && y == start.second)) {
            path.push_back({x, y});

            // Find the neighbor with the smallest distance
            double min_distance = std::numeric_limits<double>::infinity();
            std::pair<int, int> next_node = {x, y};

            for (const auto &[dx, dy] : directions) {
                int nx = x + dx;
                int ny = y + dy;

                if (isValid(nx, ny) && distance_map_[nx][ny] < min_distance) {
                    min_distance = distance_map_[nx][ny];
                    next_node = {nx, ny};
                }
            }

            x = next_node.first;
            y = next_node.second;
        }

        path.push_back(start); // Add the start position
        std::reverse(path.begin(), path.end());
    }

    return path;
}
