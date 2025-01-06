#ifndef FAST_MARCHING_SQUARE_H
#define FAST_MARCHING_SQUARE_H

#include <vector>
#include <utility>

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

#endif // FAST_MARCHING_SQUARE_H
