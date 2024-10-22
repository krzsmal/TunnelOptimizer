#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ctime>
#include <fstream>
#include <string>
#include "nlohmann/json.hpp"
#include <chrono>

using namespace std;
using json = nlohmann::json;
using namespace chrono;

// Function to read instances from a JSON file
void readFromJson(vector<vector<double>>& points, vector<pair<vector<double>, double>>& obstacles, const string& path) {
    ifstream file(path);
    if (!file.is_open()) {
        cerr << "Unable to open file: " << path << endl;
        return;
    }

    try {
        json data;
        file >> data;

        if (data.find("points") != data.end() && data.find("obstacles") != data.end()) {
            points = data["points"].get<vector<vector<double>>>();
            obstacles.clear();
            for (const auto& obstacle : data["obstacles"]) {
                vector<double> obstacle_points = obstacle[0].get<vector<double>>();
                double obstacle_size = obstacle[1].get<double>();
                obstacles.emplace_back(obstacle_points, obstacle_size);
            }
        } else {
            cerr << "Invalid JSON format. Missing 'points' or 'obstacles' key." << endl;
        }
    } catch (const json::exception& e) {
        cerr << "Error while parsing JSON: " << e.what() << endl;
    }

    file.close();
}

// Function to save results to a JSON file
void saveToJson(const vector<int>& times, const vector<pair<vector<int>, double>>& results, const string& path) {
    json data;
    data["times"] = times;
    json resultsArray;
    for (const auto& result : results) {
        json resultObject;
        resultObject.push_back(result.first);
        resultObject.push_back(result.second);
        resultsArray.push_back(resultObject);
    }
    data["results"] = resultsArray;
    ofstream file(path);
    file << data.dump(2);
    file.close();
}

// Function to check collision
bool lineSegmentSphereIntersection(const vector<double>& P1, const vector<double>& P2, const vector<double>& C, double r) {
    if (P1 == P2) {
        return false;
    }

    vector<double> direction(3), offset(3);

    for (int i = 0; i < 3; ++i) {
        direction[i] = P2[i] - P1[i];
    }

    for (int i = 0; i < 3; ++i) {
        offset[i] = P1[i] - C[i];
    }

    double a = direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2];
    double b = 2 * (offset[0] * direction[0] + offset[1] * direction[1] + offset[2] * direction[2]);
    double c = offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2] - r * r;

    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return false;
    }

    double t1 = (-b + sqrt(discriminant)) / (2 * a);
    double t2 = (-b - sqrt(discriminant)) / (2 * a);

    if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
        return true;
    }

    return false;
}

// Function to calculate distance between points
double calculateDistance(const vector<double>& P1, const vector<double>& P2) {
    double distance = 0.0;
    for (int i = 0; i < 3; ++i) {
        distance += (P1[i] - P2[i]) * (P1[i] - P2[i]);
    }
    return sqrt(distance);
}

// Function to validate the path
bool checkPath(const vector<vector<double>>& path, const vector<pair<vector<double>, double>>& obstacles) {
    for (size_t i = 0; i < path.size() - 1; ++i) {
        for (const auto& obstacle : obstacles) {
            if (lineSegmentSphereIntersection(path[i], path[i + 1], obstacle.first, obstacle.second)) {
                return false;
            }
        }
    }
    return true;
}

// Function to calculate the total distance of a path
double calculateTotalDistance(const vector<int>& path, const vector<vector<double>>& points) {
    double distance = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        distance += calculateDistance(points[path[i]], points[path[i + 1]]);
    }
    return distance;
}

// Algorithm A - based on brute force
pair<vector<int>, double> bruteforce(const vector<vector<double>>& points, const vector<pair<vector<double>, double>>& obstacles) {
    int number_of_points = points.size();
    vector<int> points_indices(number_of_points);
    iota(points_indices.begin(), points_indices.end(), 0);
    vector<int> best_path;
    double min_distance = numeric_limits<double>::infinity();

    do {
        vector<int> path(points_indices);
        path.push_back(path[0]);
        vector<vector<double>> route;
        for (int i : path) {
            route.push_back(points[i]);
        }

        if (checkPath(route, obstacles)) {
            double current_distance = calculateTotalDistance(path, points);
            if (current_distance < min_distance) {
                min_distance = current_distance;
                best_path = path;
            }
        }
    } while (next_permutation(points_indices.begin(), points_indices.end()));

    if (min_distance == numeric_limits<double>::infinity()) {
        min_distance = -1;
    }

    return make_pair(best_path, min_distance);
}

// Algorithm B - based on greedy approach
pair<vector<int>, double> greedy(const vector<vector<double>>& points, const vector<pair<vector<double>, double>>& obstacles) {
    int number_of_points = points.size();
    for (int x = 0; x < number_of_points; ++x) {
        vector<bool> visited(number_of_points, false);
        vector<int> path;
        int current_point = x;
        path.push_back(current_point);
        visited[current_point] = true;

        while (path.size() < number_of_points) {
            int nearest_point = -1;
            double min_distance = numeric_limits<double>::infinity();

            for (int next_point = 0; next_point < number_of_points; ++next_point) {
                if (!visited[next_point] && none_of(obstacles.begin(), obstacles.end(), [&](const pair<vector<double>, double>& obstacle) {
                    return lineSegmentSphereIntersection(points[current_point], points[next_point], obstacle.first, obstacle.second);
                })) {
                    double current_distance = calculateDistance(points[current_point], points[next_point]);
                    if (current_distance < min_distance) {
                        min_distance = current_distance;
                        nearest_point = next_point;
                    }
                }
            }

            if (nearest_point != -1) {
                path.push_back(nearest_point);
                visited[nearest_point] = true;
                current_point = nearest_point;
            } else {
                break;
            }
        }

        if (path.size() == number_of_points && none_of(obstacles.begin(), obstacles.end(), [&](const pair<vector<double>, double>& obstacle) {
            return lineSegmentSphereIntersection(points[path[0]], points[path.back()], obstacle.first, obstacle.second);
        })) {
            path.push_back(path[0]);
            return {path, calculateTotalDistance(path, points)};
        }
    }
    return {{},-1};
}

// Algorithm C - based on randomized greedy approach
pair<vector<int>, double> randomizedGreedy(const vector<vector<double>>& points, const vector<pair<vector<double>, double>>& obstacles) {
    int number_of_points = points.size();
    vector<int> order(number_of_points);
    iota(order.begin(), order.end(), 0);
    random_shuffle(order.begin(), order.end());

    for (const auto& number : order) {
        vector<bool> visited(number_of_points, false);
        vector<int> path;
        int current_point = number;
        path.push_back(current_point);
        visited[current_point] = true;

        while (path.size() < number_of_points) {
            int nearest_point = -1;
            double min_distance = numeric_limits<double>::infinity();

            for (const auto& next_point : order) {
                if (!visited[next_point] && none_of(obstacles.begin(), obstacles.end(), [&](const pair<vector<double>, double>& obstacle) {
                    return lineSegmentSphereIntersection(points[current_point], points[next_point], obstacle.first, obstacle.second);
                })) {
                    double current_distance = calculateDistance(points[current_point], points[next_point]);
                    if (current_distance < min_distance) {
                        min_distance = current_distance;
                        nearest_point = next_point;
                    }
                }
            }

            if (nearest_point != -1) {
                path.push_back(nearest_point);
                visited[nearest_point] = true;
                current_point = nearest_point;
            } else {
                break;
            }
        }

        if (path.size() == number_of_points && none_of(obstacles.begin(), obstacles.end(), [&](const pair<vector<double>, double>& obstacle) {
            return lineSegmentSphereIntersection(points[path[0]], points[path.back()], obstacle.first, obstacle.second);
        })) {
            path.push_back(path[0]);
            return {path, calculateTotalDistance(path, points)};
        }
    }
    return {{},-1};
}

// Function for generating a distance matrix
vector<vector<double>> generateDistanceMatrix(const vector<vector<double>>& points, const vector<pair<vector<double>, double>>& obstacles) {
    size_t number_of_points = points.size();
    vector<vector<double>> distance_matrix(number_of_points, vector<double>(number_of_points, 0.0));

    for (size_t i = 0; i < number_of_points; ++i) {
        for (size_t j = 0; j < number_of_points; ++j) {
            bool obstacle_collision = false;
            for (const auto& obstacle : obstacles) {
                if (lineSegmentSphereIntersection(points[i], points[j], obstacle.first, obstacle.second)) {
                    distance_matrix[i][j] = numeric_limits<double>::infinity();
                    obstacle_collision = true;
                    break;
                }
            }

            if (!obstacle_collision) {
                distance_matrix[i][j] = calculateDistance(points[i], points[j]);
            }
        }
    }
    return distance_matrix;
}

// Function for calculating the total distance based on the distance matrix
double calculateTotalDistanceFromMatrix(const vector<int>& order, const vector<vector<double>>& distance_matrix) {
    double total_distance = 0;
    for (size_t i = 0; i < order.size() - 1; ++i) {
        total_distance += distance_matrix[order[i]][order[i + 1]];
    }
    total_distance += distance_matrix[order.back()][order[0]];
    return total_distance;
}

// Algorithm D - based on the Tabu Search algorithm
pair<vector<int>, double> TabuSearch(const vector<vector<double>>& points, const vector<pair<vector<double>, double>>& obstacles, int max_iterations, int tabu_size) {
    vector<vector<double>> distance_matrix = generateDistanceMatrix(points, obstacles);
    int number_of_points = distance_matrix.size();

    vector<int> current_solution(number_of_points);
    iota(current_solution.begin(), current_solution.end(), 0);
    random_shuffle(current_solution.begin(), current_solution.end());

    vector<int> best_solution = current_solution;

    vector<vector<int>> tabu_list;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        vector<vector<int>> neighborhood;

        for (int i = 0; i < number_of_points - 1; ++i) {
            for (int j = i + 1; j < number_of_points; ++j) {
                vector<int> neighbor = current_solution;
                swap(neighbor[i], neighbor[j]);
                neighborhood.push_back(neighbor);
            }
        }

        vector<int> best_neighbor;
        double best_neighbor_distance = numeric_limits<double>::infinity();

        for (const auto& neighbor : neighborhood) {
            if (find(tabu_list.begin(), tabu_list.end(), neighbor) == tabu_list.end()) {
                double neighbor_distance = calculateTotalDistanceFromMatrix(neighbor, distance_matrix);

                if (neighbor_distance <= best_neighbor_distance) {
                    best_neighbor_distance = neighbor_distance;
                    best_neighbor = neighbor;
                }
            }
        }

        if (!best_neighbor.empty()) {
            current_solution = best_neighbor;

            tabu_list.push_back(current_solution);
            if (tabu_list.size() > tabu_size) {
                tabu_list.erase(tabu_list.begin());
            }

            double best_distance = calculateTotalDistanceFromMatrix(best_solution, distance_matrix);

            double current_distance = calculateTotalDistanceFromMatrix(current_solution, distance_matrix);
            if (current_distance < best_distance) {
                best_solution = current_solution;
            }
        }
    }

    double best_distance = calculateTotalDistanceFromMatrix(best_solution, distance_matrix);
    best_solution.push_back(best_solution[0]);

    if (isinf(best_distance)) {
        best_solution.clear();
        best_distance = -1;
    }

    return make_pair(best_solution, best_distance);
}


int main() {
    vector<vector<double>> loaded_points;
    vector<pair<vector<double>, double>> loaded_obstacles;
    string inFilePath = "../../instance.json";
    string outFilePath = "../../results.json";
    int max_iterations = 10000;
    int tabu_size = 100;
    vector<pair<vector<int>, double>> results;
    vector<int> times;

    readFromJson(loaded_points, loaded_obstacles, inFilePath);

    auto startTime = high_resolution_clock::now();
    auto resultBruteforce = bruteforce(loaded_points, loaded_obstacles);
    auto stopTime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stopTime - startTime);
    times.push_back(duration.count());
    results.push_back(resultBruteforce);


    startTime = high_resolution_clock::now();
    auto resultGreedy = greedy(loaded_points, loaded_obstacles);
    stopTime = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stopTime - startTime);
    times.push_back(duration.count());
    results.push_back(resultGreedy);

    startTime = high_resolution_clock::now();
    auto resultRandomizedGreedy = randomizedGreedy(loaded_points, loaded_obstacles);
    stopTime = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stopTime - startTime);
    times.push_back(duration.count());
    results.push_back(resultRandomizedGreedy);

    startTime = high_resolution_clock::now();
    auto resultTabuSearch = TabuSearch(loaded_points, loaded_obstacles, max_iterations, tabu_size);
    stopTime = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stopTime - startTime);
    times.push_back(duration.count());
    results.push_back(resultTabuSearch);

    saveToJson(times, results, outFilePath);

    cout << "Algorithm A:\n";
    cout << "Solution: ";
    for (int i : resultBruteforce.first) {
        cout << i << " ";
    }
    cout << "\nMinimal distance: " << resultBruteforce.second << endl << endl;

    cout << "Algorithm B:\n";
    cout << "Solution: ";
    for (int i : resultGreedy.first) {
        cout << i << " ";
    }
    cout << "\nMinimal distance: " << resultGreedy.second << endl << endl;

    cout << "Algorithm C:\n";
    cout << "Solution: ";
    for (int i : resultRandomizedGreedy.first) {
        cout << i << " ";
    }
    cout << "\nMinimal distance: " << resultRandomizedGreedy.second << endl << endl;

    cout << "Algorithm D:\n";
    cout << "Solution: ";
    for (int i : resultTabuSearch.first) {
        cout << i << " ";
    }
    cout << "\nMinimal distance: " << resultTabuSearch.second << endl;

    return 0;
}