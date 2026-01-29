#pragma once
#include <vector>
#include <string>
using namespace std;

class RoboPath{
    public:
        int shortest_dist;
        
        int true_cost;
        // heuristic function (stright line distannce to the end node)
        int est_cost;

        // Helper functions

        // Calculate and return the distance between two points
        int distanceFormula(vector<int> point1, vector<int> point2);

        // Takes a vertex as input and returns the set of verticies that can be reached in a stright line from the given vertex
        vector<vector<int>> strightLineList(vector<int> startVertex);

        // Detects whether two line segments intersect 
        bool isValidLine(vector<vector<int>> line1, vector<vector<int>> line2);


};