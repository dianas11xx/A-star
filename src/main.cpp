#include <iostream>
#include <vector>
#include <math.h>
#include <queue>
#include <algorithm>
#include <functional>

using namespace std;

struct Node{
    float x, y;
};

struct Edge{
    // index of node edge is pointing to
    int index;
    // weight of the edge
    float weight;
};
//Helper functions

// Calculate and return the distance between two points
float distanceFormula(Node point1, Node point2);

// Detects whether two line segments intersect 
bool linesIntersect(vector<Node> line1, vector<Node> line2);

// Implement the point in polygon algorithm to determine if the given midpoint is within the given polygon. Returns true if it is
bool midpointCheck(float mid_x, float mid_y, vector<Node>& polygon);

// Detects whether a line intersects any polygon. returns true if yes
bool polygonIntersect(vector<Node>& line, vector<vector<Node>>& polygons);

//Use the cameFrom array to reconstruct the optimal path to the start index (0). Print the resulting path of indices
vector<int> reconstruct_path(vector<int> cameFrom, int currentIdx, vector<Node>& nodes, vector<float>& gn);

int main() {
    // Initialize start and goal nodes
    Node startNode = {120,650};
    Node goalNode = {380,560};

    // Initialize the polygons
    vector<vector<Node>> polygons = {
        {{220, 616}, {220, 666}, {251, 670}, {272, 647}},
        {{341, 655}, {359, 667}, {374, 651}, {366, 577}},
        {{311, 530}, {311, 559}, {339, 578}, {361, 560}, {361, 528}, {336, 516}},
        {{105, 628}, {151, 670}, {180, 629}, {156, 577}, {113, 587}},
        {{118, 517}, {245, 517}, {245, 577}, {118, 557}},
        {{280, 583}, {333, 583}, {333, 665}, {280, 665}},
        {{252, 594}, {290, 562}, {264, 538}},
        {{198, 635}, {217, 574}, {182, 574}}};

    // Add all nodes to a vector
    vector<Node> nodes;
    nodes.push_back(startNode);
    for(vector<Node>& polygon : polygons){
        nodes.insert(nodes.end(), polygon.begin(), polygon.end());
    }
    nodes.push_back(goalNode);
    
    // Build the graph
    vector<vector<Edge>> graph(nodes.size());

    // add all edges of polygon to map
    vector<pair<int, int>> polygonEdges;
    int currNodeIDX = 1;
    // iterate through each polygon to add all edges to the list
    for(vector<Node>& polygon: polygons){
        int polyIDX1 = currNodeIDX;
        for(int i = 0; i < polygon.size(); ++i){
            int vert1_IDX = currNodeIDX;
            int vert2_IDX = polyIDX1 + ((i+1) % polygon.size());
            // Store the edge pair (sorted)
            polygonEdges.push_back({min(vert1_IDX, vert2_IDX), max(vert1_IDX, vert2_IDX)});
            currNodeIDX++;
        }
    }
    // sort and remove duplicate edges
    sort(polygonEdges.begin(), polygonEdges.end());
    polygonEdges.erase(unique(polygonEdges.begin(), polygonEdges.end()), polygonEdges.end());
    // Add the vertices of edges into the graph 
    for(pair<int,int>& edge : polygonEdges){
        int index1 = edge.first;
        int index2 = edge.second;
        // Define the weight of the edge using the distance formula
        float weight = distanceFormula(nodes[index1], nodes[index2]);
        // Add both indices to the graph with their weight
        graph[index1].push_back({index2, weight});
        graph[index2].push_back({index1, weight});
        
    }

    // Add all straight line edges from each vertex to any other in the scene that is 
    // not already in polygonEdges

    for(int i = 0; i < nodes.size(); i++){
        for(int j = i+1; j < nodes.size(); j++){
            // make sure polygon edge is not alredy in the graph
            if(binary_search(polygonEdges.begin(), polygonEdges.end(), make_pair(i,j))){
                continue;
            }
            vector<Node> line1 = {nodes[i], nodes[j]};
            // if the line does not intersect any polygons, add the edge to the graph
            if(!polygonIntersect(line1, polygons)){
                float weight = distanceFormula(nodes[i], nodes[j]);
                graph[i].push_back({j, weight});
                graph[j].push_back({i, weight});
            }
        }
    }

    
    
    /*------------------Implement A* algorithm -------------------*/

    // initialize A* algorithm funntions
    // est cost : current best guess to cheapest path
    vector<float> fn(nodes.size(), 1e9);
    // true cost : Currently known cost of the cheapest path from start to n 
    vector<float> gn(nodes.size(), 1e9);
    // heuristic function (stright line distance to the end node)
    vector<float> hn(nodes.size());
    // initialize goal node index
    int goal_idx = nodes.size()-1;

    // define heuristic function for all nodes (stright line distance from node to goal node)
    for(int i = 0; i < nodes.size(); i++){
        hn[i] = distanceFormula(nodes[i], nodes[goal_idx]);
    }

    /* Create a min-priority queue called OPEN to track nodes for exploration
       stores pair of float representin the f(n), and int which is the node */
    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> OPEN;
    // Add the start node to the heuristic function
    gn[0] = 0;
    fn[0] = gn[0] + hn[0];

    OPEN.push({fn[0],0});
    //initialize CLOSED set that contains nodes that have already been examined
    vector<int> cameFrom(nodes.size());
    vector<bool> CLOSED;
    CLOSED = vector(nodes.size(), false);

    while(!OPEN.empty()){
        // Get the lowest rank item from OPEN
        pair<float, int> current = OPEN.top();
        int currIdx = current.second;
        // if current index is the goal index, then were done
        if(currIdx == goal_idx){
            // return reconstruction fram with came from and current
            reconstruct_path(cameFrom, currIdx, nodes, gn);
            break;
        }
        // remove the lowest ranked item from OPEN
        OPEN.pop();
        //add current index to closed  
        CLOSED[currIdx] = true;
        for(Edge neighbor : graph[currIdx]){
            //distance from start to neighbor 
            float tentative_gScore = gn[currIdx] + neighbor.weight;
            // Check if tentative_gScore path is better than the previous one
            if(tentative_gScore < gn[neighbor.index]){
                // Update variables at neighbor index with better path
                cameFrom[neighbor.index] = currIdx;
                gn[neighbor.index] = tentative_gScore;
                fn[neighbor.index] = tentative_gScore + hn[neighbor.index];
                // if the node at neighbor index has not been visited, add it to OPEN
                if(CLOSED[neighbor.index] == false){
                    OPEN.push({fn[neighbor.index], neighbor.index});
                }
            }
        }

    }
    return 0;
}

// Calculate and return the distance between two points
float distanceFormula(Node point1, Node point2){
    // sqrt (x2-x1)^2 +(y2-y1)^2
    return sqrt(pow(point2.x-point1.x, 2) + pow(point2.y-point1.y, 2));
}
// Detects whether two line segments intersect 
bool linesIntersect(vector<Node> line1, vector<Node> line2){
    Node p1 = line1[0];
    Node p2 = line1[1];
    Node q1 = line2[0];
    Node q2 = line2[1];

    float k1 = p1.x - p2.x;
    float k2 = q2.y - q1.y;
    float k3 = p1.y - p2.y;
    float k4 = q2.x - q1.x;
    float k5 = p1.x - q1.x;
    float k6 = p1.y - q1.y;

    float d = (k1*k2) - (k3*k4);
    // when d is 0, lines are parrellel and do not intersect
    if(d==0){
        return false;
    }
    float a = ((k2*k5) - (k4*k6)) / d;
    float b = ((k1*k6) - (k3*k5)) / d;


    if(a > 0 && a < 1 && b > 0 && b < 1){
        // return true if p and q intersect (both a and b are between 0 & 1)
        return true;
    }

    return false;
    
}
// Implement the point in polygon algorithm to determine if the given midpoint is within the given polygon. 
// Returns true if it is
bool midpointCheck(float mid_x, float mid_y, vector<Node>& polygon){

    bool insidePoly = false;
    // loop through each edge in the polygon
    for(int i =0, j=polygon.size()-1; i<polygon.size(); j=i++){
        Node polyVert1 = polygon[i];
        Node polyVert2 = polygon[j];

        // check if midpoint of line is within the polygon
        if ((polyVert1.y > mid_y) != (polyVert2.y > mid_y)){
            float x_intercept = (polyVert2.x - polyVert1.x)*(mid_y - polyVert1.y) / (float)(polyVert2.y - polyVert1.y) + polyVert1.x;
            if(mid_x < x_intercept){
                insidePoly = !insidePoly;
            }
        }
    }
    return insidePoly;
}

// determine whether a line intersects any polygon in the grid
bool polygonIntersect(vector<Node>& line, vector<vector<Node>>& polygons){
    // iterate through every polygon in the list
    for(vector<Node>& polygon : polygons){
        // loop through each edge in the polygon
        Node polyVert1 = polygon[0];
        Node polyVert2;
        float midpoint_x = (line[0].x + line[1].x) / 2.0f;
        float midpoint_y = (line[0].y + line[1].y) / 2.0f; 
        for(int i = 1; i <= polygon.size(); i++){
            if(i==polygon.size()){
                polyVert2 = polygon[0];
            }else{
                polyVert2 = polygon[i];
            }
            vector<Node> line1 = {polyVert1, polyVert2};
            // check if the lines intersect, and return true if true
            if(linesIntersect(line, line1)){
                return true;
            }
            polyVert1 = polyVert2;
        }
        if(midpointCheck(midpoint_x,midpoint_y, polygon)){
            return true;
        }
    }
    // return false if line did not intersect the given polygon
    return false;
}
// Use the cameFrom array to reconstruct the optimal path to the start index (0). 
// Print the resulting path of indices
vector<int> reconstruct_path(vector<int> cameFrom, int currentIdx, vector<Node>& nodes, vector<float>& gn){
    vector<int> totalPath = {currentIdx};
    float totalCost = gn[currentIdx];
    // Iterate through cameFrom array back to the start index 0
    while(currentIdx != 0){
        currentIdx = cameFrom[currentIdx];
        totalPath.push_back(currentIdx);
    }
    // reverse path
    reverse(totalPath.begin(), totalPath.end());
    // print optimal path
    cout<< "smallest path: "<<endl;
    for(int index: totalPath){
        cout<<"("<<nodes[index].x<<", "<<nodes[index].y<<"): Current Cost: "<<gn[index]<<endl;
    }
    cout<<endl;
    cout<<"Final Cost: "<< totalCost<<endl;
    return totalPath;
};