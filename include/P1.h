#pragma once

#include <vector>
#include <string>
using namespace std;

class P1{
    public:
        vector<vector<int>> graph;
        vector<bool> seen;
        vector<int> ans;
        int n = 0;

        vector<int> dfs(vector<vector<int>>& adj);
        void dfs_helper(vector<vector<int>>& adj, int node);
        
};

// check if the line is along the polygon edge, if its 
/*
            if((line[0].x == polyVert1.x && line[0].y == polyVert1.y && line[1].x == polyVert2.x && line[1].y == polyVert2.y) ||
                line[0].x == polyVert2.x && line[0].y == polyVert2.y && line[1].x == polyVert1.x && line[1].y == polyVert1.y){
                    continue;
            }

            //check if the verticies are shard
            if( line[0].x == polyVert1.x && line[0].y == polyVert1.y || 
                line[1].x == polyVert1.x && line[1].y == polyVert1.y ||
                line[0].x == polyVert2.x && line[0].y == polyVert2.y || 
                line[1].x == polyVert2.x && line[1].y == polyVert2.y ){
                    continue;
            }*/