#pragma once
#include <vector>
#include <map>
using namespace std;

class Q1_2{
    public:
        map<char, int> mapper= {{'a',0}, {'b',1}, {'c',2},{'d',3},{'e',4}, {'f',5}};
        vector<vector<int>> graph;
        vector<bool> seen;
        vector<int> ans;

        vector<int> dfs(vector<vector<char>>& graph);
        void dfs_helper(vector<vector<char>>& graph, char node);

};

