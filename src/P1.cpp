#include "P1.h"
#include <vector>
using namespace std;

vector<int> P1::dfs(vector<vector<int>>& adj){
    n = adj.size();
    seen = vector(n, false);
    dfs_helper(adj, 0);
    return ans;
}
void P1::dfs_helper(vector<vector<int>>& adj, int node){
    seen[node] = true;
    ans.push_back(node);
    for(int neighbor: adj[node]){
        if(!seen[neighbor]){
            dfs_helper(adj, neighbor);
        }
    }
}


/*
#include <iostream>
#include <vector>
#include <string>
#include "P1.h"

using namespace std;

int main() {
    P1 obj;
    vector<vector<int>> adj = {{2,3,1}, {0}, {0,4}, {0}, {2}};
    vector<int> result;
    result = obj.dfs(adj);
    for(int i = 0 ; i < result.size(); i++){
        cout << result[i];
    }
    cout<<endl;
}

*/