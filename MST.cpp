#include "MST.hpp"
#include <queue>
#include <algorithm>
#include <limits>

using namespace std;

// Constructor
MST::MST(Graph graph, string algo) : graph(graph)
{
    vector<vector<int>> adjmat = graph.getAdjMat();
    int n = adjmat.size();
    if (n == 0)
    {
        mst = vector<vector<int>>();
    }
    mst = vector<vector<int>>(n, vector<int>(n, 0));

    if (algo =="kruskal")
    {
        kruskal(adjmat);
    }
    else if (algo =="boruvka")
    {
        boruvka(adjmat);
    }
    else
    {
        cout << "Invalid algorithm" << endl;
    }
}
// Function to get the weight of the MST
int MST::getWieght()
{
    if (mst.empty())
        return 0;
    int weight = 0;
    for (size_t i = 0; i < mst.size(); i++)
    {
        for (size_t j = i + 1; j < mst[i].size(); j++)
        {
            weight += mst[i][j];
        }
    }
    return weight;
}

/**
 * getMST
 * Returns the adjacency matrix of the MST.
 *
 * @return The MST adjacency matrix.
 */
vector<vector<int>> MST::getMST()
{
    return mst;
}

// Function to calculate the average distance of all edges in the MST
void MST::kruskal(vector<vector<int>> &adj)
{
    if (adj.empty())
        return;
    int n = adj.size();
    vector<tuple<int, int, int>> edges;

    // Collect all edges with weights into a list of tuples
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            if (adj[i][j] > 0)
            {
                edges.emplace_back(adj[i][j], i, j);
            }
        }
    }

    sort(edges.begin(), edges.end());

    vector<int> parent(n);
    for (int i = 0; i < n; i++)
        parent[i] = i;

    function<int(int)> find = [&](int x)
    {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    };

    auto unite = [&](int x, int y)
    {
        x = find(x);
        y = find(y);
        if (x != y)
            parent[y] = x;
    };

    // Add edges to MST, avoiding cycles
    for (const auto &[w, u, v] : edges)
    {
        if (find(u) != find(v))
        {
            unite(u, v);
            mst[u][v] = mst[v][u] = w;
        }
    }
}

/**
 * boruvka
 * Constructs the MST using Borůvka's algorithm, which finds the cheapest edge
 * for each component and merges components until only one remains.
 *
 * @param adj The adjacency matrix of the input graph.
 */

void MST::boruvka(vector<vector<int>> &adj)
{
    // Check if the adjacency matrix is empty (no vertices)
    if (adj.empty())
        return;

    int n = adj.size(); // Number of vertices in the graph
    vector<int> parent(n); // Array to keep track of the parent of each node for union-find
    vector<int> rank(n, 0); // Rank array for union by rank in union-find

    // Initialize each vertex to be its own parent (self-loop) initially
    for (int i = 0; i < n; i++)
        parent[i] = i;

    // Function to find the root of a node with path compression
    function<int(int)> find = [&](int x) {
        if (parent[x] != x)
            parent[x] = find(parent[x]); // Path compression
        return parent[x];
    };

    // Function to union two sets by rank
    auto unite = [&](int x, int y) {
        x = find(x);
        y = find(y);
        if (x == y) // If both vertices are in the same set, return false
            return false;
        if (rank[x] < rank[y]) // Union by rank
            swap(x, y);
        parent[y] = x; // Make x the parent of y
        if (rank[x] == rank[y])
            rank[x]++; // Increment rank if both have the same rank
        return true;
    };

    bool change = true; // Flag to track if we added any edges in the current iteration

    // Continue until no more edges can be added to the MST
    while (change) {
        change = false;
        // Array to store the cheapest edge for each component
        vector<pair<int, int>> cheapest(n, {-1, -1});

        // Loop through each vertex and find the cheapest outgoing edge for each component
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                // If there's an edge between i and j
                if (adj[i][j] > 0) {
                    int set1 = find(i); // Find component of vertex i
                    int set2 = find(j); // Find component of vertex j
                    // If they are in different components, find the cheaper edge
                    if (set1 != set2) {
                        // Update cheapest edge for set1 if needed
                        if (cheapest[set1].second == -1 || adj[i][j] < adj[cheapest[set1].first][cheapest[set1].second]) {
                            cheapest[set1] = {i, j};
                        }
                        // Update cheapest edge for set2 if needed
                        if (cheapest[set2].second == -1 || adj[i][j] < adj[cheapest[set2].first][cheapest[set2].second]) {
                            cheapest[set2] = {i, j};
                        }
                    }
                }
            }
        }

        // Add the cheapest edges found to the MST
        for (int i = 0; i < n; i++) {
            if (cheapest[i].second != -1) { // If there's a valid edge
                int u = cheapest[i].first;
                int v = cheapest[i].second;
                int set1 = find(u);
                int set2 = find(v);
                // If u and v are in different components, add edge to MST
                if (set1 != set2) {
                    mst[u][v] = mst[v][u] = adj[u][v]; // Add edge (u, v) to MST
                    unite(set1, set2); // Union the two components
                    change = true; // Mark that we made a change
                }
            }
        }
    }
}


/**
 * shortestPath
 * Finds the shortest path between two nodes in the MST using BFS.
 *
 * @param start The start node.
 * @param end The end node.
 * @return A vector representing the path from start to end, or an empty vector if no path exists.
 */
vector<int> MST::shortestPath(int start, int end)
{
    if (mst.empty())
        return {};
    
    if (start == end)
        return {start};
    
    int n = mst.size();
    vector<bool> visited(n, false);
    vector<int> parent(n, -1);
    queue<int> q;

    q.push(start);
    visited[start] = true;

    while (!q.empty())
    {
        int u = q.front();
        q.pop();

        if (u == end)
            break;

        for (int v = 0; v < n; v++)
        {
            if (mst[u][v] > 0 && !visited[v])
            {
                visited[v] = true;
                parent[v] = u;
                q.push(v);
            }
        }
    }

    if (!visited[end])
        return {};

    vector<int> path;
    for (int v = end; v != -1; v = parent[v])
    {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

/**
 * longestPath
 * Finds the longest path between two nodes in the MST using DFS.
 *
 * @param start The start node.
 * @param end The end node.
 * @return A vector representing the longest path from start to end, or an empty vector if no path exists.
 */
vector<int> MST::longestPath(int start, int end)
{
    if (mst.empty())
        return {};
    int n = mst.size();
    vector<int> dist(n, -1);
    vector<int> parent(n, -1);

    function<void(int, int)> dfs = [&](int u, int p)
    {
        for (int v = 0; v < n; v++)
        {
            if (mst[u][v] > 0 && v != p)
            {
                dist[v] = dist[u] + 1;
                parent[v] = u;
                dfs(v, u);
            }
        }
    };

    dist[start] = 0;
    dfs(start, -1);

    if (dist[end] == -1)
        return {};

    vector<int> path;
    for (int v = end; v != -1; v = parent[v])
    {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

/**
 * averageDist
 * Calculates the average distance of all edges in the MST.
 *
 * @return The average edge distance or -1 if no paths exist in the MST.
 */
int MST::averageDist() {
    if (mst.empty())
        return -1;

    int totalDistance = 0;
    int pathCount = 0;
    size_t n = mst.size();

    for (size_t start = 0; start < n; ++start) {
        for (size_t end = start + 1; end < n; ++end) {
            if (mst[start][end] > 0) {
                totalDistance += mst[start][end];
                pathCount++;
            }
        }
    }

    if (pathCount == 0)
        return -1;

    return (totalDistance + pathCount) / pathCount;
}
