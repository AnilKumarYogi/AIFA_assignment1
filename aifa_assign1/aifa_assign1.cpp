#include <vector>
#include <tuple>
#include <limits>
#include <time.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>

using namespace std;

#define MAX __FLT_MAX__
int vehicle_id = 0;

class EV
{
private:
    int _S;             // source node
    int _D;             // destination
    int _id;            // vehicle id
    float _B;           // battery charge status
    float _c;           // charging rate of the battery at a charging station (energy per unit time)
    float _d;           // discharging rate of the battery while travelling (distance travelled per unit charge)
    float _M;           // maximum battery capacity
    float _s;           // average traveling speed (distance per unit time)
    float _T;           // time taken to reach destination D
    clock_t _init_time; // time of start of algo

public:
    // default parameter constructor
    EV(int S = 0, int D = 7, int id = vehicle_id, float B = 10, float c = 1, float d = 0.5, float M = 50, float s = 0.7)
        : _S(S), _D(D), _B(B), _id(id), _c(c), _d(d), _M(M), _s(s) { vehicle_id++; }

    // returns src node
    int retSrc()
    {
        return _S;
    }

    // returns dest node
    int retDest()
    {
        return _D;
    }

    // returns vehicle id
    int retId()
    {
        return _id;
    }

    // returns time taken to traverse an edge
    float timeTraversal(int edge)
    {
        return edge / _d;
    }

    // returns the time spent at parent node u for charging to go to the child node v
    float timeCharging(int edge)
    {
        float b_req = edge / _d;
        if (b_req > _M)
            return MAX;
        else if (b_req > _B)
            return (b_req - _B) / _c;
        // sufficient to move ahead
        return 0;
    }

    // update battery
    void updateBattery(int edge)
    {
        float timeToCharge = timeCharging(edge);
        // not charged but moved ahead
        if (timeToCharge == 0)
            _B -= edge / _d;
        // charged sufficiently to reach the node
        else if (timeToCharge != MAX && timeToCharge > 0)
            _B = 0;
    }

    // initilaize the time
    void initTime()
    {
        _init_time = clock();
    }

    // return the relative time elapsed
    clock_t timeElapsed(clock_t currTime)
    {
        return (currTime - _init_time);
    }
};

// helper function to identify overlapping time intervals
clock_t timeCollision(tuple<float, float> t1, tuple<float, float> t2)
{
    // does not overlap
    if (get<1>(t2) <= get<0>(t1) || get<1>(t1) <= get<0>(t2))
        return 0;
    // conflict
    return (max(get<0>(t1), get<0>(t2)) - min(get<1>(t1), get<1>(t2)));
}

// helper function to find the node with min cost function in the open list
int minDistance(vector<float> dist, vector<bool> closed, int N)
{
    float min = MAX, min_index;

    for (int v = 0; v < N; v++)
        if (!closed[v] && dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}

// Backward Dijkstra, our destination acts as its source node and its dist stores our heuristic
void trueHeuristic(vector<vector<float>> graph, int N, int src, vector<float> &dist)
{
    vector<bool> closed(N, false);

    // distance of source node from itself
    dist[src] = 0;

    // find shortest path for all nodes
    for (int count = 0; count < N - 1; count++)
    {
        // pick the minimum distance node from the open list
        int u = minDistance(dist, closed, N);

        // mark the picked node in closed list
        closed[u] = true;

        // update dist value of its children nodes
        for (int v = 0; v < N; v++)
            if (!closed[v] && graph[u][v] && dist[u] != MAX && dist[u] + graph[u][v] < dist[v])
                dist[v] = dist[u] + graph[u][v];
    }
}

// -------------------------------sta* for a single agent-------------------------------
clock_t STAstar(EV ev, vector<vector<int>> dist_graph, int N)
{
    // global Lookup Matrix (2d vector of vehicle_id x node_id, each cell carrying a tuple (t_arrival, t_departure))
    static vector<vector<tuple<float, float>>> globalMat(N, vector<tuple<float, float>>(N));

    ev.initTime();
    int src = ev.retSrc();
    int dest = ev.retDest();
    int temp = src;
    // dist[i] = time(reaching i) + time(go to goal)(i.e., h_val) + time(charging)
    vector<float> dist(N, MAX), h_val(N, MAX);
    vector<int> parent(N, -1);
    vector<bool> closed(N, false);
    vector<vector<tuple<clock_t, clock_t>>> wasCharged(N, vector<tuple<clock_t, clock_t>>(N));
    // time-graph
    vector<vector<float>> graph(N, vector<float>(N, 0));

    // change the edge weights in terms of time (= power used to traverse them)
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            graph[i][j] = ev.timeTraversal(dist_graph[i][j]);
        }
    }

    // populates the true heuristics of the given graph
    trueHeuristic(graph, N, dest, h_val);

    // distance from source node to itself
    dist[src] = h_val[src];

    // find shortest path for all nodes
    while (true)
    {
        // pick the minimum distance node from the open list (src is always the first one)
        int u = minDistance(dist, closed, N);

        // stop the algo when goal is reached
        if (u == dest)
            break;

        // mark the picked node in closed list
        closed[u] = true;

        // make an entry in globalMat if charging was done before coming here
        if (get<0>(wasCharged[temp][u]))
        {
            // cout << "Refuelled at node " << temp << endl;
            globalMat[ev.retId()][temp] = make_tuple(get<0>(wasCharged[temp][u]), get<1>(wasCharged[temp][u]));
            ev.updateBattery(dist_graph[temp][u]);
        }

        // expanding child nodes
        for (int v = 0; v < N; v++)
            if (!closed[v] && graph[u][v] && dist[u] != MAX && dist[u] - h_val[u] + graph[u][v] < dist[v])
            {
                // update dist value
                float timeToCharge = ev.timeCharging(dist_graph[u][v]);
                dist[v] = dist[u] - h_val[u] + graph[u][v] + h_val[v] + timeToCharge;
                // update parent node
                parent[v] = u;

                if (timeToCharge != MAX && timeToCharge > 0)
                {
                    clock_t t_arr = ev.timeElapsed(clock());
                    clock_t t_dep = t_arr + timeToCharge;
                    wasCharged[u][v] = make_tuple(t_arr, t_dep);

                    // add waiting time in case of conflict
                    clock_t waitTime = timeCollision(wasCharged[u][v], globalMat[u][v]);
                    dist[v] += waitTime;
                }
            }
        temp = u;
    }

    clock_t timeTaken = ev.timeElapsed(clock());

    // print the path
    cout << "END"
         << "<-" << ev.retDest() << "<-";
    for (int i = dest; i != src;)
    {
        cout << parent[i] << "<-";
        i = parent[i];
    }
    cout << "EV " << ev.retId() << endl;
    cout << "Time elapsed = " << timeTaken << " clicks." << endl;
    return timeTaken;
}

int main()
{
    string line;
    ifstream infile("input.txt");
    int row = 0, col = 0;
    vector<int> vertices;

    while (getline(infile, line))
    {
        istringstream iss(line);
        int c;

        while (iss >> c)
        {
            col++;
            vertices.push_back(c);
        }
        row++;
    }
    if (row * row != col)
    {
        cout << "The input matrix must be a square matrix. Exiting the program.\n";
        exit(0);
    }
    int N = row;
    int count = 0;
    vector<vector<int>> graph(N, vector<int>(N));
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            graph[i][j] = vertices[count++];
        }
    }

    // // print the graph taken from the text file
    // for (int i = 0; i < N; i++)
    // {
    //     for (int j = 0; j < N; j++)
    //     {
    //         cout << graph[i][j] << ", ";
    //     }
    //     cout << endl;
    // }

    // initialize EV objects
    EV ev1 = EV(8, 4);
    EV ev2 = EV(0, 3);
    EV ev3 = EV(7, 5);
    EV ev4 = EV(2, 4);

    // execution starts
    vector<clock_t> timeTaken;
    clock_t start = clock();
    timeTaken.push_back(STAstar(ev1, graph, N));
    timeTaken.push_back(STAstar(ev2, graph, N));
    timeTaken.push_back(STAstar(ev3, graph, N));
    timeTaken.push_back(STAstar(ev4, graph, N));
    // execution ends
    clock_t stop = clock();

    cout << "Execution time for the code: " << ((float)(stop - start)) / CLOCKS_PER_SEC << " seconds" << endl;
    cout << "Max time taken to travel from source to destination: " << ((float)*max_element(timeTaken.begin(), timeTaken.end())) / CLOCKS_PER_SEC << " seconds" << endl;

    return 0;
}
