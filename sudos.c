
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <time.h>

#define MAX_LOC    15
#define MAX_AGENTS  4
#define INF        (INT_MAX / 2)
#define NUM_LOCATIONS  8
#define NUM_AGENTS     2
#define NUM_DELIVERIES 7

// Location names
char *locationNames[NUM_LOCATIONS] = {
    "Depot", "A-Nagar", "B-Nagar", "C-Nagar",
    "D-Nagar", "E-Nagar", "F-Nagar", "G-Nagar"
};

// Adjacency matrix 
int rawGraph[NUM_LOCATIONS][NUM_LOCATIONS] = {
    {  0, 10, 20,  0, 30,  0,  0,  0 },   // Depot
    { 10,  0,  5, 15,  0,  0,  0,  0 },   // A-Nagar
    { 20,  5,  0,  0, 10, 25,  0,  0 },   // B-Nagar
    {  0, 15,  0,  0,  0, 10,  5,  0 },   // C-Nagar
    { 30,  0, 10,  0,  0,  0, 20, 15 },   // D-Nagar
    {  0,  0, 25, 10,  0,  0,  8, 12 },   // E-Nagar
    {  0,  0,  0,  5, 20,  8,  0,  6 },   // F-Nagar
    {  0,  0,  0,  0, 15, 12,  6,  0 }    // G-Nagar
};

// Delivery locations 
int deliveries[NUM_DELIVERIES] = { 1, 2, 3, 4, 5, 6, 7 };


int n;                            // actual number of locations used
int dist[MAX_LOC][MAX_LOC];       // working distance matrix
char locName[MAX_LOC][20];        // location name strings
int sp[MAX_LOC][MAX_LOC];         // all-pairs shortest path matrix
int dpMemo[1 << MAX_LOC][MAX_LOC];
int dpPar[1 << MAX_LOC][MAX_LOC];

//function declarations
void  initGraph();
void  dijkstra(int src, int d[]);
void  buildShortestPaths(int sp[MAX_LOC][MAX_LOC]);
int   greedyRoute(int start, int *deliveries, int nd, int *route, int sp[MAX_LOC][MAX_LOC]);
int   tspDP(int *nodes, int nd, int sp[MAX_LOC][MAX_LOC]);
int   nearestInsertion(int *nodes, int nd, int sp[MAX_LOC][MAX_LOC]);
void  assignDeliveries(int *deliveries, int nd, int nAgents, int agentJobs[][MAX_LOC], int agentCount[]);
void  printRoute(int *route, int len);
void  printComparison(int totalGreedy, int totalDP, int totalHeuristic);


// function definitions

void initGraph() {
    n = NUM_LOCATIONS;
    for (int i = 0; i < n; i++) {
        strcpy(locName[i], locationNames[i]);
        for (int j = 0; j < n; j++) {
            if (i == j)              dist[i][j] = 0;
            else if (rawGraph[i][j]) dist[i][j] = rawGraph[i][j];
            else                     dist[i][j] = INF;
        }
    }
}

// Dijkstra -- single source shortest path from src
void dijkstra(int src, int d[]) {
    int visited[MAX_LOC] = {0};
    for (int i = 0; i < n; i++) d[i] = INF;
    d[src] = 0;

    for (int iter = 0; iter < n - 1; iter++) {
        int u = -1, minD = INF;
        for (int i = 0; i < n; i++)
            if (!visited[i] && d[i] < minD) { minD = d[i]; u = i; }
        if (u == -1) break;
        visited[u] = 1;
        for (int v = 0; v < n; v++)
            if (!visited[v] && dist[u][v] != INF && d[u] + dist[u][v] < d[v])
                d[v] = d[u] + dist[u][v];
    }
}

void buildShortestPaths(int sp[MAX_LOC][MAX_LOC]) {
    for (int i = 0; i < n; i++) dijkstra(i, sp[i]);
}

// Greedy Nearest Neighbor routing for one agent
int greedyRoute(int start, int *del, int nd, int *route, int sp[MAX_LOC][MAX_LOC]) {
    int visited[MAX_LOC] = {0};
    int cur = start, totalCost = 0, step = 0;
    route[step++] = start;

    for (int i = 0; i < nd; i++) {
        int best = -1, bestD = INF;
        for (int j = 0; j < nd; j++)
            if (!visited[j] && sp[cur][del[j]] < bestD) { bestD = sp[cur][del[j]]; best = j; }
        visited[best] = 1;
        totalCost += bestD;
        cur = del[best];
        route[step++] = cur;
    }
    totalCost += sp[cur][start];    
    route[step++] = start;
    return totalCost;
}

// Bitmask DP-TSP -- exact optimal tour 
int tspDP(int *nodes, int nd, int sp[MAX_LOC][MAX_LOC]) {
    int full = (1 << nd) - 1;
    for (int m = 0; m <= full; m++)
        for (int i = 0; i < nd; i++)
            dpMemo[m][i] = INF, dpPar[m][i] = -1;

    dpMemo[1][0] = 0;

    for (int mask = 1; mask <= full; mask++) {
        for (int u = 0; u < nd; u++) {
            if (!(mask & (1 << u)) || dpMemo[mask][u] == INF) continue;
            for (int v = 0; v < nd; v++) {
                if (mask & (1 << v)) continue;
                int newMask = mask | (1 << v);
                int newCost = dpMemo[mask][u] + sp[nodes[u]][nodes[v]];
                if (newCost < dpMemo[newMask][v]) {
                    dpMemo[newMask][v] = newCost;
                    dpPar[newMask][v]  = u;
                }
            }
        }
    }
    int best = INF;
    for (int u = 1; u < nd; u++) {
        if (dpMemo[full][u] == INF) continue;
        int cost = dpMemo[full][u] + sp[nodes[u]][nodes[0]];
        if (cost < best) best = cost;
    }
    return best;
}

// Nearest Insertion Heuristic -- approximation for larger inputs
int nearestInsertion(int *nodes, int nd, int sp[MAX_LOC][MAX_LOC]) {
    if (nd <= 1) return 0;
    int inTour[MAX_LOC] = {0};
    int tour[MAX_LOC + 1], tourLen = 0;

    // Start with depot and its nearest neighbor
    int closest = 1, minD = sp[nodes[0]][nodes[1]];
    for (int i = 2; i < nd; i++)
        if (sp[nodes[0]][nodes[i]] < minD) { minD = sp[nodes[0]][nodes[i]]; closest = i; }

    tour[tourLen++] = 0;
    tour[tourLen++] = closest;
    tour[tourLen++] = 0;
    inTour[0] = inTour[closest] = 1;

    for (int inserted = 2; inserted < nd; inserted++) {
        // Find nearest node not yet in tour
        int bestNode = -1, bestNodeDist = INF;
        for (int i = 0; i < nd; i++) {
            if (inTour[i]) continue;
            for (int j = 0; j < tourLen - 1; j++)
                if (sp[nodes[tour[j]]][nodes[i]] < bestNodeDist) {
                    bestNodeDist = sp[nodes[tour[j]]][nodes[i]];
                    bestNode = i;
                }
        }
        
        int bestPos = 1, bestInc = INF;
        for (int j = 0; j < tourLen - 1; j++) {
            int a = tour[j], b = tour[j + 1];
            int inc = sp[nodes[a]][nodes[bestNode]]
                    + sp[nodes[bestNode]][nodes[b]]
                    - sp[nodes[a]][nodes[b]];
            if (inc < bestInc) { bestInc = inc; bestPos = j + 1; }
        }
      
        for (int j = tourLen; j > bestPos; j--) tour[j] = tour[j - 1];
        tour[bestPos] = bestNode;
        tourLen++;
        inTour[bestNode] = 1;
    }

    int total = 0;
    for (int j = 0; j < tourLen - 1; j++)
        total += sp[nodes[tour[j]]][nodes[tour[j + 1]]];
    return total;
}

// Round-robin delivery assignment across agents
void assignDeliveries(int *del, int nd, int nAgents,
                      int agentJobs[][MAX_LOC], int agentCount[]) {
    for (int i = 0; i < nAgents; i++) agentCount[i] = 0;
    for (int i = 0; i < nd; i++) {
        int a = i % nAgents;
        agentJobs[a][agentCount[a]++] = del[i];
    }
}

// Print a route as location names
void printRoute(int *route, int len) {
    for (int i = 0; i < len; i++) {
        printf("%s", locName[route[i]]);
        if (i < len - 1) printf(" -> ");
    }
    printf("\n");
}

// Print final cost comparison table
void printComparison(int totalGreedy, int totalDP, int totalHeuristic) {
    printf("\n========================================\n");
    printf("       TOTAL COST COMPARISON\n");
    printf("========================================\n");
    printf("  Greedy Nearest Neighbor  : %d\n", totalGreedy);
    printf("  Bitmask DP-TSP (optimal) : %d\n", totalDP);
    printf("  Nearest Insertion Approx : %d\n", totalHeuristic);
    double gap = totalDP > 0 ? 100.0 * (totalGreedy - totalDP) / totalDP : 0;
    printf("  Optimality gap (Greedy vs DP): %.1f%%\n", gap);
    printf("========================================\n");
}

//main function
int main() {
    initGraph();
    buildShortestPaths(sp);

    printf("=== SUDOS: Smart Urban Delivery Optimization System ===\n\n");
    printf("Locations (%d): ", n);
    for (int i = 0; i < n; i++) printf("%s  ", locName[i]);
    printf("\n\n");

    int agentJobs[MAX_AGENTS][MAX_LOC];
    int agentCount[MAX_AGENTS];
    assignDeliveries(deliveries, NUM_DELIVERIES, NUM_AGENTS, agentJobs, agentCount);

    int totalGreedy = 0, totalDP = 0, totalHeuristic = 0;
    clock_t t1, t2;

    printf("--- PER AGENT RESULTS ---\n\n");

    for (int a = 0; a < NUM_AGENTS; a++) {
        printf("Agent %d | Deliveries: ", a + 1);
        for (int i = 0; i < agentCount[a]; i++) printf("%s ", locName[agentJobs[a][i]]);
        printf("\n");

        // Build nodes array: 
        int nodes[MAX_LOC];
        nodes[0] = 0;
        for (int i = 0; i < agentCount[a]; i++) nodes[i + 1] = agentJobs[a][i];
        int nodeCount = agentCount[a] + 1;

        // Greedy
        int route[MAX_LOC + 2];
        t1 = clock();
        int gCost = greedyRoute(0, agentJobs[a], agentCount[a], route, sp);
        t2 = clock();
        printf("  Greedy NN  | Cost: %3d | Time: %.2f us | Route: ", gCost,
               (double)(t2 - t1) / CLOCKS_PER_SEC * 1e6);
        printRoute(route, agentCount[a] + 2);
        totalGreedy += gCost;

        // DP-TSP
        if (nodeCount <= 12) {
            t1 = clock();
            int dCost = tspDP(nodes, nodeCount, sp);
            t2 = clock();
            printf("  DP-TSP     | Cost: %3d | Time: %.2f us\n", dCost,
                   (double)(t2 - t1) / CLOCKS_PER_SEC * 1e6);
            totalDP += dCost;
        } else {
            printf("  DP-TSP     | Skipped (nodeCount > 12)\n");
            totalDP += gCost;
        }

        // Heuristic
        t1 = clock();
        int hCost = nearestInsertion(nodes, nodeCount, sp);
        t2 = clock();
        printf("  Heuristic  | Cost: %3d | Time: %.2f us\n\n", hCost,
               (double)(t2 - t1) / CLOCKS_PER_SEC * 1e6);
        totalHeuristic += hCost;
    }
    printComparison(totalGreedy, totalDP, totalHeuristic);
    return 0;
}
