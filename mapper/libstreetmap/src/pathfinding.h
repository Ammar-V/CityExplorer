#include <unordered_map>


#include "StreetsDatabaseAPI.h"
#include "loadFunctions.h"
#include "m4.h"



#ifndef PATHFINDING_H
#define PATHFINDING_H

#define THREE_OPT_TIME 5
#define TWO_OPT_TIMEOUT 0.05
#define GLOBAL_TIMEOUT 20
#define GLOBAL_TIMEOUT2 80



// Class for Priority Queue used in A*
class QueueElem {
    public:
        IntersectionIdx nodeId;
        StreetSegmentIdx reachingEdge;
        double travelTime;
        double heuristic; // Euclidean

        // Default constructor
        QueueElem ();
        QueueElem (int node, int edge, double time, double h);

        // Overloading function for priority queue
        bool operator() (const QueueElem &lhs, const QueueElem &rhs);

};

// Struct to keep add to our hash map to see if node visited or not
struct Node {
    IntersectionIdx nodeId;
    bool visited;
    StreetSegmentIdx reachingEdge;

    double bestTime; // Shortest time to get to this node
};

// Helper Functions for M3
bool shortestPath (IntersectionIdx src, IntersectionIdx dest, double time_penalty, MapData* map);
double calculateHeuristic (IntersectionIdx cur, LatLon destPosition, MapData* currentMap);

// Helper Structs for M4

struct SubPathInfo {
    IntersectionIdx idx;
    double time;
    std::vector<IntersectionIdx> subpath;

    SubPathInfo (IntersectionIdx id, double bestTime, std::vector<IntersectionIdx> subpathList) {
        idx = id;
        time = bestTime;
        subpath = subpathList;
    }

    bool operator() (const SubPathInfo &a, const SubPathInfo &b);

};

struct RouteInfo {
    double routeTime;
    std::vector<IntersectionIdx> route;

    RouteInfo () {}

    RouteInfo (double time, std::vector<IntersectionIdx> routeList) {
        routeTime = time;
        route = routeList;
    }
};

// M4 stuff
typedef int DeliveryIdx;
#define P_PARAM 3


// Functions for M4
bool shortestPathMulti (IntersectionIdx src, std::vector<IntersectionIdx> dest, 
                    double time_penalty, MapData* map, std::unordered_map<IntersectionIdx, Node> &localNodes, 
                    bool isDepot, IntersectionIdx &depot);

bool findPathBetweenIntersectionsMulti (
    const std::pair<IntersectionIdx, std::vector<IntersectionIdx>> intersection_ids,
    const double turn_penalty,
    std::vector<std::vector<SubPathInfo>> &kMatrix, const int kIdx
);

// Returns path to the closest depot
std::vector<StreetSegmentIdx> findPathToDepot (
    const std::pair<IntersectionIdx, std::vector<IntersectionIdx>> intersection_ids,
    const double turn_penalty, IntersectionIdx &lastDepot
);

void computeMatrix (const std::vector<DeliveryInf>& deliveries, const double turn_penalty);
std::vector<double> test(IntersectionIdx from);

int getRandIdx(int size);

bool checkRouteValid(const std::vector<IntersectionIdx> &route);
double computeRouteTime(std::vector<IntersectionIdx> &route);

void optimizeElite (RouteInfo &eliteRoute);

void threeOpt (std::vector<IntersectionIdx> &parentRoute, std::vector<RouteInfo> &childRoutes);

void combinationsRecursive(std::vector<std::vector<IntersectionIdx>> segmentVector, std::vector<RouteInfo> &childRoutes, std::vector<std::vector<IntersectionIdx>> &currentCombination, int currentIdx);

void orderCrossover(RouteInfo &parent1Route, RouteInfo &parent2Route);
double updateTemperature(double x);
void twoOptSingle (RouteInfo &parentRoute);
void twoOpt(RouteInfo &parentRoute, std::vector<RouteInfo> &subpool);
void largeNSearch(std::vector<IntersectionIdx> &parentRoute, std::vector<RouteInfo> &subpool);
int factorial (int num);

// Map keeps only visited nodes
extern std::unordered_map<IntersectionIdx, Node> Nodes;

//contains shortest path
extern std::vector<StreetSegmentIdx> pathSegments;



#endif // PATHFINDING_H