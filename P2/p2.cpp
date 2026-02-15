#include <bits/stdc++.h>

using namespace std;

const double INF = 1e18;
const int CAR = 1;
const int METRO = 2;

struct Point {
    double latitude, longitude;
    bool operator<(const Point& other) const {
        if(longitude!=other.longitude)return longitude < other.longitude;
        return latitude<other.latitude;
    }
};

struct Edge {
    int to;
    double cost;
    int mode;
};

struct ParentInfo {
    int fromNode;
    int mode;
};


map<Point, int> pointToId;
vector<Point> idToPoint;
vector<vector<Edge>> graph;

int getID(const Point& p);
double toRadian(double degree);
double haversine(double lat1, double lon1, double lat2, double lon2);
void readRoadmapCSV();
void readMetroCSV();
int findNearestNode(Point target);
pair<vector<int>, vector<int>> dijkstra(int start, int end, double &totalCost);
void generateKML(const vector<int>& path, const vector<int>& modes, Point src, Point dest);


int main() {
    readRoadmapCSV();
    readMetroCSV();


    double sourceLon, sourceLat, destinationLon, destinationLat;
    if (!(cin >> sourceLat >> sourceLon >> destinationLat >> destinationLon)) {
        cerr<<"Please have input "<<endl;
        exit(1);
    }

    Point source = {sourceLat, sourceLon};
    Point destination = {destinationLat, destinationLon};

    
    int startID = findNearestNode(source);
    int endID = findNearestNode(destination);

    double totalCost = 0;
    auto result = dijkstra(startID, endID, totalCost);
    vector<int> path = result.first;
    vector<int> modes = result.second;

    cout << "Problem 2: Cheapest Route" << endl;
    cout << "Source: (" << source.longitude << ", " << source.latitude << ")" << endl;
    cout << "Destination: (" << destination.longitude << ", " << destination.latitude << ")" << endl;

    if (path.empty()) {
        cout << "No path found." << endl;
    } else {
        cout << "Total Cost in taka:"<<fixed << setprecision(2) << totalCost << endl;
        
        generateKML(path, modes, source, destination);
    }

    return 0;
}




int getID(const Point& p) {
    if (pointToId.count(p)) return pointToId[p];
    int newID = idToPoint.size();
    pointToId[p] = newID;
    idToPoint.push_back(p);
    graph.push_back({});
    return newID;
}

double toRadian(double degree) { 
    return degree * M_PI / 180.0; 
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    double R = 6371.0;
    double rLat1 = toRadian(lat1), rLat2 = toRadian(lat2);
    double destinationLat = toRadian(lat2 - lat1), destinationLon = toRadian(lon2 - lon1);
    double a = sin(destinationLat / 2) * sin(destinationLat / 2) + cos(rLat1) * cos(rLat2) * sin(destinationLon / 2) * sin(destinationLon / 2);
    return 2.0 * R * atan2(sqrt(a), sqrt(1 - a));
}

void readRoadmapCSV() {
    ifstream file("Roadmap-Dhaka.csv");
    if(!file.is_open()) {cerr<<"Please have Roadmap-Dhaka.csv in the directory"<<endl; exit(1); }
    string line;
    while (getline(file, line)) {
        if (line.empty()) continue;
        stringstream ss(line); string t; vector<string> tokens;
        while (getline(ss, t, ',')) tokens.push_back(t);
        
        vector<int> nodes;
        for (int i = 1; i < (int)tokens.size() - 2; i += 2) {
            // CSV is Lon, Lat
            nodes.push_back(getID({stod(tokens[i+1]), stod(tokens[i])}));
        }
        for (int k = 0; k < (int)nodes.size() - 1; k++) {
            double d = haversine(idToPoint[nodes[k]].latitude, idToPoint[nodes[k]].longitude, 
                                 idToPoint[nodes[k+1]].latitude, idToPoint[nodes[k+1]].longitude);
            graph[nodes[k]].push_back({nodes[k + 1], d * 20.0, CAR});
            graph[nodes[k + 1]].push_back({nodes[k], d * 20.0, CAR});
        }
    }
}

void readMetroCSV() {
    ifstream file("Routemap-DhakaMetroRail.csv");
    if(!file.is_open()) {cerr<<"Please have Routemap-DhakaMetroRail.csv in the directory"<<endl; exit(1); }
    string line;
    while (getline(file, line)) {
        if (line.empty()) continue;
        stringstream ss(line); string t; vector<string> tokens;
        while (getline(ss, t, ',')) tokens.push_back(t);
        
        vector<int> nodes;
        for (int i = 1; i < (int)tokens.size() - 2; i += 2) {
            nodes.push_back(getID({stod(tokens[i+1]), stod(tokens[i])}));
        }
        for (int k = 0; k < (int)nodes.size() - 1; k++) {
            double d = haversine(idToPoint[nodes[k]].latitude, idToPoint[nodes[k]].longitude, 
                                 idToPoint[nodes[k+1]].latitude, idToPoint[nodes[k+1]].longitude);
            graph[nodes[k]].push_back({nodes[k + 1], d * 5.0, METRO});
            graph[nodes[k + 1]].push_back({nodes[k], d * 5.0, METRO});
        }
    }
}

int findNearestNode(Point target) {
    int bestId = -1; double min_d = INF;
    for (int i = 0; i < (int)idToPoint.size(); i++) {
        double d = haversine(target.latitude, target.longitude, idToPoint[i].latitude, idToPoint[i].longitude);
        if (d < min_d) { min_d = d; bestId = i; }
    }
    return bestId;
}

pair<vector<int>, vector<int>> dijkstra(int start, int end, double &totalCost) {
    int n = idToPoint.size();
    vector<double> minCost(n, INF);
    vector<ParentInfo> parent(n, {-1, -1});
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    minCost[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        double c = pq.top().first; int u = pq.top().second; pq.pop();
        if (c > minCost[u]) continue;
        if (u == end) break;
        for (auto& edge : graph[u]) {
            if (minCost[u] + edge.cost < minCost[edge.to]) {
                minCost[edge.to] = minCost[u] + edge.cost;
                parent[edge.to] = {u, edge.mode};
                pq.push({minCost[edge.to], edge.to});
            }
        }
    }

    totalCost = minCost[end];
    vector<int> path, modes;
    if (minCost[end] == INF) return {path, modes};

    for (int curr = end; curr != -1; ) {
        path.push_back(curr);
        int prev = parent[curr].fromNode;
        if (prev != -1) modes.push_back(parent[curr].mode);
        curr = prev;
    }
    reverse(path.begin(), path.end());
    reverse(modes.begin(), modes.end());
    return {path, modes};
}

void generateKML(const vector<int>& path, const vector<int>& modes, Point src, Point dest) {
    ofstream kml("route.kml");
    kml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n";

    for (size_t i = 0; i < modes.size(); ++i) {
        kml << "<Placemark>\n";
        kml << "<LineString><coordinates>\n";
        kml << fixed << setprecision(6) << idToPoint[path[i]].longitude << "," << idToPoint[path[i]].latitude << ",0\n";
        kml << idToPoint[path[i+1]].longitude << "," << idToPoint[path[i+1]].latitude << ",0\n";
        kml << "</coordinates></LineString>\n</Placemark>\n";
    }
    kml << "</Document>\n</kml>";
}
