#include <bits/stdc++.h>
#define INF 100000

using namespace std;

struct Point {
    double latitude, longitude;
    
    bool operator<(const Point& other) const {
        if (latitude != other.latitude) return latitude < other.latitude;
        return longitude < other.longitude;
    }
};

struct Edge {
    int to;
    double weight; //(distance in KM)
};


map<Point, int> pointToId;
vector<Point> idToPoint;

//vector<vector<Edge>> adj; 
vector<vector<pair<int, double>>> graph;



void readCSVandMakeMap();
int getID(const Point& p);
double toRadian(double degree);
double haversine(double lat1, double long1, double lat2, double long2);
pair<vector<int>, double> dijkstra(int start, int end);
int findNearestNode(Point target);
void generateKML(const vector<int>& path, Point source, Point destination);



int main() {

    readCSVandMakeMap(); 
    
    double sourceLat, sourceLong, destLat, destinationLong;
   // cout << "Enter Source (Lat Lon): ";
    cin >> sourceLat >> sourceLong;
   // cout << "Enter Destination (Lat Lon): ";
    cin >> destLat >> destinationLong;

    Point source = {sourceLat, sourceLong};
    Point destination = {destLat, destinationLong};


    int startNode = findNearestNode(source);
    int endNode = findNearestNode(destination);

    double walkDistToStart = haversine(source.latitude, source.longitude, idToPoint[startNode].latitude, idToPoint[startNode].longitude);
    double walkDistFromEnd = haversine(idToPoint[endNode].latitude, idToPoint[endNode].longitude, destination.latitude, destination.longitude);

    auto result = dijkstra(startNode, endNode);
    
    vector<int> path = result.first;  
    double roadDist = result.second;  

    
    cout<<"Problem 1: Shortest Car Route"<<endl<<endl;

    cout << "Source: ("<<source.longitude<< ", " <<source.latitude << ")" << endl;
    cout <<"Destination: ("<< destination.longitude << ", " << destination.latitude << ")" << endl;
    
    if (walkDistToStart > 0.0001) {
        cout << "Walk to intersection: (" << idToPoint[startNode].longitude << ", " << idToPoint[startNode].latitude << ")" << endl;
    }
    
    cout << "Drive Car from intersection to final road point for " << roadDist << " km." << endl;

    if (walkDistFromEnd > 0.0001) {
        cout << "Walk from road to destination: (" << destination.longitude << ", " << destination.latitude << ")" << endl;
    }

    cout << "Total Distance: " << (walkDistToStart + roadDist + walkDistFromEnd) << " km" << endl;

    generateKML(path, source, destination);

    return 0;
}




void readCSVandMakeMap(){
    
    ifstream dhakaCSV ("Roadmap-Dhaka.csv");

    if(!dhakaCSV.is_open()){
        cerr<<"Please have Roadmap-Dhaka.csv in the directory"<<endl;
    }

    string line;
    while (getline(dhakaCSV, line)) {
        if (line.empty()) continue;

        stringstream stream(line);
        string token;
        vector<string> tokens;

        while (getline(stream, token, ',')) {
            tokens.push_back(token);
        }

        vector<Point> roadPoints;
        for (int i=1; i<(int)tokens.size()-2 ;i+=2){

            try {
                double lon = stod(tokens[i]);
                double lat = stod(tokens[i+1]);
                roadPoints.push_back({lat, lon}); 
            } catch (...) {
                cout<<"Error converting "<<token[i]<< " and "<<token[i+1]<< "into double\nSkipping..."<<endl;
                continue; 
            }
        }
        
        if (roadPoints.size()<2){cout<<"Blank line "<<endl; continue;}

        for (int k = 0; k < (int)roadPoints.size() - 1; k++) {
            Point u = roadPoints[k];
            Point v = roadPoints[k+1];

            int uID = getID(u);
            int vID = getID(v);
            
            double w = haversine(u.latitude, u.longitude, v.latitude, v.longitude);

            graph[uID].push_back({vID, w});
            graph[vID].push_back({uID, w});

        }
    }
    dhakaCSV.close();

}



int getID(const Point& p) {
    if (pointToId.count(p)) {
        return pointToId[p];
    }

    int newID = idToPoint.size();
    pointToId[p] = newID;
    idToPoint.push_back(p);

    graph.push_back(vector<pair<int, double>>()); 

    return newID;
}

double toRadian(double degree){
    return degree*M_PI /180.0;
}


double haversine(double lat1, double long1, double lat2, double long2){
    double R=6371;

    lat1=toRadian(lat1); lat2=toRadian(lat2); 
    long1=toRadian(long1); long2=toRadian(long2);

    double dlong=long2-long1, dlat=lat2-lat1;

    // Haversine formula is 2.R. arcsin ( sqrt(a))
    // a is sin^2(dlat/2) + cos(long1).cos(long2).sin^2(dlong/2)

    double a= sin(dlat/2)*sin(dlat/2) + cos(lat1)*cos(lat2)*sin(dlong/2)*sin(dlong/2);

    //to solve the +- issue we will inverse using tan
    double ans= 2*R*atan2(sqrt(a),sqrt(1-a));

    return ans;
}



pair<vector<int>, double> dijkstra(int start, int end) {
    int n = idToPoint.size(); 
    
    vector<double> dist(n, INF); 
    vector<int> parent(n, -1);
    dist[start] = 0.0;

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    pq.push({0.0, start});

    while (!pq.empty()) {
        double d = pq.top().first;
        int id = pq.top().second;
        pq.pop();
        
        if (d > dist[id]) continue; 
        if (id == end) break;

        for (auto& edge : graph[id]) { 
            int neighborID = edge.first;
            double weight = edge.second;

            if (dist[id] + weight < dist[neighborID]) {
                dist[neighborID] = dist[id] + weight;
                parent[neighborID] = id;
                pq.push({dist[neighborID], neighborID});
            }
        }
    }

    vector<int> path;
    if (dist[end] == INF) {
        return {path, -1.0};     }

    for (int curr = end; curr != -1; curr = parent[curr]) {
        path.push_back(curr);
    }
    reverse(path.begin(), path.end());

    return {path, dist[end]}; 
}


int findNearestNode(Point target) {
    
    int bestId = -1;
    double min_dist = 1e18;
    for (int i = 0; i < idToPoint.size(); i++) {
        double d = haversine(target.latitude, target.longitude, idToPoint[i].latitude, idToPoint[i].longitude);
        if (d < min_dist) {
            min_dist = d;
            bestId = i;
        }
    }
    return bestId;
}

void generateKML(const vector<int>& path, Point source, Point destination) {
    ofstream kml("route.kml");
    //header
    kml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://earth.google.com/kml/2.1\">\n<Document>\n<Placemark>\n<name>Route</name>\n<LineString>\n<coordinates>\n";

    kml << source.longitude << "," << source.latitude << ",0\n";

    for(int id : path) {
        kml << idToPoint[id].longitude << "," << idToPoint[id].latitude << ",0\n";
    }

    kml << destination.longitude << "," << destination.latitude << ",0\n";
    kml << "</coordinates>\n</LineString>\n</Placemark>\n</Document>\n</kml>";
    kml.close();
}