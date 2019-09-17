#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <memory>

using namespace std;

// A structure to represent each waypoint 
struct Waypoint {
	int x, y, penaltyTime;
	Waypoint(int x, int y, int penaltyTime) :
		x(x), y(y), penaltyTime(penaltyTime) {}
};

// A utility function to find the vertex index with minimum time value 
int minTimeIndex(shared_ptr<float[]> time, shared_ptr<bool[]> haveSeen, const vector<Waypoint>& waypoints, int vertices) {
	// Initialize min value 
	int minIndex = 0;
	int minTimeIndex, updatedMinTimeIndex;
	int newAddedProcessedNode = 0;
	vector<float> timeSet;

	for (int vertex = 1; vertex < vertices; vertex++) {
		if (haveSeen[vertex] == false) {
			timeSet.emplace_back(time[vertex]);
		}
		else {
			newAddedProcessedNode++;
		}
	}
	// Leverage algorithm header
	minTimeIndex = min_element(timeSet.begin(), timeSet.end()) - timeSet.begin();
	// we need add number of nodes which have been processed to get the correct index 
	// in the whole set
	updatedMinTimeIndex = minTimeIndex + newAddedProcessedNode + 1;
	return updatedMinTimeIndex;
}

// Function that implements Dijkstra's single source shortest path/time algorithm 
// for a graph
float dijkstra(const vector<Waypoint>& waypoints, int vertices) {
	float minTimeForEachVertex;
	shared_ptr<float[]> time(new float[vertices]); // dynamically allocated array to store shortese  
									               // time for each vertex because number of vertices
									               // are changable
	
	// std::make_shared doesn't work with raw array, code below errored out
	// shared_ptr<float[]> time = make_shared<float[]>(vertices);
	
	shared_ptr<bool[]> haveSeen(new bool[vertices]); // True if vertex is included in shortest time 
										             // tree or shortest time from start point to 
										             // vertex is finalized

	// Initialize haveSeen[] as false because we have not traversed any node yet
	for (int vertex = 0; vertex < vertices; vertex++) {
		haveSeen[vertex] = false;
	}

	// time of source vertex from itself is always 0 
	time[0] = 0.0f;
	haveSeen[0] = true;

	// Initialize and calculate all time taken from the source directly to all its 
	// children which in this case are all the nodes except the source node
	for (int vertex = 1; vertex < vertices; vertex++) {
		// 'cause the speed is 2m/s, time consumed can be calulated instead of path
		minTimeForEachVertex = float(sqrt(pow(waypoints[vertex].x, 2) + pow(waypoints[vertex].y, 2)) / 2);
		// Declare some local variable to calculate consumed time by bypassing 
		// intermediate nodes
		int penaltyIndex = vertex;
		float accumulatedPenaltyTime = 0.0f;
		while (penaltyIndex > 1) {
			accumulatedPenaltyTime += waypoints[penaltyIndex - 1].penaltyTime;
			penaltyIndex--;
		}
		// add 10 because robot will stop 10 seconds at all nodes except the source
		time[vertex] = minTimeForEachVertex + accumulatedPenaltyTime + 10;
	}

	// Find shortest time for all vertices
	for (int vertex = 1; vertex < vertices; vertex++) {
		// Pick the minimum time vertex from the set of vertices not 
		// yet processed.
		int minIndex = minTimeIndex(time, haveSeen, waypoints, vertices);
		float minTimeValue = time[minIndex];

		// Update time value of the adjacent vertex of the picked vertex minIndex.
		for (int vertex = minIndex; vertex < vertices - 1; vertex++) {
			minTimeForEachVertex = float(sqrt(pow((waypoints[minIndex].x - waypoints[vertex + 1].x), 2) + pow((waypoints[minIndex].y - waypoints[vertex + 1].y), 2)) / 2);

			float omitedNodeTime = 0.0f;
			int certainIndex = vertex;
			// calc penalty time of omited nodes
			while (certainIndex > minIndex) {
				omitedNodeTime += waypoints[certainIndex].penaltyTime;
				certainIndex--;
			}
			// Compare existing value of nodes with time consumed by detoured path
			// and find the smaller value and update the time array and it will keep
			// "have seen" nodes intact
			if (minTimeValue + minTimeForEachVertex + 10 + omitedNodeTime < time[vertex + 1]) {
				time[vertex + 1] = minTimeValue + minTimeForEachVertex + 10 + omitedNodeTime;
			}
		}

		// Mark the picked vertex as have seen and processed
		if (!haveSeen[minIndex]) {
			haveSeen[minIndex] = true;
			time[minIndex] = minTimeValue;
		}
	}
	// set decimal precision
	cout << fixed << setprecision(3);
	// return the shortest time reaching to end point
	return time[vertices - 1];
}

int main() {

	int numberOfWaypoints;
	int numberOfWaypointsInnerLoop; // used to loop through different waypoint data set
	int vertices, xCoord, yCoord, penalty;

	vector<float> output;

	while (true) {

		typedef vector<Waypoint> VectorofWaypoint;
		unique_ptr<VectorofWaypoint> waypoints = make_unique<VectorofWaypoint>();

		cin >> numberOfWaypoints;
		if (numberOfWaypoints <= 1000 && numberOfWaypoints >= 1) {
			numberOfWaypointsInnerLoop = numberOfWaypoints;
			vertices = numberOfWaypoints + 2;
			waypoints->emplace_back(0, 0, 0); // Start Point

			while (numberOfWaypointsInnerLoop > 0) {
				// User type in data of all waypoints
				// a while loop is utilized to catch variables typed in out of range
				while (true) {
					cin >> xCoord >> yCoord >> penalty;
					if (xCoord < 1 || xCoord > 99 || yCoord < 1 || yCoord > 99 || penalty < 1 || penalty > 100) {
						cout << "Values typed in are out of range, please re-enter them. Note: X or Y coordinate"
							" should be in [1, 99] and Penalty value be in [1, 100]." << endl;
					} else {
						break;
					}			
				}
				waypoints->emplace_back(xCoord, yCoord, penalty);
				numberOfWaypointsInnerLoop--;
			}
			waypoints->emplace_back(100, 100, 10); // End Point
			output.emplace_back(dijkstra(*waypoints, vertices));
		} else if (numberOfWaypoints == 0) {
			break;
		}
		else {
			cout << "Please re-enter number of waypoints. (it should be from 1 to 1000)" << endl;
		}
		cin.get();
	}
	// print out the output
	for (vector<float>::iterator it = output.begin(); it != output.end(); it++) {
		cout << *it << endl;
	}
	cin.ignore();
	cin.get();
	return 0;
}