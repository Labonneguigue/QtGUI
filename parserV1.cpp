//
//  parserV1.cpp
//  OpenCVParser
//
//  Created by Guillaume GRANIE on 06/12/2016.
//
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rapidxml_utils.hpp"
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "parserV1.hpp"

using namespace rapidxml;
using namespace cv;
using namespace std;

double Map::Min_lon = 50;
double Map::Max_lon = 0;
double Map::Min_lat = 50;
double Map::Max_lat = 0;
double Map::Alpha = 0;
double Map::Beta = 0;
double Map::Delta_Lon = 0;
double Map::Delta_Lat = 0;
int Map::CurrentRoad = 0;
cv::Mat Map::image;
cv::Mat Map::imageClose;

void Map::BuildAllRoads(int id_road, vector<Node> VEC){
	//Create the first road
	int indexFirst = 0;
	int index = 1;
	Node begin = VEC[0];
	Node end = VEC[1];
	Road * myRoad = new Road(id_road, begin, end);
	//Check if we have more than 2 nodes
	while (index<VEC.size()-1){
		//And loop through them
		index++;
		Node TempEndNode = VEC[index];
		if (abs(ComputeCoefA(begin, TempEndNode) - myRoad->GetA()) < 0.0001){
			// Points on the same straight line
			myRoad->SetEndNode(TempEndNode);
			end = TempEndNode;
		} else {
			// Add the current road to the list and start a new one;
			vector<Node>::const_iterator first = VEC.begin() + indexFirst;
			vector<Node>::const_iterator last = VEC.begin() + index;
			vector<Node> VecRoadNodes_Local(first, last);
			indexFirst = index-1;
			myRoad->SetVecRoadNodes(VecRoadNodes_Local);
			Road_Vec.push_back(myRoad);
			begin = end;
			end = TempEndNode;
			myRoad = new Road(BaseIndexToCount++, begin, end);
		}
	}
	vector<Node>::const_iterator first = VEC.begin() + indexFirst;
	vector<Node>::const_iterator last = VEC.begin() + index;
	vector<Node> VecRoadNodes_Local(first, last);
	indexFirst = index -1;
	myRoad->SetVecRoadNodes(VecRoadNodes_Local);
	Road_Vec.push_back(myRoad);
}

float Map::ComputeCoefA(Node begin, Node end){
	return (end.GetLatitude() - begin.GetLatitude())/(end.GetLongitude() - begin.GetLongitude());
}

int Map::CreateAll(int close, cv::Mat imageToWriteOn){
	imageToWriteOn = Mat::zeros(Size(LARGEUR_FENETRE, HAUTEUR_FENETRE), CV_8UC3);
	DisplayAllRoads(Road_Vec, close, imageToWriteOn);
	DisplayAllBuildings(Building_Vec, close, imageToWriteOn);
	DisplayAllUserNodes(User_Node, close, imageToWriteOn);
	return 1;
}

float Map::CapAlgorithm(){
	// Need to have called SetPosition() right before and 5 meters before
	// Our CurrentPosition_Lon and CurrentPosition_Lat are the GPS position.
	// Our PreviousPosition_Lon and PreviousPosition_Lat are the origins.
	// distance in [meters]
	float Corrective_Cap = 0.0;

	if (PathSet){
		std::cout << "Distance brute avant correction de position: " << DirectDistance(CurrentPosition_Lon, CurrentPosition_Lat, PreviousPosition_Lon, PreviousPosition_Lat) << '\n';
		double distance = DirectDistance(CurrentPosition_Lat, CurrentPosition_Lon, PreviousPosition_Lat, PreviousPosition_Lon);
		// We assume an uncertainty of 2.5 meters on the GPS measure.
		float Cap_Actuel = 0;
		float Cap_Destination = 0;

		Cap_Actuel = ComputeCoefA(Node(1, CurrentPosition_Lon, CurrentPosition_Lat), Node(2, PreviousPosition_Lon, PreviousPosition_Lat));

		//Correction de position
		float rapport = DISTANCE_BETWEEN_ACQUISITIONS / distance;
		CurrentPosition_Lon = PreviousPosition_Lon + rapport * (CurrentPosition_Lon - PreviousPosition_Lon);
		CurrentPosition_Lat = PreviousPosition_Lat + rapport * (CurrentPosition_Lat - PreviousPosition_Lat);

		std::cout << "Distance normalement égale à 5 après Correction de position: " << DirectDistance(CurrentPosition_Lon, CurrentPosition_Lat, PreviousPosition_Lon, PreviousPosition_Lat) << '\n';
		if (PathToDestination.size() != 0){
			double Local_DestinationPosition_Lon = PathToDestination[CurrentIntermediateDestinationNode]->GetLongitude();
			double Local_DestinationPosition_Lat = PathToDestination[CurrentIntermediateDestinationNode]->GetLatitude();
			std::cout << "A" << '\n';

			double RemainingDistance = DirectDistance(CurrentPosition_Lon, CurrentPosition_Lat, DestinationPosition_Lon, DestinationPosition_Lat);
			std::cout << "B" << '\n';
			// We enter here if we are closer than 5 meters from the intermediate Node
			while(RemainingDistance < 5){
				// Test if we were heading to the last intermediate Node before final destination
				if (CurrentIntermediateDestinationNode == PathToDestination.size() - 1){
					// If so, we are now heading to the final destination ( user defined point)
					Local_DestinationPosition_Lon = DestinationPosition_Lon;
					Local_DestinationPosition_Lat = DestinationPosition_Lat;
					CurrentIntermediateDestinationNode = 0;
				} else {
					// else, we are now heading to the next intermediate node
					CurrentIntermediateDestinationNode++;
					Local_DestinationPosition_Lon = PathToDestination[CurrentIntermediateDestinationNode]->GetLongitude();
					Local_DestinationPosition_Lat = PathToDestination[CurrentIntermediateDestinationNode]->GetLatitude();
				}
				RemainingDistance = DirectDistance(CurrentPosition_Lon, CurrentPosition_Lat, Local_DestinationPosition_Lon, Local_DestinationPosition_Lat);
			}
			std::cout << "C" << '\n';
			Cap_Destination = ComputeCoefA(Node(1, Local_DestinationPosition_Lon, Local_DestinationPosition_Lat), Node(1, PreviousPosition_Lon, PreviousPosition_Lat));

			Cap_Actuel = atan(Cap_Actuel) * 180 / LOCAL_PI;
			Cap_Destination = atan(Cap_Destination) * 180 / LOCAL_PI;

			float Cap_Derive = (Cap_Destination - Cap_Actuel);

			float OppositeAngle = asin(sin(Cap_Derive)*5/RemainingDistance);

			Corrective_Cap = OppositeAngle + Cap_Derive;
		} else {
			std::cout << "Can't compute corrective cap since size of PathToDestination vector is 0" << '\n';
		}
	} else {
		//Path not set
		std::cout << "Path NOT SET ! Error" << '\n';
	}
	return Corrective_Cap;
}


// Position
int Map::DisplayImage(int close, int waitKey_Local){
	if (close){
		CreateAll(1, imageClose);
		DisplayCloseMyPosition();
		imshow("Closer Image",imageClose);
	} else {
		CreateAll(0, image);
		DisplayMyPosition();
		imshow("Image",image);
	}
	waitKey(waitKey_Local);
	return 1;
}

int Map::DisplayMyPosition(){
	circle(image, Point(GetDisplayX(CurrentPosition_Lon), GetDisplayY(CurrentPosition_Lat)),  5, Scalar(0, 0, 255, 255), -1, 8, 0);
	return 1;
}

int Map::DisplayCloseMyPosition(){
	circle(imageClose, Point(GetCloseDisplayX(CurrentPosition_Lon), GetCloseDisplayY(CurrentPosition_Lat)),  5, Scalar(0, 0, 255, 255), -1, 8, 0);
	return 1;
}

int Map::DisplayCloseToLocation(cv::Mat imageToWriteOn){
	//Position in the center of the screen
	SetBeta();
	CreateAll(1, imageToWriteOn);
	return 1;
}

// Distance between two points (longitude, latitude)
double Map::DirectDistance(double lat1, double lng1, double lat2, double lng2)
{
	double earthRadius = 6371000; //meters
	double dLat = ToRadians(lat2-lat1);
	double dLng = ToRadians(lng2-lng1);
	double a = sin(dLat/2) * sin(dLat/2) +
	cos(ToRadians(lat1)) * cos(ToRadians(lat2)) *
	sin(dLng/2) * sin(dLng/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	float dist = (float) (earthRadius * c);
	return dist;
}

//Draw the buildings on the image (or imageClose)
void Map::DisplayAllBuildings(vector<Building *> v, int close, cv::Mat imageToWriteOn){
	for(vector<Building *>::iterator it = v.begin(); it != v.end(); ++it) {
		(*it)->Display(close, imageToWriteOn);
	}
}

void Map::DisplayAllUserNodes(vector<Node *> v, int close, cv::Mat imageToWriteOn){
	for(vector<Node *>::iterator it = v.begin(); it != v.end(); ++it) {
		(*it)->Display(close, imageToWriteOn);
	}
}

//	Display every roads and the one we are on with a different color
void Map::DisplayAllRoads(vector<Road *> v, int close, cv::Mat imageToWriteOn){
	int i = 0;
	for(vector<Road *>::iterator it = v.begin(); it != v.end(); ++it) {
		(*it)->Display(close, imageToWriteOn);
		i++;
	}
	cout << "roads displayed : " << i << '\n';
}

void Map::DisplayPath(int close, cv::Mat imageToWriteOn){
	if (PathToDestination.size() != 0){
		for (int i=0 ; i<PathToDestination.size() ; i++){
			GetNodeById(PathToDestination[i]->GetId())->DisplayAsPathNode(close, imageToWriteOn);
		}
	} else {
		std::cout << "Can't display a path without nodes" << '\n';
	}
}

int Map::GetCloseDisplayX(double lon){
	double temp = ((lon - Min_lon)*Beta) - Delta_Lon;
	return temp;
}

int Map::GetCloseDisplayY(double lat){
	double temp = ((Max_lat - lat)*Beta) - Delta_Lat;
	return temp;
}

int Map::GetDisplayX(double lon){
	double temp = ((lon - Min_lon)*Alpha) + 10;
	return temp;
}

int Map::GetDisplayY(double lat){
	double temp = ((Max_lat - lat)*Alpha) + 10;
	return temp;
}

Node * Map::GetClosestNode(){
	Node * CurrentClosestNodeLocal = NULL;
	double Distance_Min = 5000;
	std::cout << "A" << '\n';
	Road * CurrentRoad = GetRoadById(WhichRoadWithLatLon());
	if (CurrentRoad != NULL){
		vector<Node> MyRoadNodes = CurrentRoad->GetRoadNodes();
		std::cout << "size " << MyRoadNodes.size() << '\n';
		if (MyRoadNodes.size() != 0){
			for(vector<Node>::iterator it = MyRoadNodes.begin(); it != MyRoadNodes.end(); ++it) {
				std::cout << "B" << '\n';
				double distanceLocal = DirectDistance((*it).GetLatitude(), (*it).GetLongitude(), CurrentPosition_Lat, CurrentPosition_Lon);
				if(distanceLocal < Distance_Min){
					Distance_Min = distanceLocal;
					CurrentClosestNodeLocal = &(*it);
				}
			}
		}
	} else {
		std::cout << "No current road" << '\n';
	}
	return CurrentClosestNodeLocal;
}

// Returns the Node with the specified ID
Node * Map::GetNodeById(long double id){
	Node * null = NULL;
	for(vector<Node *>::iterator it = Node_Vec.begin(); it != Node_Vec.end(); ++it) {
		if((*it)->GetId() == id){
			return *it;
		}
	}
	return null;
}

Road * Map::GetRoadById(double id){
	std::cout << "id road " << id << '\n';
	Road * null = NULL;
	for(vector<Road *>::iterator it = Road_Vec.begin(); it != Road_Vec.end(); ++it) {
		if((*it)->GetId() == id){
			return *it;
		}
	}
	return null;
}

vector<tuple<string, double>> Map::GetTupleOfDestinations(){
	vector<tuple<string, double>> MyTupleOfDestinations;
	for (int i=0 ; i<User_Node.size() ; i++){
		MyTupleOfDestinations.push_back(std::make_tuple(User_Node[i]->GetName(), User_Node[i]->GetId()));
	}
	return MyTupleOfDestinations;
}

// Set the alpha factor that convert longitude/latitude in pixels
void Map::SetAlpha(){
	double Alpha_lon = (LARGEUR_FENETRE - 20)/(Max_lon - Min_lon);
	double Alpha_lat = (HAUTEUR_FENETRE - 20)/(Max_lat - Min_lat);
	if (Alpha_lon < Alpha_lat){
		Alpha = Alpha_lon;
	} else {
		Alpha = Alpha_lat;
	}
	cout << "Alpha = " << Alpha << '\n';
}

void Map::SetBeta(){
	Beta = 5 * Alpha;
}

/*******************************SETPATH***********************************/
/*	This function set every points between the current/starting position */
/*	and the destination. 												 */
/*************************************************************************/
void Map::SetPath(vector<double> path){
	for (int i=0 ; i<path.size() ; i++){
		PathToDestination.push_back(GetNodeById(path[i]));
	}
	CurrentIntermediateDestinationNode = 0;
	PathSet = 1;

}

int Map::SetPosition(double lon, double lat){
	PreviousPosition_Lat = CurrentPosition_Lat;
	PreviousPosition_Lon = CurrentPosition_Lon;
	CurrentPosition_Lon = lon;
	CurrentPosition_Lat = lat;
	SetBeta();
	Delta_Lat = 0;
	Delta_Lon = 0;
	Delta_Lat = GetCloseDisplayY(CurrentPosition_Lat) - (HAUTEUR_FENETRE/2);
	Delta_Lon = GetCloseDisplayX(CurrentPosition_Lon) - (LARGEUR_FENETRE/2);
	return 1;
}

void Map::SetDestination(double lon, double lat){
	DestinationPosition_Lon = lon;
	DestinationPosition_Lat = lat;
}

void Map::SetDestination(double id){
	Node * DestNode = GetNodeById(id);
	DestinationPosition_Lon = DestNode->GetLongitude();
	DestinationPosition_Lat = DestNode->GetLatitude();
}

double Map::ToRadians(double degrees)
{
	double radians = degrees * LOCAL_PI / 180;
	return radians;
}

void Map::WhichRoad(double lon, double lat){
	int Road_Nb = 0;
	double Distance_Min = 5000;
	for(vector<Road *>::iterator it = Road_Vec.begin(); it != Road_Vec.end(); ++it) {
		double Temp_Distance = (*it)->DistanceToCenter(lon, lat);
		//cout << "Distance to road " << (*it)->id << " is " << Temp_Distance << '\n';
		if(Temp_Distance < Distance_Min){
			Road_Nb = (*it)->GetId();
			Distance_Min = Temp_Distance;
		}
	}
	CurrentRoad = Road_Nb;
}

double Map::WhichRoadWithLatLon(){
	if (CurrentPosition_Lon != 0 && CurrentPosition_Lat != 0){
		WhichRoad(CurrentPosition_Lon,CurrentPosition_Lat);
		std::cout << "Currently on road : " << CurrentRoad << '\n';
	} else {
		std::cout << "Current Position not set ! " << '\n';
	}
	return CurrentRoad;
}


Map::Map(rapidxml::file<> xmlFile){

	image = cv::Mat::zeros(cv::Size(LARGEUR_FENETRE, HAUTEUR_FENETRE), CV_8UC3);
	imageClose = cv::Mat::zeros(cv::Size(LARGEUR_FENETRE, HAUTEUR_FENETRE), CV_8UC3);

	Min_lon = 50;
	Max_lon = 0;
	Min_lat = 50;
	Max_lat = 0;
	Alpha = 0;
	Beta = 0;
	Delta_Lon = 0;
	Delta_Lat = 0;
	static int CurrentRoad = 0;

	try {
		xml_document<> doc;
		doc.parse<0>(xmlFile.data());
		xml_node<> * root = doc.first_node();



		double id = 0;
		double lon = 0;
		double lat = 0;
		cout.precision(12);
		double id_way = 0;
		xml_node<> * curr_node = root->first_node();

		while(curr_node){

			//Node
			if(strcmp(curr_node->name(), "node")==0){
				int user = 0;
				string name;
				id = 0;
				lon = 0;
				lat = 0;
				xml_attribute<> * attr = curr_node->first_attribute();
				while(attr){
					if (strcmp(attr->name(), "id") == 0){
						id = atof(attr->value());
					} else if (strcmp(attr->name(),  "lat") ==0){
						lat = atof(attr->value());
					} else if (strcmp(attr->name(), "lon") ==0){
						lon = atof(attr->value());
					}
					attr = attr->next_attribute();
				}//while
				xml_node<> * tag = curr_node->first_node();
				while(tag){
					if (strcmp(tag->name(), "tag") == 0){
						if (strcmp(tag->first_attribute()->name(), "k") == 0){
							if (strcmp(tag->first_attribute()->value(), "user") == 0){
								user = 1;
								name = string(tag->first_attribute()->next_attribute()->value());
							}
						}
					}
					tag = tag->next_sibling();
				}

				//Create node if we have all the needed values
				if (id != 0 && lon != 0 && lat != 0){
					Node * myNode = new Node(id, lon, lat);
					if (user){
						myNode->SetName(name);
						User_Node.push_back(myNode);
						myNode->ToString();
					} else {
						Node_Vec.push_back(myNode);
					}
					if (lon < Min_lon){
						Min_lon = lon;
					} else if (lon > Max_lon){
						Max_lon = lon;
					}
					if (lat < Min_lat){
						Min_lat = lat;
					} else if (lat > Max_lat){
						Max_lat = lat;
					}
				}
			} // Node

			//	Way
			if(strcmp(curr_node->name(), "way")==0){
				id_way = 0;
				int building = 0;
				int road = 0;
				string Name = "" ;
				vector<Node> Vec_Way_Node;

				if (strcmp(curr_node->first_attribute()->name(), "id") == 0){
					id_way = atoi(curr_node->first_attribute()->value());
					xml_node<> * curr_nd = curr_node->first_node();
					while(curr_nd){
						if (strcmp(curr_nd->name(), "nd") == 0){
							if(strcmp(curr_nd->first_attribute()->name(), "ref") == 0){
								double temp_id = (double)atof(curr_nd->first_attribute()->value());
								Node * Retrieved_Node = GetNodeById(temp_id);
								if (Retrieved_Node != NULL){
									if (Retrieved_Node->GetId() != 0){
										Vec_Way_Node.push_back(*Retrieved_Node);
									}
								} else {
									//cout << "Pb GetNodeById()" << '\n';
								}
							} else {
								//cout << "Pb ref" << '\n';
							}
						} else if (strcmp(curr_nd->name(), "tag") == 0){
							if (strcmp(curr_nd->first_attribute()->name(), "k") == 0){
								if (strcmp(curr_nd->first_attribute()->value(), "building") == 0){
									building = 1;
								} else if (strcmp(curr_nd->first_attribute()->name(), "name") == 0){
									xml_attribute<> * attribute = curr_nd->first_attribute();
									attribute = attribute->next_attribute();
									Name = string(attribute->value());
								} else if (strcmp(curr_nd->first_attribute()->value(), "highway") == 0){
									if ((strcmp(curr_nd->first_attribute()->next_attribute()->value(), "unclassified") == 0) ||
									(strcmp(curr_nd->first_attribute()->next_attribute()->value(), "service") == 0)){
										road = 1;
									}
								}
							}
						}
						curr_nd = curr_nd->next_sibling();
					}
					// Create the appropriate environment element if we have all the needed values
					if (Vec_Way_Node.size() >= 2){

						// Building ?
						if (building == 1){
							Building * myBuilding = new Building(id_way, Name, Vec_Way_Node);
							Building_Vec.push_back(myBuilding);
						}
						// Road ?
						else if (building == 0 && road == 1){
							BuildAllRoads(id_way, Vec_Way_Node);
							//Road myRoad(id_way, Vec_Way_Node[0], Vec_Way_Node[Vec_Way_Node.size()-1]);
							//Road_Vec.push_back(myRoad);
						}
					} else {
						//cout << "Only one point for this element" << '\n';
					}
				} else {
					//cout << "Pb id" << '\n';
				}
			}
			//	Next sibling
			curr_node = curr_node->next_sibling();

		}
		SetAlpha();
		std::cout << "Number of Nodes : " << Node_Vec.size() << '\n';
		std::cout << "Number of Roads : " << Road_Vec.size() << '\n';
		std::cout << "Number of User Nodes : " << User_Node.size() << '\n';


	}
	catch (const runtime_error& error)
	{
		cout << "Problem when opening the map file : " << '\n';
	}
}








/********************************/
/*								*/
/*			NODE				*/
/*								*/
/********************************/


void Node::ToString(){
	if (user){
		cout << "Node called : " << Name << "    with id : " << id << " longitude : " << longitude << " and latitude : " << latitude << "\n";
	} else {
		cout << "My node with id : " << id << " longitude : " << longitude << " and latitude : " << latitude << "\n";
	}
}
Node::Node(){
	id = 0;
}

Node::Node(double Node_Id, float lon, float lat){
	id = Node_Id;
	longitude = lon;
	latitude = lat;
}

bool Node::operator< (const Node &other) {
	return id < other.id;
}

void Node::SetName(string NAME){
	Name = NAME;
	user = 1;
}

void Node::Display(int close, cv::Mat imageToWriteOn){
	if (close){
		circle(imageToWriteOn, Point(Map::GetCloseDisplayX(longitude), Map::GetCloseDisplayY(latitude)),  5, Scalar(0, 20, 170, 255), 1, 8, 0);
		putText(imageToWriteOn, Name, cvPoint(Map::GetCloseDisplayX(longitude)+5,Map::GetCloseDisplayY(latitude)+5),
		FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200,200,250), 1, CV_AA);
	} else {
		if (user){
			circle(imageToWriteOn, Point(Map::GetDisplayX(longitude), Map::GetDisplayY(latitude)),  5, Scalar(0, 20, 170, 255), 1, 8, 0);
			putText(imageToWriteOn, Name, cvPoint(Map::GetDisplayX(longitude)+5,Map::GetDisplayY(latitude)+5),
			FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200,200,250), 1, CV_AA);
		}
	}
}

void Node::DisplayAsPathNode(int close, cv::Mat imageToWriteOn){
	if (close){
		circle(imageToWriteOn, Point(Map::GetCloseDisplayX(longitude), Map::GetCloseDisplayY(latitude)),  5, Scalar(252, 189, 47, 255), 1, 8, 0);
	} else {
		circle(imageToWriteOn, Point(Map::GetDisplayX(longitude), Map::GetDisplayY(latitude)),  5, Scalar(252, 189, 47, 255), 1, 8, 0);
	}
}

float Node::GetLatitude(){
	return latitude;
}
float Node::GetLongitude(){
	return longitude;
}
double Node::GetId(){
	return id;
}

string Node::GetName(){
	return Name;
}








/********************************/
/*								*/
/*			ROAD				*/
/*								*/
/********************************/

void Road::Compute_Coefs(){
	a = (end.GetLatitude() - begin.GetLatitude())/(end.GetLongitude() - begin.GetLongitude());
	b = begin.GetLatitude() - begin.GetLongitude() * a;
}

void Road::ToString(){
	cout.precision(12);
	cout << "My road with id : " << id << " has an equation : " << a << " x + " << b << "\n";
}

Road::Road(double Road_Id, vector<Node> VEC){
	id = Road_Id;
	Road_Nodes = VEC;
	Compute_Coefs();
}

Road::Road(double Road_Id, Node b, Node e){
	id = Road_Id;
	begin = b;
	end = e;
	Compute_Coefs();
}

void Road::SetEndNode(Node e){
	end = e;
	Compute_Coefs();
}

void Road::SetVecRoadNodes(std::vector<Node> VEC){
	Road_Nodes = VEC;
	std::cout << "Size of Road nodes vec " << Road_Nodes.size() << '\n';
}

double Road::DistanceToCenter(double lon, double lat){
	double Proj_Lon = (lon + a*lat - a*b)/(1 + a*a);
	double Proj_Lat = (a*lon + a*a*lat + b)/(1 + a*a);
	if (Proj_Lat > begin.GetLatitude() && Proj_Lat > end.GetLatitude()){
		if (end.GetLatitude() > begin.GetLatitude()){
			return Map::DirectDistance(lat, lon, end.GetLatitude(), end.GetLongitude());
		} else {
			return Map::DirectDistance(lat, lon, begin.GetLatitude(), begin.GetLongitude());
		}
	} else if (Proj_Lat < begin.GetLatitude() && Proj_Lat < end.GetLatitude()){
		if (end.GetLatitude() < begin.GetLatitude()){
			return Map::DirectDistance(lat, lon, end.GetLatitude(), end.GetLongitude());
		} else {
			return Map::DirectDistance(lat, lon, begin.GetLatitude(), begin.GetLongitude());
		}
	} else {
		return Map::DirectDistance(lat, lon, Proj_Lat, Proj_Lon);
	}
}

void Road::Display(int close, cv::Mat imageToWriteOn){
	if (close){
		if (!Map::Beta){
			Map::SetBeta();
		}
		if (id == Map::CurrentRoad){
			line(imageToWriteOn, Point(Map::GetCloseDisplayX(begin.GetLongitude()),Map::GetCloseDisplayY(begin.GetLatitude())), Point(Map::GetCloseDisplayX(end.GetLongitude()), Map::GetCloseDisplayY(end.GetLatitude())), Scalar( 255, 0, 0, 255),  1, 0 );
		} else {
			line(imageToWriteOn, Point(Map::GetCloseDisplayX(begin.GetLongitude()),Map::GetCloseDisplayY(begin.GetLatitude())), Point(Map::GetCloseDisplayX(end.GetLongitude()), Map::GetCloseDisplayY(end.GetLatitude())), Scalar( 110, 220, 100),  1, 0 );
		}
	} else {
		if (Map::Alpha){		// Alpha needs to be defined to get (x,y) coordinates
			int y1 = Map::GetDisplayY(begin.GetLatitude());
			int x1 = Map::GetDisplayX(begin.GetLongitude());
			int y2 = Map::GetDisplayY(end.GetLatitude());
			int x2 = Map::GetDisplayX(end.GetLongitude());
			if (id == Map::CurrentRoad){
				line(imageToWriteOn, Point(x1,y1), Point(x2, y2), Scalar( 255, 0, 0, 255),  1, 0 );
			} else {
				line(imageToWriteOn, Point(x1,y1), Point(x2, y2), Scalar( 110, 220, 100),  1, 0 );
			}
		}
	}
}

vector<Node> Road::GetRoadNodes(){
	return Road_Nodes;
}

float Road::GetA(){
	return a;
}
float Road::GetB(){
	return b;
}
double Road::GetId(){
	return id;
}
Node Road::GetBegin(){
	return begin;
}
Node Road::GetEnd(){
	return end;
}


/********************************/
/*								*/
/*			BUILDING			*/
/*								*/
/********************************/

Building::Building(int ID, string NAME, vector<Node> VEC){
	id = ID;
	Name = NAME;
	Building_Corner_Vec = VEC;
}

//	Enable the building to have more than 4 walls.
void Building::Display(int close, cv::Mat imageToWriteOn){
	if (close){
		if (Map::Beta){
			long VecSize = Building_Corner_Vec.size();
			long NumberOfCoordinates = VecSize*2;
			int XAndYs [NumberOfCoordinates];						// EVEN numbers are the Xs and ODD numbers are Ys
			XAndYs[0] = Map::GetCloseDisplayX(Building_Corner_Vec[0].GetLongitude());
			XAndYs[1] = Map::GetCloseDisplayY(Building_Corner_Vec[0].GetLatitude());
			int index;
			for(index = 1; index < VecSize ; index++){
				XAndYs[index*2] = Map::GetCloseDisplayX(Building_Corner_Vec[index].GetLongitude());
				XAndYs[index*2+1] = Map::GetCloseDisplayY(Building_Corner_Vec[index].GetLatitude());
				line(imageToWriteOn, Point(XAndYs[(index-1)*2],XAndYs[(index-1)*2+1]), Point(XAndYs[index*2], XAndYs[index*2+1]), Scalar( 0, 220, 100, 255),  1, 0 );
			}
			line(imageToWriteOn, Point(XAndYs[(index-1)*2],XAndYs[(index-1)*2+1]), Point(XAndYs[0], XAndYs[1]), Scalar( 0, 220, 100, 255),  1, 0 );
		}
	} else {
		if (Map::Alpha){
			long VecSize = Building_Corner_Vec.size();
			long NumberOfCoordinates = VecSize*2;
			int XAndYs [NumberOfCoordinates];						// EVEN numbers are the Xs and ODD numbers are Ys
			XAndYs[0] = Map::GetDisplayX(Building_Corner_Vec[0].GetLongitude());
			XAndYs[1] = Map::GetDisplayY(Building_Corner_Vec[0].GetLatitude());
			int index;
			for(index = 1; index < VecSize ; index++){
				XAndYs[index*2] = Map::GetDisplayX(Building_Corner_Vec[index].GetLongitude());
				XAndYs[index*2+1] = Map::GetDisplayY(Building_Corner_Vec[index].GetLatitude());
				line(imageToWriteOn, Point(XAndYs[(index-1)*2],XAndYs[(index-1)*2+1]), Point(XAndYs[index*2], XAndYs[index*2+1]), Scalar( 0, 220, 100, 255),  1, 0 );
			}
			line(imageToWriteOn, Point(XAndYs[(index-1)*2],XAndYs[(index-1)*2+1]), Point(XAndYs[0], XAndYs[1]), Scalar( 0, 220, 100, 255),  1, 0 );
		}
	}
}

string Building::GetName(){
	return Name;
}

double Building::GetId(){
	return id;
}
