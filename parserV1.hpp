#ifndef PARSERV1_H
#define PARSERV1_H

#include <string>
#include <vector>

#include "rapidxml_utils.hpp"

#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define LOCAL_PI 3.1415926535897932385
#define LARGEUR_FENETRE 700
#define HAUTEUR_FENETRE 700
#define DISTANCE_BETWEEN_ACQUISITIONS 5
#define GPS_UNCERTAINTY 2.5

using namespace std;








/********************************/
/*								*/
/*		        NODE		    */
/*								*/
/********************************/

class Node{

private:
    std::string Name;
    int user = 0;
    double id;
    float latitude;
    float longitude;

public:
    void ToString();
    Node();

    Node(double Node_Id, float lon, float lat);

    void SetName(std::string NAME);

    void Display(int close, cv::Mat imageToWriteOn);

    void DisplayAsPathNode(int close, cv::Mat imageToWriteOn);

    float GetLatitude();
    float GetLongitude();
    double GetId();
    string GetName();

    bool operator< (const Node &other);
};







/********************************/
/*								*/
/*			   ROAD			    */
/*								*/
/********************************/

class Road{

private:
    double id;
    Node begin;
    Node end;
    float a;
    float b;
    int width;
    std::vector<Node> Road_Nodes;

    void Compute_Coefs();

public:
    void ToString();

    Road(double Road_Id, std::vector<Node> VEC);

    Road(double Road_Id, Node b, Node e);

    void SetEndNode(Node e);

    void SetVecRoadNodes(std::vector<Node> VEC);

    double DistanceToCenter(double lon, double lat);

    void Display(int close, cv::Mat imageToWriteOn);

    std::vector<Node> GetRoadNodes();
    float GetA();
    float GetB();
    double GetId();
    Node GetBegin();
    Node GetEnd();
};




/********************************/
/*								*/
/*			BUILDING			*/
/*								*/
/********************************/

class Building{

private:
    double id;
    std::string Name;
    std::vector<Node> Building_Corner_Vec;

public:
    Building(int ID, std::string NAME, std::vector<Node> VEC);

    //	Enable the building to have more than 4 walls.
    void Display(int close, cv::Mat imageToWriteOn);

    std::string GetName();

    double GetId();
};

/********************************/
/*								*/
/*			   MAP		        */
/*								*/
/********************************/

class Map {

private:
    static double Min_lon;
    static double Max_lon;
    static double Min_lat;
    static double Max_lat;
    static double Delta_Lon;
    static double Delta_Lat;

    int BaseIndexToCount = 1000;

    double CurrentPosition_Lon = 0;
    double CurrentPosition_Lat = 0;
    double PreviousPosition_Lon = 0;
    double PreviousPosition_Lat = 0;
    double DestinationPosition_Lon = 0;
    double DestinationPosition_Lat = 0;

    std::vector<Node *> Node_Vec;
    std::vector<Road *> Road_Vec;
    std::vector<Building *> Building_Vec;
    std::vector<Node *> User_Node;

    std::vector<Node *> PathToDestination;
    int CurrentIntermediateDestinationNode;
    int PathSet = 0;

    int IntermediateDestinationReached = 0;
    int FinalDestinationReached = 0;

public:
    static cv::Mat image;
    static cv::Mat imageClose;
    static double Alpha;
    static double Beta;
    static int CurrentRoad;

    static double ToRadians(double degrees);

    static double DirectDistance(double lat1, double lng1, double lat2, double lng2);

    static int GetDisplayX(double lon);

    static int GetDisplayY(double lat);

    static int GetCloseDisplayX(double lon);

    static int GetCloseDisplayY(double lat);

    Node * GetClosestNode();

    //	Display every roads and the one we are on with a different color
    void DisplayAllRoads(std::vector<Road *> v, int close, cv::Mat imageToWriteOn);

    //	Display every buildings
    void DisplayAllBuildings(std::vector<Building *> v, int close, cv::Mat imageToWriteOn);

    void DisplayAllUserNodes(std::vector<Node *> v, int close, cv::Mat imageToWriteOn);

    void DisplayPath(int close, cv::Mat imageToWriteOn);

    void WhichRoad(double lon, double lat);

    // Set the alpha factor that convert longitude/latitude in pixels
    static void SetAlpha();

    static void SetBeta();

    float ComputeCoefA(Node begin, Node end);

    void BuildAllRoads(int id_road, std::vector<Node> VEC);

    // Returns the Node with the specified ID
    Node * GetNodeById(long double id);

    Road * GetRoadById(double id);

    vector<tuple<string, double>> GetTupleOfDestinations();

    Map(rapidxml::file<> xmlFile);

    double WhichRoadWithLatLon();

    int CreateAll(int close, cv::Mat imageToWriteOn);

    int DisplayImage(int close, int waitKey_Local);

    int SetPosition(double lon, double lat);

    int DisplayMyPosition();

    int DisplayCloseMyPosition();

    int DisplayCloseToLocation(cv::Mat imageToWriteOn);

    float CapAlgorithm();

    void SetDestination(double lon, double lat);

    void SetDestination(double id);

    void SetPath(std::vector<double> path);

};



#endif
