#ifndef __struct_H__
#define __struct_H__
#include <cstring>
#include <string>
#include <map>
#include <tr1/unordered_map>
#include <queue>
#include <vector>
#include <set>
#include <algorithm>
#include <time.h>
#include <math.h>
#include <bitset>
#include <iostream>
using namespace std;

//#define OBJGEN 1
//bool RSU_SWITCH = false;
const int PUSH_NUM = 200;
const int VE_NUM = 10000;
const int QVE_NUM = 100;//querying vehicle num
//const int NODE_NUM = 24890;
//const int EDGE_NUM = 23961;
const int EMPTY_EDGE = 3000;
const int NODE_NUM = 2372;
const int EDGE_NUM = 2181;
const int STATISTIC = 4362;
const int GRID_H = 32;
const int GRID_W = 32;
const int LAT_NUM = 14;
const int LNG_NUM = 14;
const int RSU_NUM = LAT_NUM * LNG_NUM;
const bool debug = true;
const int SIMULATION_time=6000;
typedef long long LL;
const int INT = sizeof(int);
const int TRANSBIT = 3333333;//3Mbps
const int RSUBIT = 3333333;//3Mbps
const int EID = 32;
const int SEMANIC = 128;//a 64 b 64
const int QINFO = 449; // bits vid 32 + k 32 + msg_type 1 + query id 32 + trajectory(eid 32 + lat 64 + lng 64) + endtime 128 + range 64
const int HEADER = 32;//bits
const int OTHER = 352;//bits (telephone 32bit other 320bit)
const int OBJSIZEBIT = 608;//bits (nid 32 + eid 32 + lat 64 + lng 64 + other 352 + ttl 64)
const int RSUOBJBIT = 736;//bits (nid 32 + eid 32 + lat 64 +lng 64 + a 64 + b 64 + other 352 + 64)
const double RoadDistCal = 4000;//distance limit network distance calculation to avoid n^3
const int BITNUM = 1560;//if consider objs, it is casize * 6; here assume 200 edges, i.e., (200+ca)*6
const int REQ = 352;//bits(eid 32+ lat64 + lng64)*2+speed 32
const int REP = 128;//connect time 64 + Header 32 + vid32
const int RSU_MSG = RSUOBJBIT * PUSH_NUM + HEADER;
const double ALPHA = 0.15;
const double BETA = 0.75;
const int EGSIZE = 200;
const int DBDIST = 200;
const double NORMALIZE_MAX = 40;
const double NORMALIZE_MIN = 1;
const double AVGSIZE_MAX = 100;
const double AVGSIZE_MIN = 1;
const double QREL_MAX = 10;
const double QREL_MIN = 0;

//bool testExchange = false;
//200m
//0.125*communication range / distlat
//const double delat_lng = 0.002356027;
//const double delat_lat = 0.001798134;
//100m
//const double delat_lng = 0.00107954;
//const double delat_lat = 0.000899065;
//const double grid_lat = 0.001953125;//total lat divide grid number
//const double grid_lng = 0.00234375;

const double delat_lat = 100;
const double delat_lng = 100;

const double grid_lat = 114.702;//total lat divide grid number
const double grid_lng = 114.636;

const double ERROR_TOLERANCE_FOR_REAL_ARITHMETIC = 0.0000001;
	//error tolerance for real number arithmetic for the equality comparison of two real numbers: 10^-7

/* Macro functions */
#define MAX(a,b) ((a>=b) ? a : b)
#define MIN(a,b) ((a<=b) ? a : b)

//const int TTL[10] = { 2, 3, 4, 5, 10, 20, 40, 60, 80, 100};
class node;
typedef tr1::unordered_map <LL, double> hash_map;
typedef tr1::unordered_map <int, double> traffic_map;
typedef tr1::unordered_map <int, node> obj_map;
//const int MASK6BIT = 1 << 5;
class loc;
class line;
class circle;
class MathFun{
public:
	static const double eps;
	static int D(double v);
	static double Det(loc a, loc b, loc c);
	static bool oppoSide(loc u1, loc u2, loc v1, loc v2);
	static bool isInsideRect(loc x, loc* rect);
	static bool isCross(loc u1, loc u2, loc v1, loc v2);
	static bool isSegmentIntersectWithRect(line l, loc* rect);
	static bool isSegmentInterOrInsideWithRect(line l, loc* rect);
	static double innerProduct(loc a, loc b, loc c, loc d);
	static double dis(loc a, loc b);
	static loc lineCrossLine(const loc& a, const loc& b, const loc& c, const loc& d);
	static double point_seg_dist(loc p, loc s0, loc s1);
	static void lineCrossCircle(const circle& b, const loc& l1, const loc& l2, loc& p1, loc& p2);
};

class node{
public:
	int nid;
	int eid;
    double lat;
    double lng;
	bool is_object;
	int number_of_interarrivals;
	double sum_of_interarrival_time;
	bool interarrival;
	double time_to_live;
    
	node();
	node(int nid, int eid, double lat, double lng, bool is_object);
	node(int nid, int eid, double lat, double lng, double time_to_live);
	loc getLoc();
	bool isObject();
    bool isObjectTimeExpired(){
        return time_to_live <= 0;
    }
	//bool operator < (const node& b)const{
	//	return time_to_live > b.time_to_live;
	//};
};

class rsuSort{
public:
	bool operator()(const pair<double, int>& a, const pair<double, int>& b){
		return a.first < b.first;
	}
};

class candidateSort{
public:
	bool operator()(const pair<double, node>& a, const pair<double, node>& b){
		return a.first < b.first;
	}
};

class peerSort{
public:
	bool operator()(const pair<double, int>& a, const pair<double, int>& b){
		return a.first > b.first;
	}
};
class edge{
public:
    int eid;
    int startnid;
    int endnid;
    int Uturn;//0: no Uturn 1: Uturn
    int ways;//1: one way 2: two way
    double length;
	//int number_of_interarrivals_sid;
	//double sum_of_interarrival_time_sid;
	//bool interarrival_sid;
	//int number_of_interarrivals_eid;
	//double sum_of_interarrival_time_eid;
	//bool interarrival_eid;
	edge();
	edge(int _eid, int _startnid, int _endnid, int _Uturn, int _ways, double _length);
};
/************************loc*****************/
class loc{
public:
	double lat;
	double lng;
	loc();
	loc(double _lat, double _lng);
	bool operator == (const loc& b)const;
};
class line{
public:
	loc u;
	loc v;
	line();
	line(loc _u, loc _v);
};
class circle{
public:
	loc c;
	double r;
	circle();
	circle(loc _c, double _r);
};
/**********************trajectory*****************/
class trajectory{
public:
    int eid;
    loc location;
	trajectory();
	trajectory(int _eid, loc _location);
	trajectory(int _eid, double lat, double lng);
	bool operator == (const trajectory& b)const;
};
/************************range query msg*************/
enum msg_type{ KNN_QUERY=0, RANGE_QUERY=1};

//class objectCmp{
//public:
//	trajectory query_tra;
//	bool operator()(const node &x, const node &y);
//	objectCmp();
//	objectCmp(trajectory _query_tra);
//};
class msg_statistic;
//**********************cache********************//
class cache{
public:
	obj_map objects;
	map<int, vector<pair<double, double> > > roadSegments;
	trajectory last_query_tra;
	int size_in_range;
	map<int, int> edge_signature;
	//int RDsize;//road segment size by byte

	vector<int> getFristKobjects(trajectory query_trajectory, int kNum, double range);
	void updateRoadSegment(int eid, pair<double, double> segment);
	void updateRoadSegments(int eid, vector<pair<double, double> > segments);
	int getCacheSizeWithInRange(trajectory query_trajectory, double range);
	double getRangeOfRangeQuery(trajectory query_trajectory, int kNum, double r);
	bool checkDistOfEdge(int edgeid, trajectory query_trajectory, double range, node* nodemap, edge* edgemap);
	map<int, vector<pair<double, double> > > getRoadSementsCache(trajectory query_trajectory, double range, node* nodemap, edge* edgemap);
	bool isIdExisted( int id );
	void insertNewNode( node objectnode );
	void timeElapseForObjects(msg_statistic& caSta, node* nodemap, edge* edgemap, int s_time, trajectory traj);
	void updateEdgesig(int eid);
	void reSet();
};

class neigh{
public:
	int id;
	double dist;
	neigh (int _id, double _dist){
		id = _id;
		dist = _dist;
	};
	neigh (){};
	bool operator < (const neigh &t) const{ 
		return (dist > t.dist);
	}
};

class ttledge{
public:
	int snid;
	int eid;
	double length;
	double ttl;
	double tEdge;
	int num;
	ttledge (int _snid, int _eid, double _length, double _ttl, double _tEdge);
	ttledge ();
	bool operator< (const ttledge &t) const{
		return (ttl > t.ttl);
	}
};
//***************************msg******************//
class query_msg{
public:
	//static const int cache_range = 1000; //0805
	double cache_range;
	int k_q;//only for knn
    msg_type query_type;
    int query_id;
	int receive_id;
    trajectory query_trajectory;
    double endtime;
    double range;
    map<int, node> objectlist;
	map<int, vector<pair<double, double> > > roadSegments;
	bool broadcast;
	//int RD_size;
	//int ring;

	query_msg();
	query_msg(int vid, int timestamp, int kNum, double _range, map<int, vector<pair<double, double> > > _roadSegments, double _endtime, trajectory cur_trajectory, msg_type msg_Type, double _cache_range);
	query_msg(int vid, int timestamp, int kNum, double _endtime, trajectory cur_trajectory, msg_type msg_Type);
	int getDestinationVehicleId();
	int getTimeStamp();
	trajectory getQueryTrajectory();
	bool isKnnQueryFinished();
	bool isRangeQueryFinished();
	bool isCoverageSatisfy();
	double CoverageRatio();
	loc getQueryLoction();
	int getObjectsNum();
	void insertNewObject(node newObject);
	bool isIdExisted(int id);
	void updateRoadSegments(int eid, vector<pair<double, double> > segments);
};

//***********************object*******************//
class object{
public:
	int communication_range;
	int oid;
	int eid;
	double dist;
	string description;
	string type;
	//vector<vehicle> v_neighbor;
	queue<query_msg> query_msg_queue;
	object();
	object(int _oid, int _eid, double _dist);
};

class Grid;
class Statistic;
class overhead;
class msg_statistic;
class Roadsideunit;
class Point;
class Cluster;
//**************************vehicle**************************//
class vehicle{
public:
    int vid;
	bool warmup;
	bool finish;
    trajectory trajectorys;
    double vehicle_arrival_rate_per_second;
    double speed;
    cache local_cache;
    //static const double communication_range;
	static const int chipNum = 1; //6.3
	static const double timechip;
    int query_state;
	set<int> csId;
	map<int, int> search;
	int cs_time;
    map<int, query_msg> query_msg_queue;
	int label;
	trajectory query_trajectory;
	trajectory last_trajectory;
	vector<pair<int, int> > last_grid;
	int predict;
	bool exBegin;
	//static const int cache_range = 1000; //6.4
	//set<int> exObject;
	//map<int, int> exRoad;
	set<int> msgReceive;
	node* nodemap;
	vehicle * vehiclemap;
	edge* edgemap;
	loc * rsuLoc;
	map<int, vector<pair<int, int> > >* nodeToEdgemap;
	map<int, vector<pair<int, int> > >* edgeOfGrids;
	map<int, pair<double, double> > * nodeToDistmap;
	set<int> * edgeInGridmap;
	set<int>* emptyEdgemap;
	map<int, object>* objectmap;
	
	Grid* grid;
	int encrypGrid();
	double selectPeers(double _vx, double _vy, loc _pos);
	void updateRoadsFromRoads(vector<pair<double, double> > segments, vector<pair<double, double> >& roads);
	void makeCacheSig(bitset<BITNUM>& m, int id);
	int prepareObjFromSig(bitset<BITNUM> m, int vehicleId, trajectory current, obj_map& objects, map<int, vector<pair<double, double> > >& roadSegments, double tConnect);
	void findNeighors(set<int>& ids);
	void checkCacheForOverdue(msg_statistic& msgsta);
	double GetEdgeCovered(double range, set<int>& coverEdgeId, map<int, double>& unCovered);
	double Compute_Edge_Delay_test( int eid, int nid, Grid& mainGrid, double l, int cur_time, int oid);
	double compute_ttl_test ( Grid& mainGrid, double range, int cur_time );
	double Compute_ACL_length( int eid, int nid, Grid& mainGrid, double l, int cur_time);
	int getVehicleId();
	loc getCurrentPosition();
	trajectory getCurrentTrajectory();
	//bool findNextRoadVertex(int startnid, int starteid, int endeid, vector<pair<int, int> >& road_vertexs, int dp, int maxDp);
	bool findNextRoadVertex( int startnid, int starteid, int endeid, vector<pair<int, int> >& road_vertexs, vector<int>& roads, int dp, int maxDp );
	//vector<trajectory> findRoad(trajectory start, trajectory end);
	vector<trajectory> findRoad( trajectory start, trajectory end, int time );
	trajectory getQueryTrajaectory();
	loc getQueryPosition();
	void setQueryPosition();
	void updateLastTrajectory(pair<int, int> pos);
	vehicle();
	vehicle(int _vid);
	void timeElapseForQuery(int cur_time, Statistic& statistic, edge* edgemap, node* nodemap, Grid& mainGrid);
	void handleQuery(int cur_time, Statistic& statistic, edge* edgemap, node* nodemap, Grid& mainGrid);
	//void handleMove(Statistic& statistic, int time);
	void handleMove( int s_time, Roadsideunit * rsumap, Grid& mainGrid, Statistic& statistic, overhead& rsu_statistic);
	void addNewObjectFromCell(Statistic& statistic, int s_time);
	void addNewRoadSegmentFromCell( int time );
	void correctEdges();
	bool broadCastQuery(query_msg msg , Statistic& statistic, set<int>& ids);
	void receiveQuery( query_msg msg, Statistic& statistic );
	void updateLocalCacheFromMsg(query_msg msg);
	void updateMsgFromLocalCache(query_msg& msg);
	int compareid ( vector<int>& ids, set<int>& std_ids );
	void reSet();
	//void updateVehiclesfromRoadsideunit(Roadsideunit * rsumap, Grid& mainGrid);
	void updateVehiclesfromRoadsideunit(Roadsideunit * rsumap, Grid& mainGrid, overhead& rsu_statistic);
	void exchangeInfo(overhead& ex_statistic, int s_time);
	void updateObjFromVeh(obj_map objects, map<int, vector<pair<double, double> > > roadSegments);
	void DBSCAN(int eps, int MinPts, vector<Cluster*>& clusters);
	vector<Point*>* regionQuery(vector<Point*>& D, Point* P, int eps);
	void expandCluster(vector<Point*>& D, Point* P, int C, vector<Point*> *NeighborPts, int eps, int MinPts, vector<Cluster*>& clusters);
	double cacheShareCost(int eid, trajectory current);
	double cacheShareDistCost(int eid, trajectory current);
	void handleCacheShare(int s_time, overhead& ex_statistic);
	void cleanCluster(vector<Cluster*>& clusters);
};
//**************************grid**************************//
class GridCell{
public:
	set<int> Node;
	set<int> Veh;
	vector<int> Rsu;
	set<int> Edge;
	GridCell();
	void reSet();
};

class Grid{
public:
	static int TYPE;
    static bool testExchange;
    static bool RSU_SWITCH;
	static int TIME;
	static int KNUM;
	static int SPEED;
	static int KnnTime;
	static double COMRANGE;
	static loc low;
	static loc high;
	static loc trans_low;
	static loc trans_high;
	static int FILLRSU;
	static int BRORSU;
	static int BRO_PERIOD;
	static int FRESH;
	static int CASIZE;
	static int RANGE;
	static double COVERAGE;
	static double PERIOD;
	static int TOP;
	static char* NAME;
	static int RSU_N;

	GridCell gridCell[GRID_H][GRID_W];
	static vector<pair<double, double> > getCoveredRoadSegment(trajectory a, int eid, double range);
	static node *nodemap;
	static edge *edgemap;
	static map <int, vector<pair<int, int> > > *nodeToEdgemap;
	static map <int, vector<pair<int, int> > > *edgeOfGrids;
	static map <int, pair<double, double> > * nodeToDistmap;
	static set <int> * emptyEdgemap;
	static vehicle *vehiclemap;
	static map <int, object> *objectmap;
	static loc * rsuLoc;
	//static map<LL, double> roadNetworkDist;
	static hash_map roadNetworkDist;
	static traffic_map traffic;
	static map<int, vector<int> > objectsOnEdgeMap;
	class cDist{
	public:
		int id;
		int eid;
		double dist;
		cDist(){};
		cDist(int _id, int _eid, double _dist){
			id = _id;
			eid = _eid;
			dist = _dist;
		};
		bool operator < (const cDist& b)const{
			return dist > b.dist;
		};
	};

	static void initObjectOnEdge();
	static double caculateRoadNetworkDist(trajectory& a, int b );
	static double caculateRoadNetworkDist(trajectory a, trajectory b);
	static double getRoadNetworkDist(int idx, int idy);
	static void addRoadNetworkDist(int idx, int idy, double dist);
	static void initRoadNetworkDist(map <int, vector<pair<int, int> > >& nodeToEdgemap, node* nodemap);
	static pair<int, int> getLocFromGeo(double x, double y);
	static pair<int, int> getLocFromGeo(loc pos);
	static double calculateCircleDist(double startLat, double startLng, double endLat, double endLng);
	static double calculateCircleDist(loc u, loc v);
	static double calculateDist(loc u, loc v);
	static double calculateDist(double startLat, double startLng, double endLat, double endLng);
	static bool is_Inrange(int x, int y);
	void add_vehicle(loc lo, int vid, int time);
	void remove_vehicle(loc lo, int vid);
	set<int> getAllVehicleInCell(loc lo);
	void getAllVehicleInCell( int x, int y, set<int>*& ids );
	set<int> getAllVehicleInCell(pair<int, int> pos);
	void getAllObjectsInCell(int x, int y, set<int>*& ids );
	set<int> getAllObjectsInCell(int x, int y);
	set<int> getAllObjectsInCell(loc lo);
	set<int> getAllObjectsInCell(pair<int, int> pos);
	set<int> knnQueryInGrid(trajectory tra, int kNum, double r);
	static bool testSelectPeers(double _vx, double _vy, loc _pos, double& t);
	static double shortestPathDist(int idx, int idy, map<int, vector<pair<int, int> > >& nodeToEdgemap, node* nodemap);
	void reSet();
};
//**************************statistic**************************//
class Statistic{
public:
	LL number;
	LL msg_size;
	double power;
	int carry_time;
	double transtime;
	double accuracy;
	int precision;
	int completeknn;
	double ttl;
	double dist;
	double range;
	double coverage;
	Statistic();
	void reSet();
};
class overhead{
public:
	LL number;
	LL msg_size;
	double power;
	int carry_time;
	double transtime;
	overhead();
	void reSet();
};
class msg_statistic{
public:
	int number;
	int obj_size;
	int road_size;//vector num
	int road_num; //map num
	msg_statistic();
	void reSet();
};
//**************************statistic**************************//
class RsuObj{
public:
	int nid;
	int eid;
	double lat;
	double lng;
	loc roadSegments;
	double time_to_live;
	RsuObj(int _nid, int _eid, double _lat, double _lng, loc _roadSegments, double _time);
	RsuObj(){};
};
class Roadsideunit{
public:
	int mark;
	int num;
	int lable;
	vector<RsuObj> objects;
	Roadsideunit();
	void PushObjectstoVehicle(vector<RsuObj>& RsuObject);
	void Rsubroadcast();
	int predict;
	bool BroBegin;
	int count;
};
//DBscan
class Point {
public:
	int id;
    double x, y;
	int label; // label = 0 means it is a noise
    bool visited;
    Point() {}
    Point(int _id, double _x, double _y, int _l, bool _v): id(_id), x(_x), y(_y), label(_l), visited(_v) {}
    void print() {
        cout << "(x, y, label) : " << x << ", " << y << ", " << label << endl;
    }
    ~Point() {}
};
class Cluster {
public:
    vector<Point*> points;
    int label;
    Cluster() {}
    Cluster(int _l) :label(_l) {}
    void push(Point* P) {
        P -> label = label;
        points.push_back(P);
    }
    void print(){
        cout << "Cluster label : " << label << endl;
        for (int i = 0; i < points.size(); i ++) {
            points[i] -> print();
        }
    }
    ~Cluster() {
    	for (int i = 0; i < points.size(); i ++) {
    		delete points[i];
    	}
    }
};
#endif
