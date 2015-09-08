#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <iomanip>
#include <vector>
#include <map>
#include <tr1/unordered_map>
#include "loadOperation.h"
#include "vehicleOperation.h"
#include "struct.h"
#include <sys/time.h>
#include <sstream>
using namespace std;

Grid mainGrid;
node nodemap[NODE_NUM];
edge edgemap[EDGE_NUM];
set <int> edgeInGridmap;//the edges that are in one grid
map <int, pair<double, double> > nodeToDistmap;
map <int, vector<pair<int, int> > > nodeToEdgemap;
set <int> emptyEdgemap;
map <int, vector<pair<int, int> > > edgeOfGrids;
vehicle vehiclemap[VE_NUM];
//map <int, Roadsideunit> rsumap;
Roadsideunit rsumap[RSU_NUM];
int rsuRest[RSU_NUM];
loc rsuLoc[RSU_NUM];
map <int, vector<node> > edgeToObjmap;
bool isLoadFile = true;
Statistic statistic;
overhead ex_statistic;
overhead rsu_statistic;
LL query_num = 0;

void load_files();

void writeDistToFile();

void writeDistToFile(){
	isLoadFile = false;
	load_files();
	ofstream outfile;
	outfile.open("roadNetwok.dist");
	for( tr1::unordered_map<LL, double>::iterator iter = mainGrid.roadNetworkDist.begin(); iter != mainGrid.roadNetworkDist.end(); iter ++){
		outfile << iter->first << " " << iter->second << endl;
	}
	outfile.close();
	isLoadFile = true;
}

void loadDistFromFile();

void loadDistFromFile(){
	cout << "begin loading network..." << endl;
	ifstream infile;
	infile.open("roadNetwok.dist");
	if( !infile.is_open()){
		cout << "Error opening file." << endl;
	}
	LL id; double dist;
	int count = 1;
	while(infile >> id >> dist){
		count ++;
		if(count % 100000 == 0){
			cout << ".";
		}
		if(count % 2000000 == 0){
			cout << endl;
		}
		mainGrid.roadNetworkDist[id] = dist;
	}
	/*for( tr1::unordered_map<LL, double>::iterator iter = mainGrid.roadNetworkDist.begin(); iter != mainGrid.roadNetworkDist.end(); iter ++){
		cout << iter->first << " " << iter->second << endl;
	}*/
	infile.close();
	cout << "end loading network..." << endl;
}
void loadTrafficFromFile();

void loadTrafficFromFile(){
	FILE * fFile;
	fFile = fopen("traffic.statistics", "r");
	if(fFile != NULL)
	{
		while (!feof(fFile))
		{
			for (int i = 0; i < STATISTIC; i++){
				int id; double lambda;
				fscanf(fFile, "%d %lf", &id, &lambda);
				mainGrid.traffic[id] = lambda;
			}
		}
	}
	fclose(fFile);
	if(debug)printf("load traffic statistics fin!\n");
	/*for( tr1::unordered_map<int, double>::iterator iter = mainGrid.traffic.begin(); iter != mainGrid.traffic.end(); iter ++){
		cout << iter->first << " " << iter->second << endl;
	}*/
}
void do_test();
void initInfo();
void checkInfo();

void do_experiment();

void do_experiment(){
	//timeval eucStart, eucEnd;
	srand(0);
	bool warm_up = true;
	bool traffic = true;
	ifstream infile;
	string line;
	double lat, lng;
	int traid, timestep;
	int s_time;
	FILE * fFile1;
	int index =0;
	//int tclock = Grid::TIME + 500;
	fFile1 = fopen ("m30traj1w.node", "r");
	if(fFile1 != NULL)
	{
		while(!feof(fFile1))
		{
			index ++;
            if(index % 200 == 0)cerr << ".";
			for (int i=0; i < VE_NUM; i++)
			{
				fscanf(fFile1, "%d %d %lf %lf", &timestep, &traid, &lat, &lng);
				loc p = coordTrans(lat, lng);
				vehiclemap[i].trajectorys = trajectory(traid, p.lat, p.lng);
			}
			s_time = timestep;
			if ( s_time == 0 )
			{
				initInfo();
				cout << "end to load files..." << endl;
				checkInfo();
				do_test();
			}
			//if(warm_up == false)cout << "SIMULATION :\t" << s_time << " begins" << endl;//by7.2
			if(s_time > Grid::TIME)cout << "SIMULATION :\t" << s_time << " begins" << endl;//by7.2
			/************************************************************************/
			/* for traffic statistic                                                           */
			/************************************************************************/

			if(traffic == true && s_time > Grid::TIME){
				traffic = false;
				//printf("***%d %d %d\n", s_time, (int)vehiclemap[11].local_cache.objects.size(), (int)vehiclemap[11].query_msg_queue.size());
				/*for(int vehicleId = 0; vehicleId < VE_NUM; vehicleId ++){
					vehiclemap[vehicleId].local_cache.reSet();
				}*/
				initial_w_vehicle(vehiclemap, edgemap, nodemap, s_time, mainGrid, statistic);
				//gettimeofday(&eucStart, 0);
			}
			/************************************************************************/
			/* for warm_up                                                                     */
			/************************************************************************/
			if(traffic == false){
				if (warm_up == true){
					int fin = 0;
					if (s_time == Grid::TIME + 4){
						double cacount = 0;
						warm_up = false;
						statistic.reSet();
						if(Grid::testExchange)ex_statistic.reSet();
						if(Grid::RSU_SWITCH)rsu_statistic.reSet();
						query_num = 0;
						for(int vehicleId = 0; vehicleId < VE_NUM; vehicleId ++){
							vehiclemap[vehicleId].query_msg_queue.clear();
							vehiclemap[vehicleId].query_state = 0;
							vehiclemap[vehicleId].msgReceive.clear();
							cacount += 1.0 * vehiclemap[vehicleId].local_cache.objects.size() / Grid::CASIZE;
							if(vehiclemap[vehicleId].finish == true){
								fin ++;
								vehiclemap[vehicleId].finish = false;
							}
							//vehiclemap[vehicleId].predict = s_time;
						}
						initial_q_vehicle(vehiclemap, edgemap, nodemap, s_time, mainGrid, statistic);
						printf("Done warming up! Cache:%lf fin:%d\n", cacount / VE_NUM, fin);
					}
				}
			}
			/************************************************************************/
			/*Rsubroadcast                                                                     */
			/************************************************************************/
			if(Grid::RSU_SWITCH){
				for (int i = 0; i < RSU_NUM; i ++){
					if(s_time == rsumap[i].predict){
						rsumap[i].BroBegin = true;
						rsumap[i].count = ceil(1.0 * rsumap[i].objects.size() / PUSH_NUM);
					}
					if(rsumap[i].BroBegin == true){
						if (!rsumap[i].objects.empty()){
							rsumap[i].Rsubroadcast();
							if(rsumap[i].count == 1){
								rsu_statistic.msg_size += rsuRest[i];
							}
							else
								rsu_statistic.msg_size += RSU_MSG;
							rsu_statistic.number ++;
							rsumap[i].count --;
							/*if(rsumap[i].count == 0){
								rsumap[i].BroBegin = false;
								rsumap[i].predict = s_time + Grid::BRO_PERIOD * 10;
							}*/
							//rsu_statistic.power += 0.5 * temp / 8 + 56;
							//rsu_statistic.transtime += 1.0 * temp/ RSUBIT;
						}
					}	
				}
			}
			/************************************************************************/
			/*move vehicle to next position                                                                      */
			/************************************************************************/
			//cerr << "begin handle move" << endl;
			for(int vehicleId = 0; vehicleId < VE_NUM; vehicleId ++){
				vehiclemap[vehicleId].handleMove(s_time, rsumap, mainGrid, statistic, rsu_statistic);
				if(Grid::testExchange && s_time > Grid::TIME)vehiclemap[vehicleId].handleCacheShare(s_time, ex_statistic);
				/*if(s_time == Grid::TIME){
					printf("%d\n", vehicleId);
					for (int i = 0; i < (int)vehiclemap[vehicleId].last_grid.size(); i ++)
					{
						printf("(%d, %d) ", vehiclemap[vehicleId].last_grid[i].first, vehiclemap[vehicleId].last_grid[i].second);
					}
					printf("\n");
				}*/
			}
			if(Grid::RSU_SWITCH){
				for (int i = 0; i < RSU_NUM; i ++){
					if(rsumap[i].BroBegin == true){
						if(rsumap[i].count == 0){
							rsumap[i].BroBegin = false;
							rsumap[i].predict = s_time + Grid::BRO_PERIOD * 10;
						}
					}
				}	
			}
			//cerr << "end handle move" << endl;
			/************************************************************************/
			/* Do all kinds of queries                                                                      */
			/************************************************************************/
			//cerr << "begin handle query" << endl;
			for(int chipnum = 0; chipnum < vehicle::chipNum; chipnum ++){ 
				//int tmax = 0;//the biggest msg size in one chipnum
				//cerr << "."; 
				//cout << "Time elapse for query..." << endl; 
				for(int vehicleId = 0; vehicleId < VE_NUM; vehicleId ++){
					vehiclemap[vehicleId].timeElapseForQuery(s_time, statistic, edgemap, nodemap, mainGrid);
					//vehiclemap[vehicleId].local_cache.timeElapseForObjects();
				}
				//cout << "handle query..." << endl;
				for(int vehicleId = 0; vehicleId < VE_NUM; vehicleId ++){
					vehiclemap[vehicleId].handleQuery(s_time, statistic, edgemap, nodemap, mainGrid);
				}
				//cout << "Chip:\t" << chipnum  << " ends" << endl;
				//statistic.vehicle_transtime += 1.0 * tmax / TRANSBIT;
			}
			//cerr << endl;
			//cerr << "end handle move" << endl;
			//cerr << "SIMULATION :\t" << s_time << " ends" << endl;
			
			msg_statistic msgSta;
			msg_statistic caSta;
			int finveh = 0;
			double caCount = 0;
			for(int vehicleId = 0; vehicleId < VE_NUM; vehicleId ++){
				//printf("vid %d ", vehicleId);
				vehiclemap[vehicleId].local_cache.timeElapseForObjects(caSta, nodemap, edgemap, s_time, vehiclemap[vehicleId].getCurrentTrajectory());
				vehiclemap[vehicleId].checkCacheForOverdue(msgSta);
				loc current = vehiclemap[vehicleId].getCurrentPosition();
				vehiclemap[vehicleId].grid->remove_vehicle( current, vehiclemap[vehicleId].getVehicleId());
				vehiclemap[vehicleId].updateLastTrajectory( Grid::getLocFromGeo(current) );
				if(vehiclemap[vehicleId].finish == true)finveh ++;
				caCount += 1.0 * vehiclemap[vehicleId].local_cache.objects.size() / Grid::CASIZE;
				//if(vehicleId == 11)printf("$ %d %d\n", vehiclemap[vehicleId].predict);
			}
			//if(s_time == 1000 || s_time == 1001)printf("**%d %d %d\n", s_time, (int)vehiclemap[11].local_cache.objects.size(), (int)vehiclemap[11].query_msg_queue.size());
			printf("* %d %d %d %d\n", msgSta.number, msgSta.obj_size, msgSta.road_size, msgSta.road_num);
			printf("@ %d %d %d\n", caSta.obj_size, caSta.road_size, caSta.road_num);
			
			printf("%lld\t%lld\t%d\t%lf\n", statistic.number, statistic.number - query_num, finveh, caCount);
			//printf("ttl\t%lf\n", 0.1 * statistic.ttl / QVE_NUM);
			//printf("dist\t%lf\n", statistic.dist / QVE_NUM);
			//printf("range\t%lf\n", statistic.range / QVE_NUM);
			query_num = statistic.number;

			//msgSta.reSet();
			if(finveh == QVE_NUM && warm_up == false) break;
			//if(index >= tclock)break;
		}
	}
	fclose(fFile1);
	int min = EMPTY_EDGE;
	int max = 0;
	long avg = 0;
	map<int, vector<int> >historgram;
	for(int vehicleId = 0; vehicleId < VE_NUM; vehicleId ++){
		//int temp = (int)vehiclemap[vehicleId].local_cache.roadSegments.size();
		int temp = (int)vehiclemap[vehicleId].cs_time;
		if(temp < 10) historgram[10].push_back(temp);
		else
			if(temp < 20) historgram[20].push_back(temp);
			else
				if(temp < 30) historgram[30].push_back(temp);
				else
					historgram[40].push_back(temp);
		if(temp < min)min = temp;
		if(temp > max)max = temp;
		avg += temp;
	}
	printf("XD%d\t%d\t%lf\n", min, max, 1.0 * avg / VE_NUM);
	for(map<int, vector<int> >::iterator iter = historgram.begin(); iter != historgram.end(); iter ++)
	{
		int cnt = (int)iter->second.size();
		printf("%d within %d ", cnt, iter->first);
		int avg = 0;
		for(vector<int>::iterator it = iter->second.begin(); it != iter->second.end(); it ++){
			avg += *it;
		}
		printf("avg %lf\n", avg / cnt * 1.0);
	}
	//gettimeofday(&eucEnd, 0);
	FILE * f;
	int para = 0;
	switch(Grid::TYPE){
	case 1:
		{
			para = Grid::KNUM;
			break;
		}
	case 2:
		{
			para = Grid::FRESH;
			break;
		}
	case 3:
		{
			para = Grid::RANGE;
			break;
		}
	case 4:
		{
			para = Grid::PERIOD;
			break;
		}
	case 5:
		{
			para = LAT_NUM * LNG_NUM;
			break;
		}
	case 6:
		{
			para = Grid::FILLRSU;
			break;
		}
	case 7:
		{
			para = ceil(Grid::COVERAGE * 10);
			break;
		}
	case 8:
		{
			para = Grid::TOP;
			break;
		}
	}
	//char name[3] = {Grid::TYPE + '0', Grid::testExchange + '0', Grid::RSU_SWITCH + '0'};
	//char dataDir[256] = "logs/OriginalData/";
	char dataDir[256] = "./test/rawData/";
	strcat(dataDir, Grid::NAME);
	
	f = fopen(dataDir, "w");
	if(Grid::TYPE != 7){
		double test = Grid::testExchange + Grid::COVERAGE;
		if(test == 0.5 || test == 1.5)
			fprintf(f, "%d %d %d %d\n", Grid::TYPE, para, Grid::testExchange, Grid::RSU_SWITCH);
			//fprintf(f, "%d %d %d %d\n", Grid::TYPE, para, Grid::RSU_SWITCH, Grid::testExchange);
		else
			fprintf(f, "%d %d %d %d\n", Grid::TYPE, para, Grid::testExchange + 1, Grid::RSU_SWITCH);
			//fprintf(f, "%d %d %d %d\n", Grid::TYPE, para, Grid::RSU_SWITCH + 1, Grid::testExchange);
	}
	else
		fprintf(f, "%d %d %d %d\n", Grid::TYPE, para, Grid::testExchange, Grid::RSU_SWITCH);
		//fprintf(f, "%d %d %d %d\n", Grid::TYPE, para, Grid::RSU_SWITCH, Grid::testExchange);
	//fprintf(f, "NN\t%d\n", Grid::KNUM);
	double total_msg_size = 0;
	int total_msg_num = 0;
	printf("%d %d %d\n", Grid::TYPE, Grid::testExchange, Grid::RSU_SWITCH);
	printf("NN\t%d\n", Grid::KNUM);
	printf("query_msg_size\t%lf\n", 1.0 * statistic.msg_size / QVE_NUM / 8 / 1024 / 1024);//MB
	printf("msg_number\t%lf\n", 1.0 * statistic.number / QVE_NUM / 1000);//by 1,000
	total_msg_size += statistic.msg_size;
	total_msg_num += statistic.number;
	//printf("power w.s\t%lf\n", statistic.power / QVE_NUM / 1000000);
	//printf("transmission time\t%lf\n", statistic.transtime / QVE_NUM);
	if(Grid::testExchange){
		printf("exchange_msg_size\t%lf\n", 1.0 * ex_statistic.msg_size / QVE_NUM / 8 /1024 / 1024);
		printf("exchange_msg_number\t%lf\n", 1.0 * ex_statistic.number / QVE_NUM / 1000);
		total_msg_size += ex_statistic.msg_size;
		total_msg_num += ex_statistic.number;
		//printf("exchange power w.s\t%lf\n", ex_statistic.power / QVE_NUM / 1000000);
		//printf("exchange transtime\t%lf\n", ex_statistic.transtime / QVE_NUM);
	}
	if(Grid::RSU_SWITCH){
		printf("rsu_msg_size\t%lf\n", 1.0 * rsu_statistic.msg_size / QVE_NUM / 8/1024 / 1024);
		printf("rsu_msg_number\t%lf\n", 1.0 * rsu_statistic.number / QVE_NUM / 1000);
		total_msg_size += rsu_statistic.msg_size;
		total_msg_num += rsu_statistic.number;
		//printf("rsu power w.s\t%lf\n", rsu_statistic.power / QVE_NUM / 1000000);
		//printf("rsu transtime\t%lf\n", rsu_statistic.transtime / QVE_NUM);
	}
	fprintf(f, "query_msg_size\t%lf\n", 1.0 * total_msg_size / QVE_NUM / 8 / 1024 / 1024);//MB
	fprintf(f, "msg_number\t%lf\n", 1.0 * total_msg_num / QVE_NUM / 1000);//by 1,000
	fprintf(f, "carry_time\t%lf\n", 1.0 * statistic.carry_time / QVE_NUM / 10);//by second
	fprintf(f, "precision\t%lf\n", 1.0 * statistic.precision / QVE_NUM);//precision ratio
	fprintf(f, "accuracy\t%lf\n", statistic.accuracy / QVE_NUM);
	fprintf(f, "completeknn\t%d\n", statistic.completeknn);
	fprintf(f, "ttl\t%lf\n", 0.1 * statistic.ttl / QVE_NUM);
	fprintf(f, "dist\t%lf\n", statistic.dist / QVE_NUM);
	fprintf(f, "range\t%lf\n", statistic.range / QVE_NUM);
	fprintf(f, "coverage\t%lf\n", statistic.coverage / QVE_NUM);
	fclose(f);

	printf("carry_time\t%lf\n", 1.0 * statistic.carry_time / QVE_NUM / 10);//by second
	printf("precision\t%lf\n", 1.0 * statistic.precision / QVE_NUM);//precision ratio
	printf("accuracy\t%lf\n", statistic.accuracy / QVE_NUM);
	printf("completeknn\t%d\n", statistic.completeknn);
	printf("ttl\t%lf\n", 0.1 * statistic.ttl / QVE_NUM);
	printf("dist\t%lf\n", statistic.dist / QVE_NUM);
	printf("range\t%lf\n", statistic.range / QVE_NUM);
	printf("coverage\t%lf\n", statistic.coverage / QVE_NUM);
}

void initInfo(){
	for(int vehicleId = 0; vehicleId < VE_NUM; vehicleId ++){
		vehiclemap[vehicleId].nodemap = nodemap;
		vehiclemap[vehicleId].edgemap = edgemap;
		vehiclemap[vehicleId].nodeToEdgemap = &nodeToEdgemap;
		vehiclemap[vehicleId].nodeToDistmap = &nodeToDistmap;
		vehiclemap[vehicleId].edgeOfGrids = &edgeOfGrids;
		vehiclemap[vehicleId].vehiclemap = vehiclemap;
		vehiclemap[vehicleId].grid = &mainGrid;
		vehiclemap[vehicleId].emptyEdgemap = &emptyEdgemap;
		vehiclemap[vehicleId].warmup = true; 
		vehiclemap[vehicleId].finish = false;
		vehiclemap[vehicleId].edgeInGridmap = &edgeInGridmap;
		vehiclemap[vehicleId].rsuLoc = rsuLoc;
		vehiclemap[vehicleId].cs_time = 0;
	}
	Grid::nodemap = nodemap;
	Grid::edgemap = edgemap;
	Grid::nodeToEdgemap = &nodeToEdgemap;
	Grid::nodeToDistmap = &nodeToDistmap;
	Grid::edgeOfGrids = &edgeOfGrids;
	Grid::vehiclemap = vehiclemap;
	Grid::rsuLoc = rsuLoc;
	Grid::initObjectOnEdge();
	/*if(!isLoadFile)Grid::initRoadNetworkDist(nodeToEdgemap, nodemap);
	else{
		loadDistFromFile();
	}*/
	loadTrafficFromFile();
}

void do_test(){
	node n0 = nodemap[0];
	node n1 = nodemap[1];
	node n2 = nodemap[346];

	double dist1 = Grid::calculateDist(n0.getLoc(), n1.getLoc());
	double dist2 = Grid::calculateDist(n1.getLoc(), n2.getLoc());
	double dist3 = Grid::caculateRoadNetworkDist(trajectory(0, n0.getLoc()), trajectory(320, n2.getLoc()));
	double dist4 = Grid::getRoadNetworkDist(0, 346);
	double dist5 = Grid::shortestPathDist(0, 346, nodeToEdgemap, nodemap);
	double distlat = Grid::calculateCircleDist(Grid::low.lat, Grid::low.lng, Grid::high.lat, Grid::low.lng);
	double distlng = Grid::calculateCircleDist(Grid::low.lat, Grid::low.lng, Grid::low.lat, Grid::high.lng);
	//double lat_test = Grid::calculateDist(39.915, 116.395, 39.948, 116.395);
	//double lng_test = Grid::calculateDist(39.915, 116.395, 39.915, 116.438);
	cout << dist1 << " " << dist2 << endl;
	cout << dist3 << " " << dist4 << endl;
	cout << dist5 << endl;
	cout << distlat << ' ' << distlng << endl;
	//cout << lat_test << ' ' << lng_test << endl;
	cout << (Grid::trans_high.lat - Grid::trans_low.lat) * Grid::COMRANGE / distlat << ' ' << (Grid::trans_high.lng - Grid::trans_low.lng) * Grid::COMRANGE / distlng << endl;
	cout << (Grid::trans_high.lat - Grid::trans_low.lat) / GRID_H << ' ' << (Grid::trans_high.lng - Grid::trans_low.lng) / GRID_W << endl;
	double vx = (2 - 1) / 0.1;//0.1s time step
	double vy = (2 - 1) / 0.1;
	double t = 0;
	Grid::testSelectPeers(vx, vy, loc(2, 2), t);
    //cout << Grid::getLocFromGeo(Grid::low).first << ' ' << Grid::getLocFromGeo(Grid::low).second << endl;
    //cout << Grid::getLocFromGeo(Grid::high).first << ' ' << Grid::getLocFromGeo(Grid::high).second << endl;
  //  if(debug){
  //      int time_to_test = 10;
  //      while(time_to_test > 0){
  //          time_to_test --;
  //          int u = rand() % NODE_NUM, v = rand() % NODE_NUM;
  //          double dist_dij = Grid::shortestPathDist(u, v, nodeToEdgemap, nodemap);
  //          double dist_bfs = Grid::getRoadNetworkDist(u, v);
		//	//cout << "Network Distance between " << u << " and " << v <<  " is " << " vs " << dist_bfs << endl; 
  //          cout << "Network Distance between " << u << " and " << v <<  " is " << dist_dij << " vs " << dist_bfs << endl; 
  //      }
		//for(int i = 0; i < 10; i ++){
		//	double dist_dij = Grid::shortestPathDist(0, i, nodeToEdgemap, nodemap);
		//	double dist_bfs = Grid::getRoadNetworkDist(0, i);
		//	//cout << "Network Distance between " << u << " and " << v <<  " is " << " vs " << dist_bfs << endl; 
		//	cout << "Network Distance between " << 0 << " and " << i <<  " is " << dist_dij << " vs " << dist_bfs << endl; 
		//}
  //  }
	/*int num = 0;
	for (map<int, vector<int> >::iterator iter = Grid::objectsOnEdgeMap.begin(); iter != Grid::objectsOnEdgeMap.end(); iter ++)
	{
		num += (int)iter->second.size();
	}
	printf("objOnEdgeMap: %d %d\n", num, NODE_NUM);*/
}

void checkInfo(){
	int size_object = 0;
	for(int i = 0; i < GRID_W; i++){
		for(int j = 0; j < GRID_H; j++){
			size_object += mainGrid.getAllObjectsInCell(j, i).size();
		}
	}
	cout << "object size:\t" << size_object << endl;
	/*FILE * fFile;
	fFile = fopen("grids", "w");
	int labels[5] = {0, 0, 0, 0, 0};
	for (int i = 0; i < 64; i ++)
	{
	for (int j = 0; j < 64; j ++)
	{
	labels[mainGrid.gridCell[i][j].Rsu] ++;
	}
	}
	for(int i = 0; i < 5; i++){
	fprintf(fFile, "%d %d\n", i, labels[i]);
	}
	fclose(fFile);*/
}

void load_files(){
	for(int i = 0; i < VE_NUM; i++){
		vehiclemap[i] = vehicle(i);
	}
	cout << "begin to load files..." << endl;
	if(!isLoadFile)Grid::initRoadNetworkDist(nodeToEdgemap, nodemap);
	else{
		loadDistFromFile();
	}
	getnodes(nodemap, mainGrid, edgeToObjmap);
    getedges(nodemap, edgemap, nodeToEdgemap, edgeToObjmap, emptyEdgemap, mainGrid, edgeInGridmap);
	getrsu( rsumap, mainGrid, edgeToObjmap, emptyEdgemap, edgemap, nodemap, nodeToDistmap, rsuLoc);
	labelBro(mainGrid, Grid::BRORSU, LAT_NUM, LNG_NUM);
	for (int i = 0; i < RSU_NUM; i ++)
	{
		rsuRest[i] = RSUOBJBIT * ((int)rsumap[i].objects.size() % PUSH_NUM) + HEADER;
		rsumap[i].predict = Grid::TIME + 1;
		rsumap[i].BroBegin = false;
	}
	for(int i = 0; i < GRID_H; i ++){
		for(int j = 0; j < GRID_W; j ++)
		{
			if(mainGrid.gridCell[i][j].Edge.empty() == true)continue;
			for (set<int>::iterator iter = mainGrid.gridCell[i][j].Edge.begin(); iter != mainGrid.gridCell[i][j].Edge.end(); iter ++)
					edgeOfGrids[*iter].push_back(make_pair(i, j));
		}
	}
	/*for(map<int, vector<pair<int, int> > >::iterator iter = edgeOfGrids.begin(); iter != edgeOfGrids.end(); iter ++){
		printf("eid %d\n", iter->first);
		for(vector<pair<int, int> >::iterator it = iter->second.begin(); it != iter->second.end(); it ++)
			printf("(%d, %d)", it->first, it->second);
		printf("\n");
	}*/
	//initInfo();// load change at 5.30
	//cout << "end to load files..." << endl;
	//checkInfo();
	//for (map<int, int>::iterator iter = emptyEdgemap.begin(); iter != emptyEdgemap.end(); iter ++)
	//{
	//	printf("Empty %d %d\n", iter->first, iter->second);
	//}
	//printf("%d\n", (int)nodeToDistmap.size());
	//for (map<int, pair<double, double> >::iterator iter = nodeToDistmap.begin(); iter != nodeToDistmap.end(); iter ++)
	//{
	//	printf("ObjDist %lf %lf\n", iter->second.first, iter->second.second);
	//}
	/*set<int> tedge;
	for(int i = 0; i < GRID_H; i ++)
		for(int j = 0; j < GRID_W; j ++){
			if(mainGrid.gridCell[i][j].Edge.empty() == false){
				printf("Grid %d %d:\n", i, j);
				set<int> ids = mainGrid.gridCell[i][j].Edge;
				for(set<int>::iterator iter = ids.begin(); iter != ids.end(); iter ++){
					if (tedge.find(*iter) == tedge.end())tedge.insert(*iter);
					printf("%d\t", *iter);
				}
				printf("\n");
			}
		}
	printf("total edges:%d\n", (int)tedge.size());
	for (int i = 0; i < EDGE_NUM; i ++)
	{
		if (tedge.find(edgemap[i].eid) == tedge.end()){
			int sid = edgemap[i].startnid;
			int eid = edgemap[i].endnid;
			pair<int, int> pos_start = Grid::getLocFromGeo(loc(nodemap[sid].lat, nodemap[sid].lng));
			pair<int, int> pos_end = Grid::getLocFromGeo(loc(nodemap[eid].lat, nodemap[eid].lng));
			printf("eid %d s_lat %d s_lng %d e_lat %d e_lng %d\n", edgemap[i].eid, pos_start.first, pos_start.second, pos_end.first, pos_end.second);
		}
	}*/
}
void print_info();

void print_info(){
	string line = "(a) do exp\n(b) do init\n(c) gen ids of query vehicles\n(d) Init road network and write dist to file\n(e) exit...";
	cout << line << endl;
}
int readNumber(char str[]);

int readNumber(char str[]){
    stringstream ss(str);
    int num;
    ss >> num;
    return num;
}
double readDouble(char str[]);
double readDouble(char str[]){
	stringstream ss(str);
	double num;
	ss >> num;
	return num;
}
char* readString(char str[]);
char* readString(char str[]){
	//stringstream ss(str);
	//char name[20];
	//ss >> name;
	return str;
}
/**
 *  argv
 *  [1]:    k                   1,3,5
 *  [2]:    Exchange or not     0,1
 *  [3]:    Rsu nor not         0,1
 *	[4]:	Time				1000
 *  [5]:    knn time            30
 *	[6]:	fill rsu with obj in how many grids
 *	[7]:	how many grids around a rsu is assumed to be broadcasted
 *	[8]:	fresh of object message
 *	[9]:	warming up limit(number of queries finished)
 *	[10]:	range of knn message
 *	[11]:	cache size
 *  [12]:	coverage
 */

int main(int argc, char* argv[])
{
    Grid::testExchange = readNumber(argv[1]);
    Grid::RSU_SWITCH = readNumber(argv[2]);
	Grid::KNUM = readNumber(argv[3]);
	Grid::FRESH = readNumber(argv[4]);
	Grid::RANGE = readNumber(argv[5]);
	Grid::PERIOD = readDouble(argv[6]);//CS period
	Grid::RSU_N = readNumber(argv[7]);//read and do nothing
	Grid::FILLRSU = readNumber(argv[8]);//fill rsu with obj in how many grids
	Grid::TOP = readNumber(argv[9]);;//select which vehicle to switch in CS
	Grid::COVERAGE = readDouble(argv[10]);
	Grid::TYPE = readNumber(argv[11]);
	Grid::NAME = readString(argv[12]);

	Grid::BRORSU = 16;//how many grids around a rsu is assumed to be broadcasted
	Grid::BRO_PERIOD = 6;//the time period when a RSU broadcast once
	Grid::TIME = 1000;
	Grid::KnnTime = 30;
	Grid::CASIZE = 60;
	Grid::SPEED = 30;
	Grid::COMRANGE = 100;
	Grid::trans_high = loc(Grid::calculateCircleDist(Grid::low.lat, Grid::low.lng, Grid::high.lat, Grid::low.lng), Grid::calculateCircleDist(Grid::low.lat, Grid::low.lng, Grid::low.lat, Grid::high.lng));
	Grid::trans_low = loc(0, 0);
	//printf("%s\n", Grid::NAME);
	load_files();
    do_experiment();
//	while(true){
//		print_info();
//		string line = "";
//		cin >> line;
//		if(line == "a"){
//		//	int n[3] = { 1, 3, 5};
//		//	int t[10] = { 5, 10, 15, 20, 25, 30, 40, 60};
//			int t[4] = { 2, 10, 30, 60};
//		//	for (int j = 0; j < 3; j ++)
//		//	{			
//				for ( int i = 0; i < 1; i ++ )
//				{
//					Grid::TTL = t[i];
//					Grid::KNUM = 3;
//					do_experiment();
//					cout << VE_NUM << ' ' << Grid::TTL << ' ' << Grid::KNUM << ' ' << "finished" <<endl;
//					statistic.reSet();
//					for(map<int, vehicle>::iterator iter = vehiclemap.begin(); iter != vehiclemap.end(); iter ++){
//						iter->second.reSet();
//					}
//					mainGrid.reSet();
//					//gettrajectory(vehiclemap);// load change at 5.30
//				}
//		//	}
//
//		}else if(line == "b"){
//			load_files();
//		}else if(line == "e"){
//			cout << "exit ..." << endl;
//			break;
//		}else if(line == "c"){
//			queryingVehicleSelect((int)(VE_NUM / 10));
//		}else if(line == "d"){
//			writeDistToFile();
//		}else{
//			cout << "no such option!" << endl;
//		}
//	} 
}
