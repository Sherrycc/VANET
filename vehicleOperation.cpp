#include "vehicleOperation.h"
#include <iostream>
#include <set>
#include <cstdlib>
#include <sstream>
#include <string>
#include <fstream>
using namespace std;

int generatorOfRandomNumber(set<int>& dict);
int generatorOfRandomNumber(set<int>& dict){
    int num = rand() % VE_NUM;
    while(dict.find(num) != dict.end()){
        num = rand() % VE_NUM;
    }
    dict.insert(num);
    return num;
}

void queryingVehicleSelect(int num){
    srand(0);
	int queryingid[VE_NUM];
    set<int> dict;
    for(int i = 0; i < num; i++){
        queryingid[i] = generatorOfRandomNumber(dict);
    }
	ofstream outfile;
	outfile.open("queryingvehicles1000.id");
	for(int i = 0; i < num; i++){
		outfile << queryingid[i] << endl;
	}
	outfile.close();
}
void initial_w_vehicle(vehicle* vehiclemap, edge* edgemap,node* nodemap, int cur_time, Grid& mainGrid, Statistic& statistic)
{
    ifstream infile;
    string line;
    infile.open("warmingvehicles.id");
    if(infile.is_open())
    {
        while(infile.good() && !infile.eof())
        {
            getline(infile,line);
            stringstream in(line);
            int vid;  
			//int tempx, tempy;
			//double rlat, rlng;
            in >> vid;
			vehiclemap[vid].setQueryPosition();
			pair<int, int> pos = Grid::getLocFromGeo(vehiclemap[vid].getCurrentTrajectory().location);
			/*if(mainGrid.gridCell[pos.first][pos.second].Rsu != 0)
			{
				if ( Grid::calculateDist(loc(rlat, rlng), vehiclemap[vid].getCurrentTrajectory().location) <= vehiclemap[vid].communication_range )
				{
					cout << "in the range of RSUs" << endl;
					set<int> std_ids = vehiclemap[vid].grid->knnQueryInGrid(vehiclemap[vid].getCurrentTrajectory(), Grid::KNUM);
					cout << "std :\t";
					for(set<int>::iterator iter = std_ids.begin(); iter != std_ids.end(); iter ++){
						cout << (*iter) << "\t";
					}
					cout << endl;
					statistic.precision ++;
					statistic.completeknn ++;
					statistic.accuracy += 1.0;
				}
			}
			else
			{*/
            //query_msg msg = query_msg(vid, cur_time, Grid::KNUM, cur_time, vehiclemap[vid].getQueryTrajaectory(), KNN_QUERY);
			query_msg msg = query_msg(vid, cur_time, Grid::KNUM, Grid::KnnTime + cur_time, vehiclemap[vid].getQueryTrajaectory(), KNN_QUERY);
			//cout << vid << ' ' << msg.query_id << endl;
				//cout << vid << ' ' << vehiclemap[vid].compute_ttl_test ( edgemap, nodemap, query_msg::cache_range, cur_time) << endl;
			//msg.endtime = msg.endtime + vehiclemap[vid].compute_ttl_test(edgemap, nodemap, msg.range, cur_time);	
			statistic.range += msg.cache_range;
			vehiclemap[vid].query_msg_queue[msg.query_id] = msg;
			//}
				//if(vid == 0)printf("issue query trajectory %d %lf %lf\n", vehiclemap[vid].getQueryTrajaectory().eid, vehiclemap[vid].getQueryTrajaectory().location.lat, vehiclemap[vid].getQueryTrajaectory().location.lng);


        }
    }
    infile.close();
}

void initial_q_vehicle(vehicle* vehiclemap, edge* edgemap,node* nodemap, int cur_time, Grid& mainGrid, Statistic& statistic)
{
    ifstream infile;
    string line;
    infile.open("queryingvehicles.id");
    if(infile.is_open())
    {
        while(infile.good() && !infile.eof())
        {
            getline(infile,line);
            stringstream in(line);
            int vid;  
			//int tempx, tempy;
			//double rlat, rlng;
            in >> vid;
			vehiclemap[vid].setQueryPosition();
			pair<int, int> pos = Grid::getLocFromGeo(vehiclemap[vid].getCurrentTrajectory().location);
			/*if(mainGrid.gridCell[pos.first][pos.second].Rsu != 0)
			{
				if ( Grid::calculateDist(loc(rlat, rlng), vehiclemap[vid].getCurrentTrajectory().location) <= vehiclemap[vid].communication_range )
				{
					cout << "in the range of RSUs" << endl;
					set<int> std_ids = vehiclemap[vid].grid->knnQueryInGrid(vehiclemap[vid].getCurrentTrajectory(), Grid::KNUM);
					cout << "std :\t";
					for(set<int>::iterator iter = std_ids.begin(); iter != std_ids.end(); iter ++){
						cout << (*iter) << "\t";
					}
					cout << endl;
					statistic.precision ++;
					statistic.completeknn ++;
					statistic.accuracy += 1.0;
				}
			}
			else
			{*/
            //query_msg msg = query_msg(vid, cur_time, Grid::KNUM, cur_time, vehiclemap[vid].getQueryTrajaectory(), KNN_QUERY);
			query_msg msg = query_msg(vid, cur_time, Grid::KNUM, Grid::KnnTime + cur_time, vehiclemap[vid].getQueryTrajaectory(), KNN_QUERY);
				//cout << vid << ' ' << vehiclemap[vid].compute_ttl_test ( edgemap, nodemap, query_msg::cache_range, cur_time) << endl;
			//msg.endtime = msg.endtime + vehiclemap[vid].compute_ttl_test(edgemap, nodemap, msg.range, cur_time);	
			statistic.range += msg.cache_range;
			vehiclemap[vid].query_msg_queue[msg.query_id] = msg;
			//}
				//if(vid == 0)printf("issue query trajectory %d %lf %lf\n", vehiclemap[vid].getQueryTrajaectory().eid, vehiclemap[vid].getQueryTrajaectory().location.lat, vehiclemap[vid].getQueryTrajaectory().location.lng);


        }
    }
    infile.close();
}


void add_to_vtable()
{
        //vehicle.localcache.vtable
}
void add_to_otable()
{
        //vehicle.localcache.otable
}
int findneighbor(object k)
{
	return 0;
}

