#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <iomanip>
#include <map>
#include "vehicleOperation.h"
#include "struct.h"
#include "loadOperation.h"
#include <sstream>
#include <string>
using namespace std;
class loc;

loc coordTrans(double lat, double lng)
{
	loc p;
	double distlat = Grid::trans_high.lat;
	double distlng = Grid::trans_high.lng;
	double lat_delta = Grid::high.lat - Grid::low.lat;
	double lng_delta = Grid ::high.lng - Grid::low.lng;
	p.lat = distlat * (lat - Grid::low.lat) / lat_delta;
	p.lng = distlng * (lng - Grid::low.lng) / lng_delta;
	return p;
}

void getnodes(node* nodemap, Grid& mainGrid, map<int, vector<node> >& edgeToObjmap)
{
	FILE * fFile;
	
	fFile = fopen("beijing.node", "r");
	if(fFile != NULL){
		while(!feof(fFile)){
			for (int i = 0; i < NODE_NUM; i++)
			{
				int id, eid;
				double lat, lng;
				int is_obj;
				fscanf(fFile, "%d %lf %lf %d %d", &id, &lat, &lng, &eid, &is_obj);
				if(lat < Grid::low.lat || lng < Grid::low.lng || lat > Grid::high.lat || lng > Grid::high.lng)
					cout << "Oops" << endl;
				loc p = coordTrans(lat, lng);
				//printf("%d %lf %lf\n", id, p.lat, p.lng);
				node tnode = node(id, eid, p.lat, p.lng, is_obj == 1 ? true : false);
				//node tnode = node(id, eid, lat, lng, is_obj == 1 ? true : false);
				pair<int, int> pos = Grid::getLocFromGeo(loc(p.lat, p.lng));
				if(tnode.is_object && tnode.eid != -1){
					mainGrid.gridCell[pos.first][pos.second].Node.insert(id);
					edgeToObjmap[eid].push_back(tnode);
				}
				nodemap[id] = tnode;
			}
		}
	}
    fclose(fFile);
	int num = 0;
	for(map<int, vector<node> >::iterator iter = edgeToObjmap.begin(); iter != edgeToObjmap.end(); iter ++){
		num += (int)iter->second.size();
		//printf("%d\n", iter->first);
	}
	if(debug)printf("load nodes fin %d objects on %d edges!\n", num, (int)edgeToObjmap.size());

	//cout << "load nodes fin!" << endl;
    //for(j=0;j<i-1;j++)cout<<fixed<<setprecision(8)<<node_buf[j].nid<<' '<<node_buf[j].lat<<' '<<node_buf[j].lng<<endl;
}

void getedges(node* nodemap, edge* edgemap, map<int, vector<pair<int, int> > >& nodeToEdgemap,  map<int, vector<node> >& edgeToObjmap, set<int>& emptyEdgemap, Grid& mainGrid, set<int>& edgeInGridmap)
{
	FILE * fFile;
    fFile = fopen("beijing.edge", "r");
    if(fFile != NULL)
    {
		while (!feof(fFile))
		{
			for (int i = 0; i < EDGE_NUM; i++){
				int lineid, sid, eid, is_twoway;
				fscanf(fFile, "%d %d %d %d", &lineid, &sid, &eid, &is_twoway);
				//double dist = Grid::calculateDist(nodemap[sid].getLoc(), nodemap[eid].getLoc());
				edge tedge = edge(lineid, sid, eid, 1, is_twoway, Grid::getRoadNetworkDist(sid, eid));
				edgemap[tedge.eid] = tedge;
				nodeToEdgemap[sid].push_back(make_pair(lineid, eid));
				nodeToEdgemap[eid].push_back(make_pair(lineid, sid));
				if (edgeToObjmap.find(lineid) == edgeToObjmap.end())
				{
					//double dist = Grid::getRoadNetworkDist(sid, eid);
					emptyEdgemap.insert(lineid);
				}
				pair<int, int> pos_start = Grid::getLocFromGeo(loc(nodemap[sid].lat, nodemap[sid].lng));
				pair<int, int> pos_end = Grid::getLocFromGeo(loc(nodemap[eid].lat, nodemap[eid].lng));
				if((pos_start.first == pos_end.first) && (pos_start.second == pos_end.second)){
					edgeInGridmap.insert(lineid);
					mainGrid.gridCell[pos_start.first][pos_start.second].Edge.insert(lineid);
					//if(pos_start.first == 31 && pos_start.second == 5)printf("%d\n", lineid);
					continue;
				}
				if (pos_start.first == pos_end.first)
				{
					int i = pos_start.first;
					//printf("lineid %d %lf %lf %d %d %lf %lf %d %d\n", lineid, nodemap[sid].lat, nodemap[sid].lng, pos_start.first, pos_start.second, nodemap[eid].lat, nodemap[eid].lng, pos_end.first, pos_end.second);
					for (int j = MIN(pos_start.second, pos_end.second); j <= MAX(pos_start.second, pos_end.second); j ++)
					{
						
						//printf("& %d %d\n", i, j);
						loc tleftdown = loc( i * grid_lat + Grid::trans_low.lat, j * grid_lng + Grid::trans_low.lng );
						loc trightup = loc( (i + 1) * grid_lat + Grid::trans_low.lat, (j + 1) * grid_lng + Grid::trans_low.lng );
						loc rect[4] = {tleftdown, loc(tleftdown.lat, trightup.lng), trightup, loc(trightup.lat, tleftdown.lng)};
						if(MathFun::isSegmentInterOrInsideWithRect(line(loc(nodemap[sid].lat, nodemap[sid].lng), loc(nodemap[eid].lat, nodemap[eid].lng)), rect) == true){
							mainGrid.gridCell[i][j].Edge.insert(lineid);
							//if(i == 31 && j == 5)printf("1 %d\n", lineid);
						}
					}
				}
				else{
					if (pos_start.second == pos_end.second)
					{
						int j = pos_start.second;
						//printf("lineid %d %lf %lf %d %d %lf %lf %d %d\n", lineid, nodemap[sid].lat, nodemap[sid].lng, pos_start.first, pos_start.second, nodemap[eid].lat, nodemap[eid].lng, pos_end.first, pos_end.second);
						for(int i = MIN(pos_start.first, pos_end.first); i <= MAX(pos_start.first, pos_end.first); i ++)
						{
							//printf("* %d %d\n", i, j);
							loc tleftdown = loc( i * grid_lat + Grid::trans_low.lat, j * grid_lng + Grid::trans_low.lng );
							loc trightup = loc( (i + 1) * grid_lat + Grid::trans_low.lat, (j + 1) * grid_lng + Grid::trans_low.lng );
							loc rect[4] = {tleftdown, loc(tleftdown.lat, trightup.lng), trightup, loc(trightup.lat, tleftdown.lng)};
							if(MathFun::isSegmentInterOrInsideWithRect(line(loc(nodemap[sid].lat, nodemap[sid].lng), loc(nodemap[eid].lat, nodemap[eid].lng)), rect) == true){
								mainGrid.gridCell[i][j].Edge.insert(lineid);
								//if(i == 31 && j == 5)printf("2 %d\n", lineid);
							}
						}
					}
					else{
						//printf("lineid %d %lf %lf %d %d %lf %lf %d %d\n", lineid, nodemap[sid].lat, nodemap[sid].lng, pos_start.first, pos_start.second, nodemap[eid].lat, nodemap[eid].lng, pos_end.first, pos_end.second);
						for(int i = MIN(pos_start.first, pos_end.first); i <= MAX(pos_start.first, pos_end.first); i ++)
							for(int j = MIN(pos_start.second, pos_end.second); j <= MAX(pos_start.second, pos_end.second); j ++)
							{
								//printf("* %d %d\n", i, j);
								loc tleftdown = loc( i * grid_lat + Grid::trans_low.lat, j * grid_lng + Grid::trans_low.lng );
								loc trightup = loc( (i + 1) * grid_lat + Grid::trans_low.lat, (j + 1) * grid_lng + Grid::trans_low.lng );
								loc rect[4] = {tleftdown, loc(tleftdown.lat, trightup.lng), trightup, loc(trightup.lat, tleftdown.lng)};
								if(MathFun::isSegmentInterOrInsideWithRect(line(loc(nodemap[sid].lat, nodemap[sid].lng), loc(nodemap[eid].lat, nodemap[eid].lng)), rect) == true){
									mainGrid.gridCell[i][j].Edge.insert(lineid);
									//if(i == 31 && j == 5)printf("3 %d\n", lineid);
								}
							}
					}
				}
			}
		}
    }
    fclose(fFile);
	if(debug)printf("load edges fin\n");
}

//void getobjects(map<int, object>& objectmap, node* nodemap, edge* edgemap)
//{
//	for(int nodeId = 0; nodeId < NODE_NUM; nodeId ++){
//		node* tnode1 = &nodemap[nodeId];
//		if(tnode1->is_object && tnode1->eid != -1){
//			node tnode2 = nodemap[edgemap[tnode1->eid].startnid];
//			double dist = Grid::calculateDist(tnode1->lat, tnode1->lng, tnode2.lat, tnode2.lng);
//			object obj = object(tnode1->nid, tnode1->eid, dist);
//			objectmap[obj.oid] = obj;
//		}
//	}
//	cout << "load objects fin!\tobjects size:\t" << objectmap.size() << endl;
//}

void getrsu(Roadsideunit * rsumap, Grid& mainGrid, map<int, vector<node> >& edgeToObjmap, set<int>& emptyEdgemap, edge * edgemap, node * nodemap, map<int, pair<double, double> >& nodeToDistmap, loc * rsuLoc)
{
	lableRSU(mainGrid, rsuLoc, Grid::FILLRSU, LAT_NUM, LNG_NUM);
	int id = EMPTY_EDGE;
	map<int, RsuObj>emptySet;
	map<int, vector<RsuObj> >objEdgeSet;
	map<int, vector<pair<int, int> > > rsuGrid;

	//map<int, vector<pair<int, int> > > edgeToGridmap;
	//for(int i = 0; i < GRID_H; i ++)
	//	for(int j = 0; j < GRID_W; j ++){
	//		if(mainGrid.gridCell[i][j].Edge.empty() == false){
	//			pair<int, int> loca = make_pair(i, j);
	//			//printf("Grid %d %d:\n", i, j);
	//			set<int> ids = mainGrid.gridCell[i][j].Edge;
	//			for(set<int>::iterator iter = ids.begin(); iter != ids.end(); iter ++){
	//				edgeToGridmap[*iter].push_back(loca);
	//			}
	//		}
	//	}
	/*for (map<int, vector<pair<int, int> > >::iterator iter = edgeToGridmap.begin(); iter != edgeToGridmap.end(); iter ++)
	{
		printf("edge %d\n", iter->first);
		for (vector<pair<int, int> >::iterator it = iter->second.begin(); it != iter->second.end(); it ++)
		{
			printf("(%d, %d)\t", it->first, it->second);
		}
		printf("\n");
	}*/
	for (set<int>::iterator it = emptyEdgemap.begin(); it != emptyEdgemap.end(); it ++)
	{
		int edgeid = *it;
		int sid = edgemap[edgeid].startnid;
		int eid = edgemap[edgeid].endnid;
		double dist = edgemap[edgeid].length;
		set<int> rsuBro;
		RsuObj rsuobj = RsuObj(id, edgeid, 0.0, 0.0, loc(0, dist), SIMULATION_time);
		emptySet[edgeid] = rsuobj;
		id ++;
	}
	printf("%d empty edge with no obj %d\n", id, (int)emptyEdgemap.size());
	/*for (map<int, RsuObj>::iterator iter = emptySet.begin(); iter != emptySet.end(); iter ++)
	{
		printf("%d\n", iter->first);
		if(emptyEdgemap.find(iter->first) == emptyEdgemap.end())printf("cannot find %d\n", iter->first);
	}*/
	for (map<int, vector<node> >:: iterator iter = edgeToObjmap.begin(); iter != edgeToObjmap.end(); iter ++)
	{
		int edgeid = iter->first;
		vector<node> objects = iter->second;
		vector<pair<double, int> > objdist;
		vector<pair<double, int> > findist;
		int tsid = edgemap[edgeid].startnid;
		int teid = edgemap[edgeid].endnid;
		if (objects.empty())continue;
		double tlen = Grid::calculateDist(nodemap[tsid].getLoc(), nodemap[teid].getLoc());
		for (int i = 0; i < objects.size(); i ++)
		{
			double dist = Grid::calculateDist(nodemap[tsid].getLoc(), objects[i].getLoc());
			objdist.push_back(make_pair(dist, objects[i].nid));
		}
		sort(objdist.begin(), objdist.end());
		findist.push_back(make_pair(0.0, tsid));
		for (int j = 0; j < objdist.size(); j ++)
		{
			findist.push_back(objdist[j]);
		}
		findist.push_back(make_pair(tlen, teid));
		for (int j = 1; j < findist.size() - 1; j ++)
		{
			int temp = findist[j].second;
			RsuObj rsuobj = RsuObj(nodemap[temp].nid, nodemap[temp].eid, nodemap[temp].lat, nodemap[temp].lng, loc(findist[j - 1].first, findist[j + 1].first), nodemap[temp].time_to_live);
			objEdgeSet[edgeid].push_back(rsuobj);
			nodeToDistmap[nodemap[temp].nid] = make_pair(findist[j - 1].first, findist[j + 1].first);
		}
		/*printf("# edge %d\n", iter->first);
		for (vector<node>::iterator it = iter->second.begin(); it != iter->second.end(); it ++)
		{
			printf("# %d\n", it->nid);
		}*/
	}
	/*for (map<int, vector<RsuObj> >::iterator iter = objEdgeSet.begin(); iter != objEdgeSet.end(); iter ++)
	{
		printf("edge %d\n", iter->first);
		for (vector<RsuObj>::iterator it = iter->second.begin(); it != iter->second.end(); it ++)
		{
			printf("%d %d %lf %lf %lf %lf %lf\n", it->nid, it->eid, it->lat, it->lng, it->roadSegments.lat, it->roadSegments.lng, it->time_to_live);
		}
	}*/
	for(int i = 0; i < GRID_H; i ++)
		for(int j = 0; j < GRID_W; j ++){
			int label = (unsigned int)mainGrid.gridCell[i][j].Rsu.size();
			if(label > 0){
				for (int k = 0; k < label; k ++){
					int rsuid = mainGrid.gridCell[i][j].Rsu[k] - 1;
					rsuGrid[rsuid].push_back(make_pair(i, j));
				}
			}
		}
	/*for (map<int, vector<pair<int, int> > >::iterator iter = rsuGrid.begin(); iter != rsuGrid.end(); iter ++)
	{
		printf("rsu %d\n", iter->first);
		for (vector<pair<int, int> >::iterator it = iter->second.begin(); it != iter->second.end(); it ++)
		{
			printf("(%d, %d)\n", it->first, it->second);
		}
	}*/
	for (map<int, vector<pair<int, int> > >::iterator iter = rsuGrid.begin(); iter != rsuGrid.end(); iter ++)
	{
		set<int> broEdge;
		for (vector<pair<int, int> >::iterator it = iter->second.begin(); it != iter->second.end(); it ++)
		{
			int i = it->first;
			int j = it->second;
			if(mainGrid.gridCell[i][j].Edge.empty() == false){
				set<int> ids = mainGrid.gridCell[i][j].Edge;
				for(set<int>::iterator iter = ids.begin(); iter != ids.end(); iter ++){
					if(broEdge.find(*iter) == broEdge.end())broEdge.insert(*iter);
				}
			}
		}
		//printf("rsu %d\n", iter->first);
		vector<pair<double, int> >edgeIds;
		for (set<int>::iterator ii = broEdge.begin(); ii != broEdge.end(); ii ++)
		{
			//printf("%d\n", *ii);
			int id = *ii;
			double d = RoadDistCal;
			int sid = nodemap[edgemap[id].startnid].nid;
			int eid = nodemap[edgemap[id].endnid].nid;
			int nids[2]= {sid, eid};
			for (int i = 0; i < 2; i ++){
				double dist = Grid::calculateDist(rsuLoc[iter->first], nodemap[nids[i]].getLoc());
				d = MIN(d, dist);
			}
			edgeIds.push_back(make_pair(d, id));
		}
		sort(edgeIds.begin(), edgeIds.end(), rsuSort());
		for (vector<pair<double, int> >::iterator jj = edgeIds.begin(); jj != edgeIds.end(); jj ++)
		{
			//printf("%d\t%lf\n", jj->second, jj->first);
			int eid = jj->second;
			if(emptySet.find(eid) != emptySet.end())
				rsumap[iter->first].objects.push_back(emptySet[eid]);
			else{
				if(objEdgeSet.find(eid) == objEdgeSet.end())
					printf("Cannot find %d\n", eid);
				for (vector<RsuObj>::iterator kk = objEdgeSet[eid].begin(); kk != objEdgeSet[eid].end(); kk ++)
				{
					rsumap[iter->first].objects.push_back(*kk);
				}
			}
		}	
	}
	for(int i = 0; i < GRID_H; i ++)
		for(int j = 0; j < GRID_W; j ++){
			mainGrid.gridCell[i][j].Rsu.clear();
		}
		for (int i = 0; i < LAT_NUM * LNG_NUM; i ++){
			printf("load RSU %d fin\t node size:%lu at %lf %lf\n", i, rsumap[i].objects.size(), rsuLoc[i].lat, rsuLoc[i].lng);
			/*for (int j = 0; j < rsumap[i].objects.size(); j ++){
				printf("%d %d %lf %lf %lf %lf\n",rsumap[i].objects[j].nid, rsumap[i].objects[j].eid, rsumap[i].objects[j].lat, rsumap[i].objects[j].lng, rsumap[i].objects[j].roadSegments.lat, rsumap[i].objects[j].roadSegments.lng);
			}*/
		}
	printf("getrsu fin!\n");
}

void getCoverArea(Grid* mainGrid, pair<int, int> center, int rsu_cover_range, int color){
    int dx[] = {0, 1, -1, 0};
    int dy[] = {1, 0, 0, -1};
    queue<pair<int, int> > Q;
    set<pair<int, int> > cnt;
    Q.push(center);
    cnt.insert(center);
    mainGrid->gridCell[center.first][center.second].Rsu.push_back(color);
    int curRange = 1;
    while(!Q.empty()){
        pair<int, int> top = Q.front(); Q.pop();
        for(int d = 0; d < 4; d++){
            pair<int, int> nCt = make_pair(top.first + dx[d], top.second + dy[d]);
            if(Grid::is_Inrange(nCt.first, nCt.second) && cnt.find(nCt) == cnt.end()){
                cnt.insert(nCt);
                Q.push(nCt);
                mainGrid->gridCell[nCt.first][nCt.second].Rsu.push_back(color);
                if ((++ curRange) >= rsu_cover_range) {
                    return;
                }
            }
        }
    }
}

void lableRSU(Grid& mainGrid, loc* rsuLoc, int rsu_cover_range, int lat_range, int lng_range)
{
	int label = 1;
	double lat_delat = ( Grid::trans_high.lat - Grid::trans_low.lat ) / lat_range;
	double lng_delat = ( Grid::trans_high.lng - Grid::trans_low.lng ) / lng_range;
	for (int i = 0; i < lat_range; i++){
		for(int j = 0; j < lng_range; j++){
			double temp_lat = lat_delat * i + Grid::trans_low.lat + 0.5 * lat_delat; 
			double temp_lng = lng_delat * j + Grid::trans_low.lng + 0.5 * lng_delat;
			
			pair<int, int> center = Grid::getLocFromGeo(temp_lat, temp_lng);
			getCoverArea(&mainGrid, center, rsu_cover_range, label ++);	
			int id = i * lat_range + j;
			//printf("test %d %d %d\n", i, j, id);
			rsuLoc[id] = loc(temp_lat, temp_lng);
		}
	}
}

void labelBro(Grid& mainGrid, int rsu_cover_range, int lat_range, int lng_range)
{
	int label = 1;
	double lat_delat = ( Grid::trans_high.lat - Grid::trans_low.lat ) / lat_range;
	double lng_delat = ( Grid::trans_high.lng - Grid::trans_low.lng ) / lng_range;
	for (int i = 0; i < lat_range; i++){
		for(int j = 0; j < lng_range; j++){
			double temp_lat = lat_delat * i + Grid::trans_low.lat + 0.5 * lat_delat; 
			double temp_lng = lng_delat * j + Grid::trans_low.lng + 0.5 * lng_delat;

			pair<int, int> center = Grid::getLocFromGeo(temp_lat, temp_lng);
			getCoverArea(&mainGrid, center, rsu_cover_range, label ++);	
		}
	}
}
