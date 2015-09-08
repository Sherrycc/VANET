#include "struct.h"
#include <iostream>
#include <cmath>
#include <queue>
#include <algorithm>
#include <map>
#include <tr1/unordered_map>
#include <vector>
#include <sys/time.h>
#include <stdio.h>
using namespace std;

//const double vehicle::communication_range = Grid::COMRANGE;
const double vehicle::timechip = 1.0 / vehicle::chipNum;

/************************************************************************/
/* node                                                                     */
/************************************************************************/

loc node::getLoc()
{
	return loc(lat, lng);
}

node::node( int nid, int eid, double lat, double lng, bool is_object )
{
	this->nid = nid;
	this->eid = eid;
	this->lat = lat;
	this->lng = lng;
	this->is_object = is_object;
	this->number_of_interarrivals = 0;
	this->sum_of_interarrival_time = 0.0;
	this->interarrival = true;
	//this->time_to_live = 1.0 * ((rand() % Grid::FRESH) + 1); //rand a time_to_live 1~fresh
	this->time_to_live = 1.0 * Grid::FRESH;
	//if(this->is_object == true)printf("%d %lf\n", this->nid = nid, this->time_to_live);
}

node::node(int nid, int eid, double lat, double lng, double time_to_live)
{
	this->nid = nid;
	this->eid = eid;
	this->lat = lat;
	this->lng = lng;
	this->time_to_live = time_to_live;
}
node::node()
{

}

bool node::isObject()
{
	return is_object == true && eid != -1;
}
/************************************************************************/
/* ttl                                                                     */
/************************************************************************/
ttledge::ttledge (int _snid, int _eid, double _length, double _ttl, double _tEdge)
{
	snid = _snid;
	eid = _eid;
	length = _length;
	ttl = _ttl;
	tEdge = _tEdge;
}

ttledge::ttledge()
{

}
/************************************************************************/
/* edge                                                                     */
/************************************************************************/
edge::edge( int _eid, int _startnid, int _endnid, int _Uturn, int _ways, double _length )
{
	eid = _eid;
	startnid = _startnid;
	endnid = _endnid;
	Uturn = _Uturn;
	ways = _ways;
	length = _length;
	/*number_of_interarrivals_sid = 0;
	sum_of_interarrival_time_sid = 0.0;
	interarrival_sid = true;
	number_of_interarrivals_eid = 0;
	sum_of_interarrival_time_eid = 0.0;
	interarrival_eid = true;*/
}

edge::edge()
{

}

/************************************************************************/
/* trajectory                                                                     */
/************************************************************************/
trajectory::trajectory( int _eid, loc _location )
{
	eid = _eid;
	location = _location;
}

trajectory::trajectory( int _eid, double lat, double lng )
{
	eid = _eid;
	location = loc(lat, lng);
}

trajectory::trajectory()
{
	eid = -1;
}

bool trajectory::operator==( const trajectory& b ) const
{
	if(eid == b.eid && location == b.location)return true;
	return false;
}

/************************************************************************/
/* object                                                                     */
/************************************************************************/
object::object( int _oid, int _eid, double _dist )
{
	oid = _oid;
	eid = _eid;
	communication_range = Grid::COMRANGE;
	dist = _dist;
}

object::object()
{

}


/************************************************************************/
/*        cache                                                              */
/************************************************************************/

vector<int> cache::getFristKobjects( trajectory query_trajectory, int kNum, double range)
{
	double d =0.0;
	vector< pair<double, int> > tmpNodes;
	//cout<< objects.size() << endl;//by 0731
	for( tr1::unordered_map<int, node>::iterator iter = objects.begin(); iter != objects.end() ;iter ++){
		//cout << iter->second.eid << ' ' << iter->second.nid << ' '<<  iter->second.time_to_live << endl;//by 0731
		//if(iter->second.time_to_live <= 0) continue;
		double dist = Grid::caculateRoadNetworkDist(query_trajectory, trajectory(iter->second.eid, iter->second.getLoc()));
		//cout << iter->second.eid << ' ' << iter->second.nid << ' '<< dist << endl;
		if(MathFun::D(dist - range) <= 0){
			//cout << iter->second.eid << ' ' << iter->second.nid << ' '<< dist << ' ' << range << endl;
			tmpNodes.push_back(make_pair(dist, iter->second.nid));
		}
	}
	//cout << "tmpNodes size" << " "<< tmpNodes.size() << endl;// by 0729
	sort(tmpNodes.begin(), tmpNodes.end());
	vector<int> ans;
	//for(int i = 0; i < tmpNodes.size() && i < kNum; i++){
	for(int i = 0; i < tmpNodes.size() ; i++){
		if (ans.size() >= kNum && MathFun::D(tmpNodes[i].first - d) > 0)return ans;
		ans.push_back(tmpNodes[i].second);
		if (ans.size() == kNum) d = tmpNodes[i].first;
		//cout << "getFirstKojb :" << ' ' << tmpNodes[i].second << ' ' << tmpNodes[i].first << endl;// add at 6.1
		/*cerr << tmpNodes[i].nid << ",";
		cerr << Grid::caculateRoadNetworkDist(query_trajectory, trajectory(tmpNodes[i].eid, tmpNodes[i].getLoc())) << "\t";*/
	}
	return ans;
}


void cache::updateRoadSegment( int eid, pair<double, double> segment )
{
	if(segment.first > segment.second){
		swap(segment.first, segment.second);
	}
	vector<pair<double, double> > segments, newSegments;
	if(roadSegments.find(eid) != roadSegments.end()){
		segments = roadSegments[eid];
	}
	segments.push_back(segment);
	sort(segments.begin(), segments.end());
	pair<double, double> curSegment;
	for(vector<pair<double, double> >::iterator iter = segments.begin(); iter != segments.end(); iter ++){
		if(iter == segments.begin()){
			curSegment = *iter;
		}else{
			curSegment.second = max(curSegment.second, iter->second);
			/*if(MathFun::D(curSegment.second - iter->first) >= 0){
				curSegment.second = max(curSegment.second, iter->second);
			}else{
				newSegments.push_back(curSegment);
				curSegment = *iter;
			}*/
		}
	}
	newSegments.push_back(curSegment);
	roadSegments[eid] = newSegments;
}

int cache::getCacheSizeWithInRange(trajectory query_trajectory, double range)
{

	if(last_query_tra == query_trajectory)return size_in_range;
	last_query_tra = query_trajectory;
	int ans = 0;
	for( tr1::unordered_map<int, node>::iterator iter = objects.begin(); iter != objects.end() ;iter ++){
		//if (iter->second.time_to_live <= 0)continue;
		double dist = Grid::caculateRoadNetworkDist(query_trajectory, trajectory(iter->second.eid, iter->second.getLoc()));			
		if(MathFun::D(dist - range) <= 0){			
			ans ++;
		}
	}
	size_in_range = ans;
	return ans;
}

bool cache::isIdExisted( int id )
{
	return objects.find(id) != objects.end();
}

void cache::insertNewNode( node objectnode )
{
	objects[objectnode.nid] = objectnode;
}
void cache::updateEdgesig(int eid)
{
	if(edge_signature.find(eid) == edge_signature.end())edge_signature[eid] = 1;
	else
		edge_signature[eid] ++;
}
double cache::getRangeOfRangeQuery(trajectory query_trajectory, int kNum, double r)
{
	//if(getCacheSizeWithInRange(query_trajectory, r) < kNum)return r;
	vector< pair<double, int> > tmpNodes;
	for( tr1::unordered_map<int, node>::iterator iter = objects.begin(); iter != objects.end() ;iter ++){
		//if (iter->second.time_to_live <= 0)continue;
		double dist = Grid::caculateRoadNetworkDist(query_trajectory, trajectory(iter->second.eid, iter->second.getLoc()));
		if(MathFun::D(dist - r) <= 0){
			tmpNodes.push_back(make_pair(dist, iter->second.nid));
		}
	}
	sort(tmpNodes.begin(), tmpNodes.end());
	//for (vector< pair<double, int> >::iterator i = tmpNodes.begin(); i != tmpNodes.end(); i ++)
	//{
	//	cout << i->second << ' ' << i->first << endl; // add at 5.31
	//}
	//if(getCacheSizeWithInRange(query_trajectory, r) < kNum)return r;
	double dist = 0;
	if((int)tmpNodes.size() < kNum)dist = tmpNodes.back().first;
	else
		dist = tmpNodes[kNum-1].first;
	//cout << "dist" << ' ' << dist << endl;
	return dist;
}

map<int, vector<pair<double, double> > > cache::getRoadSementsCache(trajectory query_trajectory, double range, node* nodemap, edge* edgemap)
{
	map<int, vector<pair<double, double> > > ans;
	for(map<int, vector<pair<double, double> > >::iterator iter = roadSegments.begin(); iter != roadSegments.end() ;iter ++){
		int sid = nodemap[edgemap[iter->first].startnid].nid;
		int eid = nodemap[edgemap[iter->first].endnid].nid;
		int nids[2]= {sid, eid};
		for (int i = 0; i < 2; i ++){
			double dist = Grid::caculateRoadNetworkDist(query_trajectory, nids[i]);
			if(MathFun::D(dist - range) <= 0){
				ans[iter->first] = iter->second;
				break;
			}
		}
	}
	return ans;
}

bool cache::checkDistOfEdge(int edgeid, trajectory query_trajectory, double range, node* nodemap, edge* edgemap)
{
	int sid = nodemap[edgemap[edgeid].startnid].nid;
	int eid = nodemap[edgemap[edgeid].endnid].nid;
	int nids[2]= {sid, eid};
	for (int i = 0; i < 2; i ++){
		double dist = Grid::caculateRoadNetworkDist(query_trajectory, nids[i]);
		if(MathFun::D(dist - range) <= 0){
			return true;
		}
	}
	return false;
}

void cache::updateRoadSegments( int eid, vector<pair<double, double> > segments )
{
	vector<pair<double, double> > newSegments;
	if(roadSegments.find(eid) == roadSegments.end()){
		roadSegments[eid] = segments;
		return;
	}else{
		newSegments = roadSegments[eid];
		for(vector<pair<double, double> >::iterator iter = newSegments.begin(); iter != newSegments.end(); iter ++){
			segments.push_back(*iter);
		}
		newSegments.clear();
	}
	sort(segments.begin(), segments.end());
	pair<double, double> curSegment;
	for(vector<pair<double, double> >::iterator iter = segments.begin(); iter != segments.end(); iter ++){
		if(iter == segments.begin()){
			curSegment = *iter;
		}else{
			if(MathFun::D(curSegment.second - iter->first) >= 0){
				curSegment.second = max(curSegment.second, iter->second);
			}else{
				newSegments.push_back(curSegment);
				curSegment = *iter;
			}
		}
	}
	newSegments.push_back(curSegment);
	roadSegments[eid] = newSegments;
}

void cache::timeElapseForObjects(msg_statistic& caSta, node * nodemap, edge * edgemap, int s_time, trajectory traj)
{
	map<int, int> edge_check;
	for( tr1::unordered_map<int, node>::iterator iter = objects.begin(); iter != objects.end(); ){
		iter->second.time_to_live -= vehicle::timechip;
		if (iter->second.time_to_live <= 0){
			if (edge_signature.find(iter->second.eid) != edge_signature.end())
			{
				edge_signature[iter->second.eid] --;
				if(edge_signature[iter->second.eid] == 0)
				{
					roadSegments.erase(iter->second.eid);
					edge_signature.erase(iter->second.eid);
				}
			}
			else
				printf("Oops, not consist\n");
			objects.erase(iter ++);
		}
		else{
			if(edge_check.find(iter->second.eid) == edge_check.end())
				edge_check[iter->second.eid] = 1;
			else
				edge_check[iter->second.eid] ++;
			iter ++;
		}
	}
	//if(s_time == 1200)printf("before %d\n", (int)roadSegments.size());
	for (map<int, vector<pair<double, double> > >::iterator i = roadSegments.begin(); i != roadSegments.end();){
		//caSta.road_size += i->second.size();
		if (edge_signature[i->first] == 0){
			if ((Grid::testExchange == 0) && (checkDistOfEdge(i->first, traj, Grid::RANGE, nodemap, edgemap) == false)){
			//if (checkDistOfEdge(i->first, traj, Grid::RANGE, nodemap, edgemap) == false){
				edge_signature.erase(i->first);
				roadSegments.erase(i ++);
			}
			else{
				caSta.road_size += i->second.size();
				i ++;
			}
		}
		else{
			caSta.road_size += i->second.size();
			i ++;
		}
		//RDsize += i->second.size() * 16;//16 = 2 * DOUBLE
	}
	//if(s_time == 1200)printf("after %d\n", (int)roadSegments.size());
	//for (map<int, vector<pair<double, double> > >::iterator i = roadSegments.begin(); i != roadSegments.end(); i ++){
	//	caSta.road_size += i->second.size();
	//	//RDsize += i->second.size() * 16;//16 = 2 * DOUBLE
	//}
	//RDsize += roadSegments.size() * INT;
	caSta.obj_size += (int)objects.size();
	caSta.road_num += (int)roadSegments.size();
	//if((int)objects.size()> CASIZE)printf("# %d\n", (int)objects.size());
	/*for (map<int, int>::iterator it = edge_check.begin(); it != edge_check.end(); it ++)
	{
		if (edge_signature.find(it->first) == edge_signature.end())printf("Oop1, edge %d not exist, have %d object\n", it->first, it->second);
		else
			if (edge_signature[it->first] != edge_check[it->first])printf("Oop2, %d should be %d\n", edge_signature[it->first], edge_check[it->first]);
	}*/
}
void cache::reSet()
{
	objects.clear();
	roadSegments.clear();
	last_query_tra = trajectory();
	size_in_range = 0;
	//RDsize = 0;
}
/************************************************************************/
/* query_msg                                                                     */
/************************************************************************/
class Grid;

query_msg::query_msg( int vid, int timestamp, int kNum, double _endtime, trajectory cur_trajectory, msg_type msg_Type)
{
	query_id = vid * SIMULATION_time  + timestamp;
	endtime = _endtime;
	query_trajectory = cur_trajectory;
	query_type = msg_Type;
	k_q = kNum;
	//cache_range = (500 + rand() % 500) * 1.0;//500-1000m
	cache_range = Grid::RANGE * 1.0;
	//RD_size = 0;
	//printf("%d %lf\n", query_id, cache_range);
	broadcast = false;
}

query_msg::query_msg()
{
}

query_msg::query_msg( int vid, int timestamp, int kNum, double _range, map<int, vector<pair<double, double> > > _roadSegments, double _endtime, trajectory cur_trajectory, msg_type msg_Type, double _cache_range)
{
	query_id = vid * SIMULATION_time + timestamp;
	range = _range;
	endtime = _endtime;
	query_trajectory = cur_trajectory;
	query_type = msg_Type;
	roadSegments = _roadSegments;
	k_q = kNum;
	cache_range = _cache_range;
	broadcast = false;
	//RD_size = 0;
	//for (int i = 0; i < _roadSegments.size(); i ++)
	//{
	//	RD_size += _roadSegments[i].size() * 16;
	//}
}

int query_msg::getDestinationVehicleId()
{
	return (query_id / SIMULATION_time);
}

int query_msg::getTimeStamp()
{
	return (query_id % SIMULATION_time) ;
}

trajectory query_msg::getQueryTrajectory()
{
	return query_trajectory;
}

loc query_msg::getQueryLoction()
{
	return query_trajectory.location;
}

int query_msg::getObjectsNum()
{
	return (int)objectlist.size();
}


bool query_msg::isKnnQueryFinished()
{
	return k_q <= getObjectsNum();//6.3
}
double query_msg::CoverageRatio()
{
	set<int> cnt;
	queue<int> Q;
	double coverage = 0;
	double total = 0;
	Q.push(getQueryTrajectory().eid);
	cnt.insert(getQueryTrajectory().eid);
	while(!Q.empty()){
		int top = Q.front();
		Q.pop();
		vector<pair<double, double> > coveredSegment = Grid::getCoveredRoadSegment(getQueryTrajectory(), top, range);
		if(coveredSegment.size() == 0)continue;
		if(roadSegments.find(top) == roadSegments.end()){
			for(int j = 0; j < coveredSegment.size(); j++){
				total += coveredSegment[j].second - coveredSegment[j].first;
			}
		}
		else{
			vector<pair<double, double> > segments = roadSegments[top];
			for(int j = 0; j < coveredSegment.size(); j ++){
				total += coveredSegment[j].second - coveredSegment[j].first;
				for(int i = 0; i < segments.size(); i ++){
					if (MathFun::D(coveredSegment[j].second - segments[i].first) <= 0 || MathFun::D(coveredSegment[j].first - segments[i].second) >= 0)continue;
					else
						coverage += MIN(segments[i].second, coveredSegment[j].second) - MAX(segments[i].first, coveredSegment[j].first);
				}
			}
		}
		edge tmpEdge = (Grid::edgemap)[top];
		vector<pair<int, int> > *tmpEdgepair = &((*Grid::nodeToEdgemap)[tmpEdge.startnid]);
		for(int i = 0; i < tmpEdgepair->size(); i++){
			if(cnt.find((*tmpEdgepair)[i].first) == cnt.end()){
				cnt.insert((*tmpEdgepair)[i].first);
				Q.push((*tmpEdgepair)[i].first);
			}
		}
		vector<pair<int, int> > *tmpEdgepair1 = &((*Grid::nodeToEdgemap)[tmpEdge.endnid]);
		for(int i = 0; i < tmpEdgepair1->size(); i++){
			if(cnt.find((*tmpEdgepair1)[i].first) == cnt.end()){
				cnt.insert((*tmpEdgepair1)[i].first);
				Q.push((*tmpEdgepair1)[i].first);
			}
		}
	}
	//printf("%lf %lf %lf\n", coverage, total, coverage / total);
	return coverage / total;
}
bool query_msg::isCoverageSatisfy()
{
	set<int> cnt;
	queue<int> Q;
	double coverage = 0;
	double total = 0;
	Q.push(getQueryTrajectory().eid);
	cnt.insert(getQueryTrajectory().eid);
	while(!Q.empty()){
		int top = Q.front();
		Q.pop();
		vector<pair<double, double> > coveredSegment = Grid::getCoveredRoadSegment(getQueryTrajectory(), top, range);
		if(coveredSegment.size() == 0)continue;
		if(roadSegments.find(top) == roadSegments.end()){
			for(int j = 0; j < coveredSegment.size(); j++){
				total += coveredSegment[j].second - coveredSegment[j].first;
			}
			//printf("lack edge %d\n", top);
		}
		else{
			vector<pair<double, double> > segments = roadSegments[top];
			for(int j = 0; j < coveredSegment.size(); j ++){
				total += coveredSegment[j].second - coveredSegment[j].first;
				for(int i = 0; i < segments.size(); i ++){
					if (MathFun::D(coveredSegment[j].second - segments[i].first) <= 0 || MathFun::D(coveredSegment[j].first - segments[i].second) >= 0)continue;
					else
						coverage += MIN(segments[i].second, coveredSegment[j].second) - MAX(segments[i].first, coveredSegment[j].first);
				}
			}
		}
		edge tmpEdge = (Grid::edgemap)[top];
		vector<pair<int, int> > *tmpEdgepair = &((*Grid::nodeToEdgemap)[tmpEdge.startnid]);
		for(int i = 0; i < tmpEdgepair->size(); i++){
			if(cnt.find((*tmpEdgepair)[i].first) == cnt.end()){
				cnt.insert((*tmpEdgepair)[i].first);
				Q.push((*tmpEdgepair)[i].first);
			}
		}
		vector<pair<int, int> > *tmpEdgepair1 = &((*Grid::nodeToEdgemap)[tmpEdge.endnid]);
		for(int i = 0; i < tmpEdgepair1->size(); i++){
			if(cnt.find((*tmpEdgepair1)[i].first) == cnt.end()){
				cnt.insert((*tmpEdgepair1)[i].first);
				Q.push((*tmpEdgepair1)[i].first);
			}
		}
	}
	//printf("%lf %lf %lf\n", coverage, total, coverage / total);
	//if (MathFun::D(coverage / total - 1.0) >= 0)return true;
	if (MathFun::D(coverage / total - Grid::COVERAGE) >= 0)return true;
	else
		return false;
}
bool query_msg::isRangeQueryFinished()
{
	set<int> cnt;
	queue<int> Q;
	Q.push(getQueryTrajectory().eid);
	cnt.insert(getQueryTrajectory().eid);
	while(!Q.empty()){
		int top = Q.front();
		Q.pop();
		vector<pair<double, double> > coveredSegment = Grid::getCoveredRoadSegment(getQueryTrajectory(), top, range);
		if(coveredSegment.size() == 0)continue;
		if(roadSegments.find(top) == roadSegments.end()){
			return false;
		}
		vector<pair<double, double> > segments = roadSegments[top];
		for(int j = 0; j < coveredSegment.size(); j++){
			bool found = false;
			for(int i = 0; i < segments.size(); i++){
				if(MathFun::D(segments[i].first - coveredSegment[j].first) <= 0 && MathFun::D(segments[i].second - coveredSegment[j].second) >= 0){
					found = true;
					break;
				}
			}
			if(!found){
				return false;
			}
		}
		edge tmpEdge = (Grid::edgemap)[top];
		vector<pair<int, int> > *tmpEdgepair = &((*Grid::nodeToEdgemap)[tmpEdge.startnid]);
		for(int i = 0; i < tmpEdgepair->size(); i++){
			if(cnt.find((*tmpEdgepair)[i].first) == cnt.end()){
				cnt.insert((*tmpEdgepair)[i].first);
				Q.push((*tmpEdgepair)[i].first);
			}
		}
		tmpEdgepair = &((*Grid::nodeToEdgemap)[tmpEdge.endnid]);
		for(int i = 0; i < tmpEdgepair->size(); i++){
			if(cnt.find((*tmpEdgepair)[i].first) == cnt.end()){
				cnt.insert((*tmpEdgepair)[i].first);
				Q.push((*tmpEdgepair)[i].first);
			}
		}
	}
	//cout << "range is :" << range << endl;
	return true;
}

void query_msg::insertNewObject( node newObject )
{
	objectlist[newObject.nid] = newObject;
}

bool query_msg::isIdExisted( int id )
{
	return objectlist.find(id) != objectlist.end();
}

void query_msg::updateRoadSegments( int eid, vector<pair<double, double> > segments )
{

	vector<pair<double, double> > newSegments;
	if(roadSegments.find(eid) == roadSegments.end()){
		roadSegments[eid] = segments;
		return;
	}else{
		newSegments = roadSegments[eid];
		for(vector<pair<double, double> >::iterator iter = newSegments.begin(); iter != newSegments.end(); iter ++){
			segments.push_back(*iter);
		}
		newSegments.clear();
	}
	sort(segments.begin(), segments.end());
	pair<double, double> curSegment;
	for(vector<pair<double, double> >::iterator iter = segments.begin(); iter != segments.end(); iter ++){
		if(iter == segments.begin()){
			curSegment = *iter;
		}else{
			if(MathFun::D(curSegment.second - iter->first) >= 0){
				curSegment.second = max(curSegment.second, iter->second);
			}else{
				newSegments.push_back(curSegment);
				curSegment = *iter;
			}
		}
	}
	newSegments.push_back(curSegment);
	roadSegments[eid] = newSegments;
}

/************************************************************************/
/* vehicle                                                                     */
/************************************************************************/

double vehicle:: Compute_Edge_Delay_test( int eid, int nid, Grid& mainGrid, double l, int cur_time, int oid)
{
	double v = Grid::SPEED / 3.6;
	double c = 0.00763;
	double R = Grid::COMRANGE;
	double delay = 0;
	double forwarding_dist = 0;
	double carry_distance = 0;
	double roadlen = Grid::getRoadNetworkDist(nid, oid);
	//if(l < Grid::COMRANGE)return l /R * c;

	if (MathFun::D(roadlen - l) == 0){
		double len1 = Compute_ACL_length(eid, nid, mainGrid, l, cur_time);
		double len2 = Compute_ACL_length(eid, oid, mainGrid, l, cur_time);
		//if (vid == 27)printf("+ %lf %lf\n", len1, len2);
		forwarding_dist = MIN(len1 + len2, l); // 0 <= ACL <= l
		if(forwarding_dist < 0){
			cout << "forwarding_dist < 0" << endl;
			return 0.0;
		}
		carry_distance = l - forwarding_dist;
		delay = carry_distance / v + forwarding_dist / R * c;
		//if(l / v <= delay)printf("%lf %lf %lf %lf %lf\n", forwarding_dist, carry_distance, l, delay, l / v);
	}
	else{
		if (l < Grid::COMRANGE){
			//if (vid == 27)printf("$ %lf %lf %lf\n", l, roadlen, l / R * c);
			return l / R * c;
		}
		double len1 = Compute_ACL_length(eid, nid, mainGrid, roadlen, cur_time);
		double len2 = Compute_ACL_length(eid, oid, mainGrid, roadlen, cur_time);
		//if (vid == 27)printf("$ %lf %lf\n", len1, len2);
		forwarding_dist = MIN( len1 + MAX(len2 - roadlen + l, 0), l);
		if(forwarding_dist < 0){
			cout << "forwarding_dist < 0" << endl;
			return 0.0;
		}
		carry_distance = l - forwarding_dist;
		delay = carry_distance / v + forwarding_dist / R * c;
	}
	//if (vid == 27)printf("$ %lf %lf %lf %lf\n", l, forwarding_dist, carry_distance, delay);
	return delay;

}
double vehicle:: Compute_ACL_length( int eid, int nid, Grid& mainGrid, double l, int cur_time)
{ //compute the edge delay for the road segment r_ij with head node pGraphNode->vertex
  double lambda = 0; //vehicle arrival rate per second
  double v = Grid::SPEED / 3.6; //average vehicle speed on road segment r_ij
  double R = Grid::COMRANGE; //radio communication range
  double ACL = 0; //Average Convoy Length (ACL) used as forwarding distance
  double a = R/v; //movement time within the communication range R with the speed v
  double alpha = 0; //alpha for ACL computation
  double beta = 0; //beta for ACL computation
  double N = 0; //index N for ACL computation
  double B = 0; //coefficient
  double mean_interarrival_time;
  /* check the vehicle arrival for the directional edge:
     Note that for the directional edge with no vehicle arrival, the edge's mean link delay is 
     the edge's length divided by mean vehicle speed and the link delay standard deviation is zero.*/

  //if(nodemap[nid].number_of_interarrivals == 0){
	 // //printf("no interarrivals\n");
	 // return ACL;
  //}
  if (edgemap[eid].startnid == nid){
	  int tsid = eid * NODE_NUM + edgemap[eid].startnid;
	  if (mainGrid.traffic[tsid] != 0)
	  {
		  /* compute the mean interarrival time for the road segment whose head node is pGraphNode */
		  mean_interarrival_time = 1 / mainGrid.traffic[tsid];
		  //printf("%lf %d %lf\n", cur_time - nodemap[nid].sum_of_interarrival_time, nodemap[nid]. number_of_interarrivals, 1 / mean_interarrival_time);

		  /** compute Average Convoy Length (ACL) for the directional edge where the vehicle is moving */
		  lambda = 1 / mean_interarrival_time;
		  alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));
		  beta = 1 - exp(-1*lambda*a);
		  N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected convoy length is greater than l

		  B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
		  if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
			  B = 0; //Note that B may be a extremely small number, sduch as 1*10^(-3). In this case, we regard B as zero.

		  ACL = alpha/pow(1-beta,2)*B + l*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
		  ACL = MIN(ACL, l); // 0 <= ACL <= l
		  if(ACL < 0)
		  {
			  cout << "ACL < 0" << endl;
			  return 0.0;
		  }
		  /* compute edge delay based on TBD edge delay model */
		  //if(Grid::PLAMBDA){
			 // //printf("%lf\n", lambda);
		  //}  
	  }
  }
  else{
  		int teid = eid * NODE_NUM + edgemap[eid].endnid;
	  if(mainGrid.traffic[teid] != 0){
		  /* compute the mean interarrival time for the road segment whose head node is pGraphNode */
		  mean_interarrival_time = 1 / mainGrid.traffic[teid];
		  //printf("%lf %d %lf\n", cur_time - nodemap[nid].sum_of_interarrival_time, nodemap[nid]. number_of_interarrivals, 1 / mean_interarrival_time);

		  /** compute Average Convoy Length (ACL) for the directional edge where the vehicle is moving */
		  lambda = 1 / mean_interarrival_time;
		  alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));
		  beta = 1 - exp(-1*lambda*a);
		  N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected convoy length is greater than l

		  B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
		  if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
			  B = 0; //Note that B may be a extremely small number, sduch as 1*10^(-3). In this case, we regard B as zero.

		  ACL = alpha/pow(1-beta,2)*B + l*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
		  ACL = MIN(ACL, l); // 0 <= ACL <= l
		  if(ACL < 0)
		  {
			  cout << "ACL < 0" << endl;
			  return 0.0;
		  }
		  /* compute edge delay based on TBD edge delay model */
		  //if(Grid::PLAMBDA){
			 // //printf("%lf\n", lambda);
		  //}  
	  }
  }
  //printf("%lf\n", ACL);
  return ACL;
}
double vehicle::GetEdgeCovered(double range, set<int>& coverEdgeId, map<int, double>& unCovered)
{
	set<int> cnt;
	queue<int> Q;
	double coverage = 0;
	double total = 0;
	Q.push(query_trajectory.eid);
	cnt.insert(query_trajectory.eid);
	while(!Q.empty()){
		int top = Q.front();
		Q.pop();
		vector<pair<double, double> > coveredSegment = Grid::getCoveredRoadSegment(query_trajectory, top, range);
		if(coveredSegment.size() == 0)continue;
		//if(vid == 21)printf("edge %d\n", top);
		double semantic = 0;
		double complete = 0;
		for (vector<pair<double, double> >::iterator iter = coveredSegment.begin(); iter != coveredSegment.end(); iter ++)
		{
			total += iter->second - iter->first;
			semantic += iter->second - iter->first;;
			//if(vid == 21)printf("%lf %lf\n", iter->first, iter->second);
		}
		if(local_cache.roadSegments.find(top) == local_cache.roadSegments.end()){
			//for (vector<pair<double, double> >::iterator iter = coveredSegment.begin(); iter != coveredSegment.end(); iter ++)
			//{
			//	total += iter->second - iter->first;
			//	mustCover += iter->second - iter->first;
			//	//if(vid == 21)printf("%lf %lf\n", iter->first, iter->second);
			//}
			unCovered[top] = semantic;
			//if(vid == 21)printf("$ %d %lf %lf %d\n", top, total, mustCover, 0);
			//printf("lack edge %d\n", top);
		}
		else{
			vector<pair<double, double> > segments = local_cache.roadSegments[top];
			for(int j = 0; j < coveredSegment.size(); j ++){
				for(int i = 0; i < segments.size(); i ++){
					if (MathFun::D(coveredSegment[j].second - segments[i].first) <= 0 || MathFun::D(coveredSegment[j].first - segments[i].second) >= 0)continue;
					else{
						double dist = MIN(segments[i].second, coveredSegment[j].second) - MAX(segments[i].first, coveredSegment[j].first);
						complete += dist;
					}
				}
			}
			//if(vid == 21)printf("@ %d %lf %lf\n", top, mustCover, complete);
			if(MathFun::D(semantic - complete) == 0){
				coverEdgeId.insert(top);
				coverage += complete;
			}
			else{
				unCovered[top] = semantic;
			}
		}
		edge tmpEdge = (Grid::edgemap)[top];
		vector<pair<int, int> > *tmpEdgepair = &((*Grid::nodeToEdgemap)[tmpEdge.startnid]);
		for(int i = 0; i < tmpEdgepair->size(); i++){
			if(cnt.find((*tmpEdgepair)[i].first) == cnt.end()){
				cnt.insert((*tmpEdgepair)[i].first);
				Q.push((*tmpEdgepair)[i].first);
			}
		}
		vector<pair<int, int> > *tmpEdgepair1 = &((*Grid::nodeToEdgemap)[tmpEdge.endnid]);
		for(int i = 0; i < tmpEdgepair1->size(); i++){
			if(cnt.find((*tmpEdgepair1)[i].first) == cnt.end()){
				cnt.insert((*tmpEdgepair1)[i].first);
				Q.push((*tmpEdgepair1)[i].first);
			}
		}
	}
	//printf("%lf %lf %lf\n", coverage, total, coverage / total);
	return total * Grid::COVERAGE - coverage;
}
double vehicle:: compute_ttl_test ( Grid& mainGrid, double range, int cur_time )
{
	if (range <= Grid::COMRANGE)return ceil((range / Grid::COMRANGE * 0.00763) * 10);
	set<int> coverEdgeId;
	map<int, double> unCovered;
	double remain = GetEdgeCovered(range, coverEdgeId, unCovered);
	/*for (map<int, double>::iterator iter = unCovered.begin(); iter != unCovered.end(); iter ++)
	{
		printf("uncover %d %lf\n", iter->first, iter->second);
	}*/
	if(remain <= 0) return 0;
	//printf("vehicle %d %lf\n", vid, remain);
	//vector <pair<double, double> >t2l;
	//priority_queue< pair<int, double> > Q;
	priority_queue< ttledge > Q;
	map<int, double> cnt;
	//set<int> mindist;
	int tempeid = getQueryTrajaectory().eid;
	int tstart = edgemap[tempeid].startnid;
	int tend = edgemap[tempeid].endnid;
	//double len = Grid::getRoadNetworkDist(tstart, tend);
	//printf("%d %d %d %lf\n", tempeid, tstart, tend, len);
	//double len = Grid::calculateDist(nodemap[edgemap[tempeid].startnid].getLoc(), nodemap[edgemap[tempeid].endnid].getLoc());
	double dist_s = Grid::calculateDist(getQueryPosition(), nodemap[tstart].getLoc());
	double dist_e = Grid::calculateDist(getQueryPosition(), nodemap[tend].getLoc());
	if(dist_s < dist_e){
		Q.push(ttledge(tstart, tempeid, edgemap[tempeid].length, 0, 0));
		//cnt.insert(tstart);
		cnt[tstart] = 0;
	}
	else{
		Q.push(ttledge(tend, tempeid, edgemap[tempeid].length, 0, 0));
		cnt[tend] = 0;
		//cnt.insert(tend);
	}
	//if(dist_s >= range){
	//	double temp = Compute_Edge_Delay_test( tempeid, tend, nodemap, edgemap, dist_e + range, cur_time, tstart) - Compute_Edge_Delay_test( tempeid, tend, nodemap, edgemap, dist_e, cur_time, tstart);
	//	t2l.push_back(temp);
	//}
	//else{
	//	double ttl_s = Compute_Edge_Delay_test( tempeid, tend, nodemap, edgemap, len, cur_time, tstart) - Compute_Edge_Delay_test( tempeid, edgemap[tempeid].endnid, nodemap, edgemap, dist_e, cur_time, tstart);
	//	//Q.push(make_pair( tstart, dist_s));
	//	Q.push(ttledge( tstart, dist_s, ttl_s));
	//}
	//if(dist_e >= range){
	//	double temp = Compute_Edge_Delay_test( tempeid, tstart, nodemap, edgemap, dist_s + range, cur_time, tend) - Compute_Edge_Delay_test( tempeid, tstart, nodemap, edgemap, dist_s, cur_time, tend);
	//	t2l.push_back(temp);
	//}
	//else{
	//	double ttl_e = Compute_Edge_Delay_test( tempeid, tstart, nodemap, edgemap, len, cur_time, tend) - Compute_Edge_Delay_test( tempeid, tstart, nodemap, edgemap, dist_s, cur_time, tend);
	//	Q.push(ttledge( tend, dist_e, ttl_e));
	//}
	double ansTime = 0;
	set<int> test_set;
	set<int> test_edge;
	int eNum = (int)unCovered.size();
	while (!Q.empty())
	{
		int id = Q.top().snid;
		int edgeId = Q.top().eid;
		double length = Q.top().length;
		double time = Q.top().ttl;
		double tEdge = Q.top().tEdge;
		//if(vid == 21)printf("%d %lf %lf %lf %d\n", id, length, time, weight, num);
		Q.pop();
		if(test_set.find(id) == test_set.end()){
			test_set.insert(id);
		}
		if(test_edge.find(edgeId) == test_edge.end()){
			test_edge.insert(edgeId);
		}
		/*if (mindist.find(id) != mindist.end())continue;
		else
			mindist.insert(id);*/
		if (coverEdgeId.find(edgeId) == coverEdgeId.end())
		{
			//if(vid == 21)printf("%lf %lf %lf\n", remain, length, time);
			if(unCovered.find(edgeId) != unCovered.end()){
				eNum --;
				//if(vid == 21)printf("%lf %lf %lf\n", remain, length, time);
				remain -= unCovered[edgeId];
				double dist1 = Grid::caculateRoadNetworkDist(getQueryTrajaectory(), trajectory(edgeId, nodemap[edgemap[edgeId].startnid].getLoc()));
				double dist2 = Grid::caculateRoadNetworkDist(getQueryTrajaectory(), trajectory(edgeId, nodemap[edgemap[edgeId].endnid].getLoc()));
				//printf("%lf %lf %lf\n", dist1, dist2, range);
				if((dist1 >= range && dist2 < range) ||(dist1 < range && dist2 >= range))
					time -= (length - unCovered[edgeId]) / length * tEdge;
				ansTime = max(ansTime, ceil(time * 10));
				if(MathFun::D(remain) <= 0){
					return ceil(time * 10);
				}
				//ansTime_test = time * 10;
				coverEdgeId.insert(edgeId);
				unCovered.erase(edgeId);
			}
		}
		vector<pair<int, int> > *tmpEdgepair = &((*Grid::nodeToEdgemap)[id]);
		for(int i = 0; i < tmpEdgepair->size(); i++){
			double tmp = edgemap[(*tmpEdgepair)[i].first].length;
			double T1 = Compute_Edge_Delay_test((*tmpEdgepair)[i].first, id, mainGrid, tmp, cur_time, (*tmpEdgepair)[i].second);
			double T2 = Compute_Edge_Delay_test((*tmpEdgepair)[i].first, (*tmpEdgepair)[i].second, mainGrid, tmp, cur_time, id);
			double tTime = time + T1 + T2;

			if(cnt.find((*tmpEdgepair)[i].second) != cnt.end()){
				if (MathFun::D(cnt[(*tmpEdgepair)[i].second] -  tTime) <= 0){
					//if(coverEdgeId.find((*tmpEdgepair)[i].first) != coverEdgeId.end())continue;
					if(unCovered.find((*tmpEdgepair)[i].first) == unCovered.end())continue;
				}
				else
					cnt[(*tmpEdgepair)[i].second] = tTime;
			}
			else
				cnt[(*tmpEdgepair)[i].second] = tTime;
			Q.push(ttledge((*tmpEdgepair)[i].second, (*tmpEdgepair)[i].first, tmp, tTime, T1 + T2));
		}
	}
	//cout << "setsize : " << test_set.size() << ' ' << eNum <<  ' '<<  test_edge.size() <<endl;
	return ansTime;
}
loc vehicle::getCurrentPosition()
{
	return trajectorys.location; // load changes at 5.30
}

loc vehicle::getQueryPosition()
{
	return query_trajectory.location;
}

void vehicle::setQueryPosition()
{
	query_trajectory = getCurrentTrajectory();
	query_state |= 1;
}

vehicle::vehicle( int _vid )
{
	vid = _vid;
	last_trajectory = trajectory();
	label = 0;
	exBegin = false;
}

vehicle::vehicle()
{
	last_trajectory = trajectory();
	label = 0;
	exBegin = false;
}

void vehicle::updateRoadsFromRoads(vector<pair<double, double> > segments, vector<pair<double, double> >& roads)
{
	vector<pair<double, double> > newSegments;
	for(vector<pair<double, double> >::iterator iter = segments.begin(); iter != segments.end(); iter ++){
		roads.push_back(*iter);
	}
	sort(roads.begin(), roads.end());
	pair<double, double> curSegment;
	for(vector<pair<double, double> >::iterator iter = roads.begin(); iter != roads.end(); iter ++){
		if(iter == roads.begin()){
			curSegment = *iter;
		}else{
			if(MathFun::D(curSegment.second - iter->first) >= 0){
				curSegment.second = max(curSegment.second, iter->second);
			}else{
				newSegments.push_back(curSegment);
				curSegment = *iter;
			}
		}
	}
	newSegments.push_back(curSegment);
	roads.swap(newSegments);
	return;
}
void vehicle::expandCluster(vector<Point*>& D, Point* P, int C, vector<Point*> *NeighborPts, int eps, int MinPts, vector<Cluster*>& clusters) {
    Cluster* cluster = new Cluster(C);
    cluster -> push(P);
    for (int i = 0; i < NeighborPts -> size(); i ++) {
        Point* tP = (*NeighborPts)[i];
        if (!tP -> visited) {
            tP -> visited = true;
            vector<Point*>* tNeighborPts = regionQuery(D, tP, eps);
            if (tNeighborPts -> size() >= MinPts) {
                for (vector<Point*> :: iterator tP2 = tNeighborPts -> begin(); tP2 != tNeighborPts -> end(); tP2 ++) {
                    NeighborPts -> push_back(*tP2);
                }
            }
            delete tNeighborPts;
        }
        if (tP -> label == -1) {
            cluster -> push(tP);
        }
    }
    clusters.push_back(cluster);
}
void vehicle::DBSCAN(int eps, int MinPts, vector<Cluster*>& clusters) {
    vector< Point* > D;
	set<int> queryLoc;
	for(map<int, query_msg>::iterator iter = query_msg_queue.begin(); iter != query_msg_queue.end(); iter ++){
		double lat = iter->second.getQueryLoction().lat;
		double lng = iter->second.getQueryLoction().lng;
		D.push_back(new Point(iter->first, lat, lng, -1, 0));
	}
    int C = 0;
    for (int i = 0; i < D.size(); i ++) {
        Point* P = D[i];
        if (P -> visited) continue;
        P -> visited = true;
        vector<Point*>* NeighborPts = regionQuery(D, P, eps);
        if (NeighborPts -> size() < MinPts) {
            P -> label = -1;
        } else {
            C ++;
            expandCluster(D, P, C, NeighborPts, eps, MinPts, clusters);
        }
        delete NeighborPts;
    }
	return;
}
vector<Point*>* vehicle::regionQuery(vector<Point*>& D, Point* P, int eps) {
    vector<Point*>* NeighborPts = new vector<Point*>();
    for (int i = 0; i < D.size(); i ++) {
        Point* tP = D[i];
		double dist = Grid::calculateDist(tP -> x, tP -> y, P -> x, P -> y);
        if (dist <= eps) {
            NeighborPts -> push_back(tP);
        }
    }
    return NeighborPts;
}
void vehicle::cleanCluster(vector<Cluster*>& clusters) {
	for (int i = 0; i < clusters.size(); i ++) {
		delete clusters[i];
	}
}
double vehicle::cacheShareCost(int eid, trajectory current)
{
	vector<Cluster *> clusters;
	map<int, loc> centers;
	map<int, double> search_range;
	map<int, int> ans;
	DBSCAN(DBDIST, 1, clusters);
    for (int i = 0; i < (int)clusters.size(); i ++) {
        //clusters[i] -> print();
		double range = 0;
		int cnt = 0;
		double xlabel = 0;
		double ylabel = 0;
		for(vector<Point*>::iterator iter = clusters[i]->points.begin(); iter != clusters[i]->points.end(); iter ++)
		{
			xlabel += (*iter)->x;
			ylabel += (*iter)->y; 
			/*if(query_msg_queue[(*iter)->id].query_type == RANGE_QUERY)
			{
				range += query_msg_queue[(*iter)->id].range;
				cnt ++;
			}*/
			if(query_msg_queue[(*iter)->id].query_type == RANGE_QUERY)range += query_msg_queue[(*iter)->id].range;
			else
				range += query_msg_queue[(*iter)->id].cache_range;
		}
		ans[clusters[i]->label] = (int)clusters[i]->points.size();
		centers[clusters[i]->label] = loc(xlabel / ans[clusters[i]->label], ylabel / ans[clusters[i]->label]);
		/*if(cnt != 0)search_range[clusters[i]->label] = range / cnt;
		else
			search_range[clusters[i]->label] = Grid::RANGE * 1.0;*/
		search_range[clusters[i]->label] = range / ans[clusters[i]->label];
		
    }
	/*for(map<int, loc>::iterator iter = centers.begin(); iter != centers.end(); iter ++){
		if(MathFun::D(search_range[iter->first] - Grid::RANGE) != 0)
			printf("cluster %d range %lf\n", iter->first, search_range[iter->first]);
	}*/

	double priority = 0;
	//pair<int, int> curGrid = Grid::getLocFromGeo(current.location);
	for(map<int, loc>::iterator iter = centers.begin(); iter != centers.end(); iter ++)
	{
		double dist = 0;
		loc startLoc = nodemap[edgemap[eid].startnid].getLoc();
		double d_start = Grid::calculateDist(startLoc, iter->second);
		loc endLoc = nodemap[edgemap[eid].endnid].getLoc();
		double d_end = Grid::calculateDist(endLoc, iter->second);
		if(MathFun::D(d_start - d_end) < 0)dist = d_start;
		else
			dist = d_end;
		double score = search_range[iter->first] / dist;

		/*pair<int, int> centerGrid= Grid::getLocFromGeo(iter->second);
		int Manhattan = abs(curGrid.first - centerGrid.first) + abs(curGrid.second - centerGrid.second);
		double priority = pow(ALPHA, Manhattan);
		priority += score * priority;*/
		priority += score * ans[iter->first] / (int)query_msg_queue.size();
	}
	cleanCluster(clusters);
	//double distCost = cacheShareDistCost(eid, current);
	//return priority / distCost;
	return priority;
}
double vehicle::cacheShareDistCost(int eid, trajectory current)
{
	double d = RoadDistCal;
	int start = edgemap[eid].startnid;
	int end = edgemap[eid].endnid;
	int nids[2]= {start, end};
	for (int i = 0; i < 2; i ++){
		double dist = Grid::caculateRoadNetworkDist(current, nids[i]);
		d = MIN(d, dist);
	}
	return d;
}

int vehicle::prepareObjFromSig(bitset<BITNUM> m, int vehicleId, trajectory current, obj_map& objects, map<int, vector<pair<double, double> > >& roadSegments, double tConnect)
{
	int roadbits = HEADER;
	int SizeTrans = floor(tConnect * TRANSBIT);
	if(roadbits > SizeTrans) return 0;
	cs_time ++;
	vector<pair<double, int> >edgeIds;
	map<int, vector<node> >objIds;
	for (tr1::unordered_map<int, node>::iterator iter = local_cache.objects.begin(); iter != local_cache.objects.end(); iter ++){
		bitset<BITNUM> n;
		makeCacheSig(n, iter->first);
		if((n & m) == n)continue;
		else{
			int teid = iter->second.eid;
			objIds[teid].push_back(iter->second);
		}
	}
	//printf("vid: %d\n", vid);
	for (map<int, vector<pair<double, double> > >::iterator it = local_cache.roadSegments.begin(); it != local_cache.roadSegments.end(); it ++)
	{
		bitset<BITNUM>n;
		makeCacheSig(n, it->first + NODE_NUM);
		//collect statistics to calculate search size
		if((n & m) == n){
			if(search.find(it->first) == search.end()){
				search[it->first] = 1;
			}
			else
				search[it->first] ++;
			//continue;
		}
		//double avg = cacheShareCost(it->first);
		//printf("%d avgSize %lf\n", it->first, avg);
		//if (local_cache.checkDistOfEdge(it->first, current, Grid::RANGE, nodemap, edgemap) == false)continue;
		//double Dscore = cacheShareDistCost(it->first, current);
		//if(MathFun::D(Dscore - RoadDistCal) == 0)continue;
		//double Qscore = cacheShareCost(it->first, current);
		double distScore = cacheShareDistCost(it->first, current);
		//printf("%d Qscore %lf\n", it->first, Qscore);
		/*double d = RoadDistCal;
		int sid = nodemap[edgemap[it->first].startnid].nid;
		int eid = nodemap[edgemap[it->first].endnid].nid;
		int nids[2]= {sid, eid};
		for (int i = 0; i < 2; i ++){
			double dist = Grid::caculateRoadNetworkDist(current, nids[i]);
			d = MIN(d, dist);
		}*/
		edgeIds.push_back(make_pair(distScore, it->first));
		//edgeIds.push_back(make_pair(Qscore, it->first));
	}
	//sort(edgeIds.begin(), edgeIds.end(), peerSort());
	sort(edgeIds.begin(), edgeIds.end(), rsuSort());
	int cnt = 0;
	int edge_num = ceil ((int)edgeIds.size() * BETA);//only transmit BETA edges
	
	for (vector<pair<double, int> >::iterator iter = edgeIds.begin(); iter != edgeIds.end(); iter ++)
	{
		//printf("%d %lf %lf\n", iter->second, iter->first, cacheShareDistCost(iter->second, current));
		int eid = iter->second;
		if(objIds.find(eid) == objIds.end()){

			int tsize = (int)local_cache.roadSegments[eid].size() * SEMANIC + EID;
			roadbits += tsize;
			if(roadSegments.find(eid) == roadSegments.end()){
				roadSegments[eid] = local_cache.roadSegments[eid];
			}
			else{
				updateRoadsFromRoads(local_cache.roadSegments[eid], roadSegments[eid]);
			}
		}
		else{
			int tsize = OBJSIZEBIT * (int)objIds[eid].size() + (int)local_cache.roadSegments[eid].size() * SEMANIC + EID;
			roadbits += tsize;
			for (vector<node>::iterator i = objIds[eid].begin(); i != objIds[eid].end(); i ++){
				if(objects.find(i->nid) == objects.end()){
					objects[i->nid] = *i;
				}
				else{
					if(i->time_to_live > objects[i->nid].time_to_live)objects[i->nid].time_to_live = i->time_to_live;
				}
			}
			if(roadSegments.find(eid) == roadSegments.end())roadSegments[eid] = local_cache.roadSegments[eid];
			else{
				updateRoadsFromRoads(local_cache.roadSegments[eid], roadSegments[eid]);
			}
		}
		cnt ++;
		if (cnt > edge_num) break;
	}
	return roadbits;
}
void vehicle::makeCacheSig(bitset<BITNUM>& m, int id)
{
	/*int k[4] = {3301, 3469, 3943, 4451};
	id = (id + 1) * 3517;
	for(int i = 0; i < 4; i ++)
	{
		int tmp = id % k[i] % BITNUM;
		m[tmp] = 1;
	}*/
	int k[4] = {6947, 6977, 7759, 7829};
	id = (id + 1) * 8681;
	for(int i = 0; i < 4; i ++)
	{
		int tmp = id % k[i] % BITNUM;
		m[tmp] = 1;
	}
}

int vehicle::encrypGrid()
{
	int num = 0;
	for (vector<pair<int, int> >::iterator iter = last_grid.begin(); iter != last_grid.end(); iter ++)
	{
		num += iter->first + iter->second;
	}
	return num;
}
double vehicle::selectPeers(double _vx, double _vy, loc _pos)
{
	double t = 0;
	double x = getCurrentPosition().lat;
	double y = getCurrentPosition().lng;
	//printf("%lf %lf %lf %lf\n", x, y, last_trajectory.location.lat, last_trajectory.location.lng);
	double vx = (x - last_trajectory.location.lat) / 0.1;//0.1s time step
	double vy = (y - last_trajectory.location.lng) / 0.1;
	//printf("%lf %lf %lf %lf %lf %lf\n", _vx, _vy, vx, vy, _vx * _vx + _vy * _vy, vx * vx + vy * vy);
	double A = _vx - vx;
	double B = _pos.lat - x;
	double C = _vy - vy;
	double D = _pos.lng - y;
	//printf("* %lf %lf %lf %lf\n", A, B, C, D);
	double a = A * A + C * C;
	double b = 2 * (A * B + C * D);
	double c = B * B + D * D - Grid::COMRANGE * Grid::COMRANGE;
	if(MathFun::D(c) > 0)return t;
	//printf("%lf %lf %lf\n", a, b, c);
	if(MathFun::D(a) == 0){
		t = Grid::PERIOD;
	}
	else{
		double delat = b * b - 4 * a * c;
		if(MathFun::D(delat) > 0){
			double sqrt_delat = sqrt(delat);
			double tmp = a + a;
			double t1 = (-1 * b + sqrt_delat) / tmp;
			double t2 = (-1 * b - sqrt_delat) / tmp;
			t = MAX(t1, t2);
			t = MIN(t, Grid::PERIOD);
			//if(MathFun::D(c / a) > 0){
			//	printf("Oops %lf %lf %lf\n", t1, t2, c);
			//	//printf("%lf %lf %lf %lf %lf %lf\n", _vx, _vy, vx, vy, _vx * _vx + _vy * _vy, vx * vx + vy * vy);
			//}
		}
	/*	else
			printf("%lf\n", delat);*/
	}
	return t;
}

void vehicle::exchangeInfo(overhead& ex_statistic, int s_time)
{
	obj_map objects;
	map<int, vector<pair<double, double> > > roadSegments;
	//map<int, vector<double> > qScore;
	int n = 0;//neigbour number
	bitset<BITNUM> m;
	for (tr1::unordered_map<int, node>::iterator iter = local_cache.objects.begin(); iter != local_cache.objects.end(); iter ++){
		makeCacheSig(m, iter->first);
		//if(vid == 2131)printf("%d\n", iter->first);
	}
	for(map<int, vector<pair<double, double> > >::iterator iter = local_cache.roadSegments.begin(); iter != local_cache.roadSegments.end() ;iter ++){
		//double curLength = iter->second[0].second - iter->second[0].first;
		//printf("WOW%lf %lf\n", curLength, edgemap[iter->first].length);
		//if(MathFun::D(curLength - edgemap[iter->first].length) == 0){
			/*printf("COM%d\n", iter->first);
			if(Grid::objectsOnEdgeMap.find(iter->first) != Grid::objectsOnEdgeMap.end()){
				vector<int> ids = Grid::objectsOnEdgeMap[iter->first];
				printf("XD%d\n", (int)ids.size());
				for(int i = 0; i < ids.size(); i++){
					if(local_cache.objects.find(ids[i]) == local_cache.objects.end())
						printf("Oops, edge not complete %lf %lf\n", curLength, edgemap[iter->first].length);
				}
			}*/
			makeCacheSig(m, iter->first + NODE_NUM);
		//}
	}
	int temp = BITNUM + HEADER + REQ;
	loc curLoction = getCurrentPosition();
	loc leftdown = loc( curLoction.lat - delat_lat, curLoction.lng - delat_lng );
	loc rightup = loc( curLoction.lat + delat_lat, curLoction.lng + delat_lng );
	pair<int, int> pos = Grid::getLocFromGeo(curLoction);
	vector<pair<double, int> > peers;
	for(int i = -1; i <= 1; i++){
		for(int j = -1; j <= 1; j++){
			int x = pos.first + i, y = pos.second + j;
			if(!Grid::is_Inrange(x, y))continue;
			loc tleftdown = loc( x * grid_lat + Grid::trans_low.lat, y * grid_lng + Grid::trans_low.lng );
			loc trightup = loc( (x + 1) * grid_lat + Grid::trans_low.lat, (y + 1) * grid_lng + Grid::trans_low.lng );
			if (( leftdown.lng < trightup.lng )&&( rightup.lng > tleftdown.lng )&&( leftdown.lat < trightup.lat )&&( rightup.lat > tleftdown.lat ))
			{
				//set<int> ids = grid->getAllVehicleInCell(x, y);
				set<int>* ids;
				grid->getAllVehicleInCell(x, y, ids);
				for(set<int>::iterator iter = ids->begin(); iter != ids->end(); iter ++){
					int id = *iter;
					if(id == getVehicleId())continue;
					vehicle* tu = &(vehiclemap[id]);
					double mindist = Grid::calculateDist(getCurrentPosition(), tu->getCurrentPosition());
					if(MathFun::D(mindist - Grid::COMRANGE) <= 0){
						double vx = (getCurrentPosition().lat - last_trajectory.location.lat) / 0.1;//0.1s time step
						double vy = (getCurrentPosition().lng - last_trajectory.location.lng) / 0.1;
						double t = tu->selectPeers(vx, vy, getCurrentPosition());
						peers.push_back(make_pair(t, id));
						ex_statistic.msg_size += REP;
						ex_statistic.number ++;
					}
				}
			}
		}
	}
	sort(peers.begin(), peers.end(), peerSort());
	//printf("%d %d\n", n, (int)peers.size());
	//printf("# %d\n", vid);
	ex_statistic.msg_size += HEADER + Grid::TOP * 12;//broadcast the list
	for (vector<pair<double, int> >::iterator it = peers.begin(); it != peers.end(); it ++)
	{
		if(csId.find(it->second) == csId.end())csId.insert(it->second);
		else
			continue;
		vehicle* tu = &(vehiclemap[it->second]);
		int msgsize = tu->prepareObjFromSig(m, vid, getCurrentTrajectory(), objects, roadSegments, it->first);
		//printf("%d %d %lf\n", it->second, msgsize, it->first);
		ex_statistic.power += 0.5 * temp / 8 + 56;//receive request
		ex_statistic.msg_size += msgsize;
		ex_statistic.power += (1.9 * msgsize + 454) + (0.5 * msgsize + 356);//return message
		ex_statistic.number ++;
		n ++;
		if(n == Grid::TOP)break;
	}
	if(n > 0){
		//printf("%d %lf %d %d\n", encry, 1.0 * grid_sta / n, grid_sta, n);
		ex_statistic.number += 2;
		ex_statistic.msg_size += temp;
		ex_statistic.power += 1.9 * temp / 8 + 266;//broadcast request
		ex_statistic.transtime += 1.0 * temp / TRANSBIT;
		updateObjFromVeh(objects, roadSegments);
	}
	/*int t = 0;
	for (map<int, vector<pair<double, double> > >::iterator iter = roadSegments.begin(); iter != roadSegments.end(); iter ++)
	{
	t += (int)iter->second.size() * SEMANIC + EID;
	}*/
	//printf("%d %d %d\n", vid, (int)objects.size(), t);
	//updateObjFromVeh(objects, roadSegments);
	predict = s_time + floor(Grid::PERIOD * 10);
	//if(vid == 11)printf("%d neighbor number  %d object number %d\n",vid, n, (int)local_cache.objects.size());
}

void vehicle::updateObjFromVeh(obj_map objects, map<int, vector<pair<double, double> > > roadSegments)
{
	//map<int, node> ids = msg.objectlist;// can be faster
	if((int)roadSegments.size() + local_cache.roadSegments.size() <= EGSIZE){
		for (tr1::unordered_map<int, node>::iterator iter = objects.begin(); iter != objects.end(); iter ++){
			if(local_cache.isIdExisted(iter->first) == false){
				local_cache.insertNewNode(iter->second);
				local_cache.updateEdgesig(iter->second.eid);
			}
			else{
				if (local_cache.objects[iter->first].time_to_live < objects[iter->first].time_to_live){
					local_cache.objects[iter->first].time_to_live = objects[iter->first].time_to_live;
				}
			}
		}
		for(map<int, vector<pair<double, double> > >::iterator it = roadSegments.begin(); it != roadSegments.end(); it ++){
			local_cache.updateRoadSegments(it->first, it->second);
			if(local_cache.edge_signature.find(it->first) == local_cache.edge_signature.end())local_cache.edge_signature[it->first] = 0;
		}
	}
	else{
		vector<pair<double, int> >edgeIds;
		set<int> chosen;
		for (map<int, vector<pair<double, double> > >::iterator it = roadSegments.begin(); it != roadSegments.end(); it ++)
		{
			if(local_cache.roadSegments.find(it->first) == local_cache.roadSegments.end())local_cache.roadSegments[it->first] = it->second;
			else
				local_cache.updateRoadSegments(it->first, it->second);
		}
		for (map<int, vector<pair<double, double> > >::iterator it = local_cache.roadSegments.begin(); it != local_cache.roadSegments.end(); it ++)
		{
			trajectory traj = getCurrentTrajectory();
			double qVal = cacheShareCost(it->first, traj);
			edgeIds.push_back(make_pair(qVal, it->first));
		}
		sort(edgeIds.begin(), edgeIds.end(), peerSort());

		int cnt = 0;
		for (vector<pair<double, int> >::iterator iter = edgeIds.begin(); iter != edgeIds.end(); iter ++)
		{
			chosen.insert(iter->second);
			cnt ++;
			if(cnt == EGSIZE)break;
		}
		set<int> overdue;
		for (tr1::unordered_map<int, node>::iterator it = local_cache.objects.begin(); it != local_cache.objects.end();){
			int teid = it->second.eid;
			if(chosen.find(teid) == chosen.end()){
				if(overdue.find(teid) == overdue.end())overdue.insert(teid);
				//if(local_cache.edge_signature[teid] < 0)printf("Oops, edge_signature < 0\n");
				local_cache.objects.erase(it ++);//pay attention to this
			}
			else
				it ++;
		}
		for(set<int>::iterator it = overdue.begin(); it != overdue.end(); it ++){
			local_cache.edge_signature.erase(*it);
			local_cache.roadSegments.erase(*it);
			//if(search.find(*it) != search.end())search.erase(*it);
		}
		for (tr1::unordered_map<int, node>::iterator iter = objects.begin(); iter != objects.end(); iter ++){
			int teid = iter->second.eid;
			if(chosen.find(teid) != chosen.end()){
				if(local_cache.isIdExisted(iter->first) == false){
					local_cache.insertNewNode(iter->second);
					local_cache.updateEdgesig(iter->second.eid);
				}
				else{
					if (local_cache.objects[iter->first].time_to_live < objects[iter->first].time_to_live){
						local_cache.objects[iter->first].time_to_live = objects[iter->first].time_to_live;
					}
				}
			}	
		}
		for (map<int, vector<pair<double, double> > >::iterator it = local_cache.roadSegments.begin(); it != local_cache.roadSegments.end();)
		{
			int teid = it->first;
			if(chosen.find(teid) == chosen.end()){
				local_cache.edge_signature.erase(teid);
				//if(search.find(teid) != search.end())search.erase(teid);
				local_cache.roadSegments.erase(it ++);
			}
			else{
				if(local_cache.edge_signature.find(teid) == local_cache.edge_signature.end())local_cache.edge_signature[teid] = 0;
				it ++;
			}
		}
	}
	return;
}
int vehicle::compareid ( vector<int>& ids, set<int>& std_ids )
{
	int ans = 0;
	for(int i = 0; i < ids.size(); i ++ ){
		if (std_ids.find(ids[i]) != std_ids.end()){
		ans ++;
		}
	}
	return ans;
}
void vehicle::checkCacheForOverdue(msg_statistic& msgsta)
{
	msgsta.number += query_msg_queue.size();
	
	for ( map<int, query_msg>::iterator iter = query_msg_queue.begin(); iter != query_msg_queue.end(); iter ++){
		msgsta.obj_size += iter->second.objectlist.size();
		msgsta.road_num += iter->second.roadSegments.size();
		//int roadbit = 0;
		for (map<int, vector<pair<double, double> > >::iterator i = iter->second.roadSegments.begin(); i != iter->second.roadSegments.end(); i ++)
		{
			msgsta.road_size += i->second.size();
			//roadbit += i->second.size() * 16;//16 = 2 * double
		}
		//roadbit += 4;
		//iter->second.RD_size = roadbit;
		for (map<int, node>::iterator it = iter->second.objectlist.begin(); it != iter->second.objectlist.end();){
			it->second.time_to_live -= vehicle::timechip;

			if (it->second.time_to_live <= 0){
				iter->second.objectlist.erase(it ++);
			}
			else
				it ++;
		}
	}
}
void vehicle::timeElapseForQuery(int cur_time, Statistic& statistic, edge* edgemap, node * nodemap, Grid& mainGrid)
{
	for(map<int, query_msg>::iterator iter = query_msg_queue.begin(); iter != query_msg_queue.end(); ){
		if(cur_time >= iter->second.endtime){
			query_msg msg = iter->second;
			if(msg.query_type == KNN_QUERY){
				if(msg.getDestinationVehicleId() == getVehicleId() && (((query_state >> 1) & 1) == 0)){
					if(debug)cout << "vehicle:\t" << getVehicleId() << " kNN query failed" << endl;
					double tmpttl = compute_ttl_test(mainGrid, msg.cache_range, cur_time);
					query_msg newMsg = query_msg(vid, cur_time, msg.k_q, msg.cache_range, 
						local_cache.getRoadSementsCache(getQueryTrajaectory(), msg.cache_range, nodemap, edgemap),
						tmpttl + cur_time, getQueryTrajaectory(), RANGE_QUERY, msg.cache_range);
					printf("%d newMsg ttl: %lf %lf\n", newMsg.getDestinationVehicleId(), newMsg.endtime - msg.endtime, msg.cache_range);
					//printf("*newMsg ttl: %lf\n", newMsg.endtime - cur_time);
					/*query_msg newMsg = query_msg(vid, cur_time, msg.k_q, query_msg::cache_range, 
						local_cache.getRoadSementsCache(getQueryTrajaectory(), query_msg::cache_range),
						2 + cur_time, getQueryTrajaectory(), RANGE_QUERY);*/
					//compute_ttl ( msg, edgemap, nodemap, query_msg::cache_range, cur_time),  getQueryTrajaectory(), RANGE_QUERY);
					statistic.ttl += tmpttl;
					statistic.dist += msg.cache_range;
					query_msg_queue[newMsg.query_id] = newMsg;
					query_state |= 2;
					//printf("knn exp %d %d %d\n", vid, cur_time, cur_time - msg.getTimeStamp());
					statistic.carry_time += cur_time - msg.getTimeStamp();
				}
			}else if(msg.query_type == RANGE_QUERY ){
				if(msg.getDestinationVehicleId() == getVehicleId() && (((query_state >> 2) & 1) == 0) ){
					vector<int> ids = local_cache.getFristKobjects(msg.getQueryTrajectory(), msg.k_q, msg.range);
					
					if(debug){
						cout << "range:\t" << msg.range << " " << msg.k_q << endl;
						cout << "vehicle:\t" << getVehicleId() << " range query failed" << endl;
						cout << "ans :\t";
						for(int i = 0 ; i < ids.size(); i++){
							cout << ids[i] << "\t";
						}
						cout << endl;
					}
					
					set<int> std_ids = grid->knnQueryInGrid(msg.getQueryTrajectory(), msg.k_q, msg.range);
					if (debug){
						cout << "std :\t";
						for(set<int>::iterator iter = std_ids.begin(); iter != std_ids.end(); iter ++){
							cout << (*iter) << "\t";
						}
						cout << endl;
					}
					//printf("range exp %d %d %d\n", vid, cur_time, cur_time - msg.getTimeStamp());
					statistic.coverage += msg.CoverageRatio();
					statistic.carry_time += cur_time - msg.getTimeStamp();
					//***************************************************************
					int ansnum = compareid( ids, std_ids );// by sherry
					if (std_ids.size() != 0)
						statistic.accuracy += ansnum * 1.0 / std_ids.size();//k-nn
					else
						if ( ansnum == 0 ) statistic.accuracy += 1.0;
					if (ansnum == std_ids.size()){
						if(msg.isCoverageSatisfy()){
							statistic.completeknn ++;
						}
						statistic.precision ++;
					}
					//***************************************************************
					query_state |= 4;
					finish = true;
				}
			}else{
				cerr << "vehicle:\t" << getVehicleId() << " No such query" << endl;
			}	
			query_msg_queue.erase(iter ++);
		}else{
			iter ++;
		}
	}
}

void vehicle::handleQuery(int cur_time, Statistic& statistic, edge* edgemap, node* nodemap, Grid& mainGrid)
{
	set<int> neighids;
	if(cur_time > Grid::TIME && (int)query_msg_queue.size() > 0)findNeighors(neighids);
	for(map<int, query_msg>::iterator iter = query_msg_queue.begin(); iter != query_msg_queue.end();){

		pair<int, int> pos = Grid::getLocFromGeo(iter->second.getQueryLoction());
		//updateLocalCacheFromMsg(iter->second);
		updateMsgFromLocalCache(iter->second);

		query_msg msg = iter->second;
		if(msg.query_type == KNN_QUERY){
			if(msg.getDestinationVehicleId() == getVehicleId()){
				updateLocalCacheFromMsg(iter->second);
				if(msg.isKnnQueryFinished() && (((query_state >> 1) & 1) == 0) ){
					if(debug)cout << "vehicle:\t" << getVehicleId() << " kNN query finished" << endl;

					statistic.carry_time += cur_time - msg.getTimeStamp();//use to +1 by 0725
					//printf("knn finish %d %d %d\n", vid, cur_time, cur_time - msg.getTimeStamp());
					query_msg_queue.erase(iter ++);
					//cout << "vehicleid" << ' ' << getVehicleId() << endl; // add at 5.31
					double tmpdist = local_cache.getRangeOfRangeQuery(getQueryTrajaectory(), msg.k_q, msg.cache_range);
					double tmpttl = compute_ttl_test(mainGrid, tmpdist, cur_time);
					//double kthD = local_cache.getRangeOfRangeQuery(getQueryTrajaectory(), msg.k_q, msg.cache_range);
					query_msg newMsg = query_msg(vid, cur_time, msg.k_q, tmpdist, local_cache.getRoadSementsCache(getQueryTrajaectory(), tmpdist, nodemap, edgemap),
						cur_time + tmpttl, getQueryTrajaectory(), RANGE_QUERY, msg.cache_range);
					statistic.ttl += tmpttl;
					statistic.dist += tmpdist;
					if(debug)printf("%d newMsg ttl: %lf %lf\n", newMsg.getDestinationVehicleId(), newMsg.endtime - cur_time, tmpdist);
					/*query_msg newMsg = query_msg(vid, cur_time, msg.k_q, tmpdist, local_cache.getRoadSementsCache(getQueryTrajaectory(), tmpdist),
						msg.endtime + 2, getQueryTrajaectory(), RANGE_QUERY);*/

					//cout << getVehicleId() << ' ' << compute_ttl ( msg, edgemap, nodemap, local_cache.getRangeOfRangeQuery(getQueryTrajaectory(), msg.k_q), cur_time) << endl;
					query_msg_queue[newMsg.query_id] = newMsg;
					query_state |= 2;
				}else{
					//if(vid == 0)printf("knn + is destination + knn not finished\n");
					if (iter->second.broadcast == false)
					{
						iter->second.broadcast = true;
						msg.broadcast = true;
						broadCastQuery(msg, statistic, neighids);
						//printf("%d broadcast knn query %d\n", getVehicleId(), msg.query_id);
						//cout << msg.broadcast << endl;
					}
					iter ++;
				}
			}else{
				//if(vid == 0)printf("knn + not destination\n");
				double testdist = Grid::caculateRoadNetworkDist(msg.getQueryTrajectory(), getCurrentTrajectory());
				if (MathFun::D(testdist - msg.cache_range) > 0){
					iter ++;
					continue;
				}
				bool found = broadCastQuery(msg, statistic, neighids);
				query_msg_queue.erase(iter ++);
				//if(found && msg.isKnnQueryFinished())query_msg_queue.erase(iter ++);
				//else {
				//	//broadCastQuery(msg);
				//	iter ++;
				//}
			}
		}else if(msg.query_type == RANGE_QUERY){
			if(msg.getDestinationVehicleId() == getVehicleId()){
				updateLocalCacheFromMsg(iter->second);
				//if(msg.isRangeQueryFinished() && (((query_state >> 2) & 1) == 0) ){
				if(msg.isCoverageSatisfy() && (((query_state >> 2) & 1) == 0) ){
					//printf("range finish %d %d %d\n", vid, cur_time, cur_time - msg.getTimeStamp());
					statistic.coverage += msg.CoverageRatio();
					statistic.carry_time += cur_time - msg.getTimeStamp();
					finish = true;
					query_msg_queue.erase(iter ++);
					query_state |= 4;
					vector<int> ids = local_cache.getFristKobjects(msg.getQueryTrajectory(), msg.k_q, msg.range);
					if (debug){
						cout << "range:\t" << msg.range << " " << msg.k_q << endl;
						cout << "vehicle:\t" << getVehicleId() << " Query finished" << endl;
						cout << "ans :\t";
						for(int i = 0 ; i < ids.size(); i++){
							cout << ids[i] << "\t";
						}
						cout << endl;
					}
					
					set<int> std_ids = grid->knnQueryInGrid(msg.getQueryTrajectory(), msg.k_q, msg.range);
					if (debug){
						cout << "std :\t";
						for(set<int>::iterator iter = std_ids.begin(); iter != std_ids.end(); iter ++){
							cout << (*iter) << "\t";
						}
						cout << endl;
					}
					
					//********************************************************
					int ansnum = compareid( ids, std_ids );//by sherry
					if ( std_ids.size() != 0 )
						statistic.accuracy += ansnum * 1.0 / std_ids.size();//k-nn
					else
						if( ansnum == 0 )statistic.accuracy += 1.0;
					if (ansnum == std_ids.size())
					{
						statistic.precision ++;
						statistic.completeknn ++;
					}
					//********************************************************
				}else{
					//if(vid ==0)printf("range query + is destination + not finished\n");
					if (iter->second.broadcast == false)
					{
						iter->second.broadcast = true;
						msg.broadcast = true;
						broadCastQuery(msg, statistic, neighids);
						//printf("%d broadcast range query %d\n", getVehicleId(), msg.query_id);
						//cout << msg.broadcast << endl;
					}
					iter ++;
				}
			}else{
				//if(vid == 0)printf("range query + not destination\n");
				double testdist = Grid::caculateRoadNetworkDist(msg.getQueryTrajectory(), getCurrentTrajectory());
				if (MathFun::D(testdist - msg.range) > 0){
					iter ++;
					continue;
				}
				bool found = broadCastQuery(msg, statistic, neighids);
				query_msg_queue.erase(iter ++);
				//if(found && msg.isRangeQueryFinished()){
				//	query_msg_queue.erase(iter ++);
				//}else{
				//	//broadCastQuery(msg);
				//	iter ++;
				//}
			}
		}else{
			cerr << "vehicle:\t" << getVehicleId() <<" no such query type" << endl; 
		}
	}
}

void vehicle::handleMove( int s_time, Roadsideunit * rsumap, Grid& mainGrid, Statistic& statistic, overhead& rsu_statistic)
{
	//cerr << getCurrentPosition().lat << ' ' << getCurrentPosition().lng << ' ' << getVehicleId() << endl;
	//if(trajectorys.size() > 1){ // load changes at 5.30
		//grid->remove_vehicle(getCurrentPosition(), getVehicleId()); //add at 6.1 for load changes
		//trajectorys.pop();// load changes at 5.30
		//checkCacheForOverdue(objSemantic);
		grid->add_vehicle( getCurrentPosition(), getVehicleId(), s_time );
		//if(local_cache.objects.size() < CASIZE){
			addNewRoadSegmentFromCell( s_time );
			addNewObjectFromCell( statistic, s_time);
			correctEdges();
		//}
		if(Grid::RSU_SWITCH && s_time > Grid::TIME)updateVehiclesfromRoadsideunit(rsumap, mainGrid, rsu_statistic);
		//if(Grid::testExchange && s_time > Grid::TIME){
		//	if ((int)query_msg_queue.size() != 0){
		//		if(exBegin == false){
		//			predict = s_time;
		//			exBegin = true;
		//		}
		//		if (s_time != predict)return;
		//		exchangeInfo(ex_statistic, s_time);
		//	}
		//	//exchangeInfo(ex_statistic);
		//	//if(vid == 11||vid == 5000)printf("# %d %d\n", vid, predict);
		//}
	//}// load changes at 5.30
	//meet object
	return;
}

void vehicle::handleCacheShare(int s_time, overhead& ex_statistic)
{
	if ((int)query_msg_queue.size() != 0){
		if(exBegin == false){
			predict = s_time;
			exBegin = true;
		}
		if (s_time != predict)return;
		exchangeInfo(ex_statistic, s_time);
	}
	//exchangeInfo(ex_statistic);
	//if(vid == 11||vid == 5000)printf("# %d %d\n", vid, predict);
	return;
}
int vehicle::getVehicleId()
{
	return vid;
}
void vehicle::correctEdges()
{
	for (map<int, vector<pair<double, double> > >::iterator iter = local_cache.roadSegments.begin(); iter != local_cache.roadSegments.end(); iter ++)
	{
		if (local_cache.edge_signature.find(iter->first) == local_cache.edge_signature.end())
		{
			//if ((*emptyEdgemap).find(iter->first) == (*emptyEdgemap).end())
				local_cache.edge_signature[iter->first] = 0;
		}
	}
}
void vehicle::addNewObjectFromCell(Statistic& statistic, int s_time)
{
	loc tloc = getCurrentPosition();
	loc leftdown = loc( tloc.lat - delat_lat, tloc.lng - delat_lng );
	loc rightup = loc( tloc.lat + delat_lat, tloc.lng + delat_lng );
	pair<int, int> pos = Grid::getLocFromGeo(tloc);
	//vector<int> tobjs;
	vector<int> tmust;
	bool replace = false;
	int num = (int)local_cache.objects.size();
	set<int> crossEdge;
	map<int, pair<double, double> > roads;
	bool flag = false;
	if (s_time % 120 == 0)flag = true;
	//add by sherry***********************************
	//int n = 0;//neighbor num
	//int temp = INT * 3 + DOUBLE + OTHER + HEADER;
	//************************************************
	for(int i = -1; i <= 1; i++){
		for(int j = -1; j <= 1; j++){
			int x = i + pos.first, y = j + pos.second;
			if(!Grid::is_Inrange(x, y))continue;
			loc tleftdown = loc( x * grid_lat + Grid::trans_low.lat, y * grid_lng + Grid::trans_low.lng );
			loc trightup = loc( (x + 1) * grid_lat + Grid::trans_low.lat, (y + 1) * grid_lng + Grid::trans_low.lng );
			
			if (( leftdown.lng < trightup.lng )&&( rightup.lng > tleftdown.lng )&&( leftdown.lat < trightup.lat )&&( rightup.lat > tleftdown.lat ))
			{
				set<int>* ids;
				grid->getAllObjectsInCell(x, y, ids);
				for(set<int>::iterator iter = ids->begin(); iter != ids->end(); iter ++){
					int id = *iter;
					if(local_cache.isIdExisted(id))continue;
					node* objectnode = &(nodemap[id]);
					double dist = Grid::calculateDist(getCurrentPosition(), objectnode->getLoc());
					//printf("%lf objloc %lf %lf vehloc %lf %lf", dist, objectnode->getLoc().lat, objectnode->getLoc().lng, getCurrentPosition().lat, getCurrentPosition().lng);//0718
					if(MathFun::D(dist - Grid::COMRANGE) <= 0){
						local_cache.insertNewNode(*objectnode);
						if(local_cache.edge_signature.find(objectnode->eid) == local_cache.edge_signature.end())
							local_cache.edge_signature[objectnode->eid] = 1;
						else
							local_cache.edge_signature[objectnode->eid] ++;
						if (local_cache.roadSegments.find(objectnode->eid) == local_cache.roadSegments.end())
						{
							local_cache.updateRoadSegment(objectnode->eid, (*nodeToDistmap)[objectnode->nid]);
						}
						//statistic.number++;
						//statistic.power += 0.5 * temp + 56;
						//statistic.msg_size += temp;
						//cout << "vehicle id:\t" << getVehicleId() << " add object:\t" << objectnode->nid << " at edge:\t" << objectnode->eid << " from Grid" << endl;
					}
				}
				//add edges
				if(flag == true){
					for (set<int>::iterator it = grid->gridCell[x][y].Edge.begin(); it !=  grid->gridCell[x][y].Edge.end(); it ++)
					{
						if (crossEdge.find(*it) == crossEdge.end())crossEdge.insert(*it);
					}
				}
		    }
			else continue;
		}
	}
	if(crossEdge.empty() == true)return;
	for (set<int>::iterator iter = crossEdge.begin(); iter != crossEdge.end(); iter ++)
	{
		int eid = *iter;
		
		/*if(local_cache.roadSegments.find(eid) == local_cache.roadSegments.end()){
			if(vid == 11)printf("edge %d checked, new\n", eid);}
		else
			if(vid == 11)printf("edge %d checked, old\n", eid);*/
		int line_sid = edgemap[eid].startnid;
		int line_eid = edgemap[eid].endnid;
		if(edgeInGridmap->find(*iter) != edgeInGridmap->end()){
			roads[eid] = make_pair(0, Grid::getRoadNetworkDist(line_sid, line_eid));
			//if(vid == 11)printf("%d %d %lf\n", eid, 0, Grid::getRoadNetworkDist(line_sid, line_eid));
			continue;
		}
		else{
			loc loc_start = nodemap[line_sid].getLoc();
			loc loc_end = nodemap[line_eid].getLoc();
			//printf("%lf\n", MathFun::point_seg_dist(loc(2, 2), loc(3, 0), loc(1, 1)));
			//printf("%lf\n", MathFun::point_seg_dist(loc(2, 2), loc(0, 1), loc(1, 2)));
			if(MathFun::D(MathFun::point_seg_dist(tloc, loc_start, loc_end) - Grid::COMRANGE) >= 0)continue;
			loc p1, p2;
			//MathFun::lineCrossCircle( circle(loc(2, 2), 1), loc(3, 2), loc(0, 1), p1, p2);
			//MathFun::lineCrossCircle( circle(loc(2, 2), 1), loc(3, 0), loc(1, 1), p1, p2);
			//MathFun::lineCrossCircle( circle(loc(2, 2), 1), loc(0, 1), loc(1, 2), p1, p2);
			//printf("# %lf %lf %lf %lf\n", p1.lat, p1.lng, p2.lat, p2.lng);
			//if(vid == 11)printf("* %lf %lf %lf %lf\n", loc_start.lat, loc_start.lng, loc_end.lat, loc_end.lng);
			MathFun::lineCrossCircle( circle(tloc, Grid::COMRANGE), loc_start, loc_end, p1, p2);
			//if(vid == 11){
				//printf("# %lf %lf %lf %lf\n", p1.lat, p1.lng, p2.lat, p2.lng);
				//if(p1.lat < Grid::low.lat||p1.lat > Grid::high.lat ||p2.lat < Grid::low.lat ||p2.lat > Grid::high.lat)printf("Oops, vid %d %d lat wrong\n", vid, eid);
				//if(p1.lng < Grid::low.lng||p1.lng > Grid::high.lng ||p2.lng < Grid::low.lng ||p2.lng > Grid::high.lng)printf("Oops, vid %d %d lng wrong\n", vid, eid);
			//}
			int p1_loc, p2_loc, direct;
			if(MathFun::innerProduct(loc_start, p1, loc_end, p1) > 0)p1_loc = -1;//p1 is not on (loc_start, loc_end)
			else
				p1_loc = 1;
			if(MathFun::innerProduct(loc_start, p2, loc_end, p2) > 0)p2_loc = -1;
			else
				p2_loc = 1;
			direct = p1_loc + p2_loc;
			switch (direct)
			{
			case -2:
				{
					if (MathFun::innerProduct(p1, loc_start, p2, loc_start) < 0)
						roads[eid] = make_pair(0, Grid::getRoadNetworkDist(line_sid, line_eid));
					//if(vid == 11)printf("-2 %d %d %lf\n", eid, 0, Grid::getRoadNetworkDist(line_sid, line_eid));
					break;
				}
			case 0:
				{
					loc vertexOnP1P2, PonSegment;
					//what is on the p1p2
					if (MathFun::innerProduct(p1, loc_start, p2, loc_start) <= 0)vertexOnP1P2 = loc_start;
					else
						vertexOnP1P2 = loc_end;
					if (p1_loc == 1) PonSegment = p1;
					else
						PonSegment = p2;
					double l1 = Grid::calculateDist(loc_start, vertexOnP1P2);
					double l2 = Grid::calculateDist(loc_start, PonSegment);
					double first = MIN(l1, l2);
					double second = MAX(l1, l2);
					roads[eid] = make_pair(first, second);
					//if(MathFun::D(Grid::getRoadNetworkDist(line_sid, line_eid) - second) < 0)printf("case 0\n");
					//if(vid == 11)printf("0 %d %lf %lf\n", eid, first, second);
					break;
				}
			case 2:
				{
					double l1 = Grid::calculateDist(loc_start, p1);
					double l2 = Grid::calculateDist(loc_start, p2);
					double first = MIN(l1, l2);
					double second = MAX(l1, l2);
					roads[eid] = make_pair(first, second);
					if(MathFun::D(Grid::getRoadNetworkDist(line_sid, line_eid) - second) < 0)printf("case 2\n");
					//if(vid == 11)printf("2 %d %lf %lf\n", eid, first, second);
					break;
				}

			default:
				{
					printf("Oops, wrong value %d\n", direct);
					break;
				}
			}
		}
	}
	//printf("edges not in roadsegment:%d\n", n_e);
	//printf("vid %d roads %d\n", vid, (int)roads.size());
	for (map<int, pair<double, double> >::iterator iter = roads.begin(); iter != roads.end(); iter ++)
	{
		//if(vid == 11)printf("road %d added\n", iter->first);
		local_cache.updateRoadSegment(iter->first, iter->second);
		if(local_cache.edge_signature.find(iter->first) == local_cache.edge_signature.end()){
			local_cache.edge_signature[iter->first] = 0;
		}
	}
}

void vehicle::addNewRoadSegmentFromCell( int time )
{
	if(last_trajectory.eid == -1 || last_trajectory == getCurrentTrajectory())
	{
		return;
	}
	if(last_trajectory.eid == getCurrentTrajectory().eid){
		node* node1 = &(nodemap)[edgemap[last_trajectory.eid].startnid];
		double dist1 = Grid::calculateDist(node1->getLoc(), last_trajectory.location);
		double dist2 =  Grid::calculateDist(node1->getLoc(), getCurrentTrajectory().location);
		local_cache.updateRoadSegment(last_trajectory.eid, make_pair(dist1, dist2));
	}
	else{
		int last_sid = edgemap[last_trajectory.eid].startnid;
		int last_eid = edgemap[last_trajectory.eid].endnid;
		double last_dist = Grid::getRoadNetworkDist(last_sid, last_eid);
		int now_sid = edgemap[getCurrentTrajectory().eid].startnid;
		int now_eid = edgemap[getCurrentTrajectory().eid].endnid;
		double now_dist = Grid::getRoadNetworkDist(now_sid, now_eid);

		vector<pair<int, int> > edgeList = (*nodeToEdgemap)[last_sid];
		for(int i = 0; i < edgeList.size(); i ++){
			if(edgeList[i].first == getCurrentTrajectory().eid){

				edge* edge1 = &(edgemap)[getCurrentTrajectory().eid];
				//if(last_sid == now_sid){
				//	edge1->number_of_interarrivals_sid ++;
				//	//printf("%d ", edge1->number_of_interarrivals_sid);
				//	if(edge1->interarrival_sid){
				//		edge1->sum_of_interarrival_time_sid = time;
				//		edge1->interarrival_sid = false;
				//	}
				//}
				//else{
				//	edge1->number_of_interarrivals_eid ++;
				//	//printf("%d ", edge1->number_of_interarrivals_eid);
				//	if(edge1->interarrival_eid){
				//		edge1->sum_of_interarrival_time_eid = time;
				//		edge1->interarrival_eid = false;
				//	}
				//}
				node* node1 = &(nodemap)[last_sid];
				/*node1->number_of_interarrivals ++;
				if ( node1->interarrival ){
					node1->sum_of_interarrival_time = time;
					node1->interarrival = false;
				}*/
				//printf("%d %d %d %d\n", vid, node1->nid, last_trajectory.eid,getCurrentTrajectory().eid);
				double tdist1 = Grid::calculateDist(node1->getLoc(), last_trajectory.location);
				double tdist2 = Grid::calculateDist(node1->getLoc(), getCurrentTrajectory().location);
				local_cache.updateRoadSegment(last_trajectory.eid, make_pair(0, tdist1));
				if (last_sid == now_sid){
					local_cache.updateRoadSegment(getCurrentTrajectory().eid, make_pair(0, tdist2));
				}
				else
					local_cache.updateRoadSegment(getCurrentTrajectory().eid, make_pair(now_dist-tdist2, now_dist));
				return;
			}
		}
		vector<pair<int, int> > edgeList1 = (*nodeToEdgemap)[last_eid];
		for(int i = 0; i < edgeList1.size(); i ++){
			if(edgeList1[i].first == getCurrentTrajectory().eid){
				edge* edge1 = &(edgemap)[getCurrentTrajectory().eid];
				//if(last_eid == now_sid){
				//	edge1->number_of_interarrivals_sid ++;
				//	//printf("%d ", edge1->number_of_interarrivals_sid);
				//	if(edge1->interarrival_sid){
				//		edge1->sum_of_interarrival_time_sid = time;
				//		edge1->interarrival_sid = false;
				//	}
				//}
				//else{
				//	edge1->number_of_interarrivals_eid ++;
				//	//printf("%d ", edge1->number_of_interarrivals_eid);
				//	if(edge1->interarrival_eid){
				//		edge1->sum_of_interarrival_time_eid = time;
				//		edge1->interarrival_eid = false;
				//	}
				//}
				node* node1 = &(nodemap)[last_eid];
				/*node1->number_of_interarrivals ++;
				if ( node1->interarrival ){
					node1->sum_of_interarrival_time = time;
					node1->interarrival = false;
				}*/
				//printf("%d %d %d %d\n", vid, node1->nid, last_trajectory.eid,getCurrentTrajectory().eid);
				double tdist1 = Grid::calculateDist(node1->getLoc(), last_trajectory.location);
				double tdist2 = Grid::calculateDist(node1->getLoc(), getCurrentTrajectory().location);
				local_cache.updateRoadSegment(last_trajectory.eid, make_pair(tdist1, last_dist - tdist1));
				if(last_eid == now_sid){
					local_cache.updateRoadSegment(getCurrentTrajectory().eid, make_pair(0, tdist2));
				}
				else
					local_cache.updateRoadSegment(getCurrentTrajectory().eid, make_pair(now_dist-tdist2, now_dist));
				return;
			}
		}
		//printf("Need to find road!\n");
		vector<trajectory> roads = findRoad(last_trajectory, getCurrentTrajectory(), time);
		for(int i = 0; i < roads.size() - 1; i++){
			node* node1 = &(nodemap)[edgemap[roads[i].eid].startnid];
			double dist1 = Grid::calculateDist(node1->getLoc(), roads[i].location);
			//double dist2 = Grid::calculateDist(node1->getLoc(), roads[i + 1].location);
			double dd = Grid::getRoadNetworkDist(Grid::edgemap[roads[i].eid].startnid, Grid::edgemap[roads[i].eid].endnid);
			//local_cache.updateRoadSegment(roads[i].eid, make_pair(dist1, dist2));
			local_cache.updateRoadSegment(roads[i].eid, make_pair(0, dd));

			//node1->number_of_interarrivals ++;
			//if ( node1->interarrival ){
			//	node1->sum_of_interarrival_time = time;
			//	node1->interarrival = false;
			//}
			//printf("%d %d %d %d\n", vid, node1->nid, last_trajectory.eid,getCurrentTrajectory().eid);
			//printf("%d %d\n", vid, node1->nid);
		}
	}
}
void vehicle::updateLastTrajectory( pair<int, int> pos )
{
	last_trajectory = getCurrentTrajectory();
	if(last_grid.empty() == true)last_grid.push_back(pos);
	else{
		pair<int, int> last = last_grid.back();
		if(last.first == pos.first && last.second == pos.second)return;
		else
			last_grid.push_back(pos);
	}
	//if(s_time < 3)cout << s_time << ' ' << last_trajectory.eid << ' ' << last_trajectory.location.lat << ' ' << last_trajectory.location.lng << endl;
}

trajectory vehicle::getCurrentTrajectory()
{
	return trajectorys; // load changes at 5.30
}

vector<trajectory> vehicle::findRoad( trajectory start, trajectory end, int time )
{
	vector<trajectory> ans;
	vector<pair<int, int> > tmpRoadVertex; 
	vector<int> tmpRoads;
	ans.push_back(start);
	edge edge1;
	if(start.eid != end.eid) edge1 = (edgemap)[start.eid];
	if(findNextRoadVertex(edge1.startnid, start.eid, end.eid, tmpRoadVertex, tmpRoads, 0, 3) || 
		findNextRoadVertex(edge1.endnid, start.eid, end.eid, tmpRoadVertex, tmpRoads, 0, 3)){
			for(int i = 0 ; i < tmpRoadVertex.size(); i++){
			node tmpNode = (nodemap)[tmpRoadVertex[i].second];
			ans.push_back(trajectory(tmpRoadVertex[i].first, tmpNode.getLoc()));
			//printf("%d %d %d %d\n", vid, tmpRoadVertex[i].second, last_trajectory.eid, getCurrentTrajectory().eid);
		}
			for (int i = 0; i < tmpRoads.size(); i ++)
			{
				node* node1 = &(nodemap)[tmpRoads[i]];
				node1->number_of_interarrivals ++;
				if ( node1->interarrival ){
					node1->sum_of_interarrival_time = time;
					node1->interarrival = false;
				}
				//printf("%d %d %d %d\n", vid, node1->nid, last_trajectory.eid,getCurrentTrajectory().eid);
			}
	}else{
		node node1 = (nodemap)[(edgemap)[start.eid].startnid];
		node node2 = (nodemap)[(edgemap)[start.eid].endnid];
		double dist1 = Grid::calculateDist(node1.getLoc(), start.location);
		double dist2 = Grid::calculateDist(node2.getLoc(), start.location);
		if(dist1 < dist2){
			ans.push_back(trajectory(start.eid, node1.getLoc()));
		}else{
			ans.push_back(trajectory(start.eid, node2.getLoc()));
		}
		node1 = (nodemap)[(edgemap)[end.eid].startnid];
		node2 = (nodemap)[(edgemap)[end.eid].endnid];
		dist1 = Grid::calculateDist(node1.getLoc(), end.location);
		dist2 = Grid::calculateDist(node2.getLoc(), end.location);
		if(dist1 < dist2){
			ans.push_back(trajectory(end.eid, node1.getLoc()));
		}else{
			ans.push_back(trajectory(end.eid, node2.getLoc()));
		}
	}
	ans.push_back(end);

	return ans;
}

bool vehicle::findNextRoadVertex( int startnid, int starteid, int endeid, vector<pair<int, int> >& road_vertexs, vector<int>& roads, int dp, int maxDp )
{
	if(starteid == endeid)return true;
	if(maxDp <= dp)return false;
	vector<pair<int, int> > edgeList = (*nodeToEdgemap)[startnid];
	for(int i = 0; i < edgeList.size(); i++){
		if(edgeList[i].first == starteid)continue;
		road_vertexs.push_back(edgeList[i]);
		roads.push_back(startnid);
		//cout << startnid << endl;
		if(findNextRoadVertex(edgeList[i].second, edgeList[i].first, endeid, road_vertexs, roads, dp + 1, maxDp))return true;
		road_vertexs.pop_back();
		roads.pop_back();
	}
	return false;
}

void vehicle::findNeighors(set<int>& ids)
{
	loc curLoction = getCurrentPosition();
	loc leftdown = loc( curLoction.lat - delat_lat, curLoction.lng - delat_lng );
	loc rightup = loc( curLoction.lat + delat_lat, curLoction.lng + delat_lng );
	pair<int, int> pos = Grid::getLocFromGeo(curLoction);
	for(int i = -1; i <= 1; i++){
		for(int j = -1; j <= 1; j++){
			int x = pos.first + i, y = pos.second + j;
			if(!Grid::is_Inrange(x, y))continue;
			loc tleftdown = loc( x * grid_lat + Grid::trans_low.lat, y * grid_lng + Grid::trans_low.lng );
			loc trightup = loc( (x + 1) * grid_lat + Grid::trans_low.lat, (y + 1) * grid_lng + Grid::trans_low.lng );

			if (( leftdown.lng < trightup.lng )&&( rightup.lng > tleftdown.lng )&&( leftdown.lat < trightup.lat )&&( rightup.lat > tleftdown.lat )){
				set<int>* vids;
				grid->getAllVehicleInCell(x, y, vids);

				for(set<int>::iterator iter = vids->begin(); iter != vids->end(); iter ++){
					int id = *iter;
					if(id == getVehicleId())continue;
					vehicle* tu = &(vehiclemap[id]);
					double mindist = Grid::calculateDist(getCurrentPosition(), tu->getCurrentPosition());
					if(MathFun::D(mindist - Grid::COMRANGE) <= 0){
						ids.insert(id);
					}
				}
			}
		}
	}
	return;
}

bool vehicle::broadCastQuery( query_msg msg , Statistic& statistic, set<int>& ids)
{
	//double probabilty = 1.0 * (rand() % 100) / 100;
	//if(probabilty > 0.1 )return false;//by 0909
	int temp = 0;
	if(msg.query_type == KNN_QUERY)
		temp = ((int)msg.objectlist.size()) * OBJSIZEBIT + HEADER + QINFO;// temp in bits
	else{
		int roadbits = 0;
		roadbits += msg.roadSegments.size() * EID;//(eid, a, b) eid is 4 bits
		for ( map<int, vector<pair<double, double> > >::iterator iter = msg.roadSegments.begin(); iter != msg.roadSegments.end(); iter ++)
		{
			roadbits += iter->second.size() * SEMANIC;//(eid, a, b) a and b are both 8 bits
		}
		temp = ((int)msg.objectlist.size()) * OBJSIZEBIT + roadbits + HEADER + QINFO;// temp in bits
	}
	bool found = false;
	//int n = 0;
	for(set<int>::iterator iter = ids.begin(); iter != ids.end(); iter ++){
		int id = *iter;
		if(id == msg.getDestinationVehicleId())found = true;
		vehicle* tu = &(vehiclemap[id]);
		//printf("broadcast %d %lf\n", vid, neiNum);
		tu->receiveQuery(msg, statistic);
		statistic.power += 0.5 * temp / 8 + 56;
		//n ++;
	}
	//if(n > 0){
		statistic.number ++;
		statistic.msg_size += temp;
		statistic.power += 1.9 * temp / 8 + 266;// 1.9 * bytes + 266
		statistic.transtime += 1.0 * temp / TRANSBIT;
		//printf("%d total broadcast %d total neighbor %d\n", vid, n, (int)ids.size());
	//}
	return found;
}

void vehicle::receiveQuery( query_msg msg, Statistic& statistic )
{
	//checkRange 4km
	if (msgReceive.find(msg.query_id) != msgReceive.end()){
		if(msg.getDestinationVehicleId() != getVehicleId())return;
	}
	msgReceive.insert(msg.query_id);
	double distFromQueryPoint = Grid::caculateRoadNetworkDist(getCurrentTrajectory(), msg.getQueryTrajectory());
	if(distFromQueryPoint > msg.cache_range){
		return;
	}
	if(msg.query_type == KNN_QUERY){
		if(query_msg_queue.find(msg.query_id) == query_msg_queue.end()){
			if(msg.getDestinationVehicleId() == getVehicleId()){
				return;
			}
			query_msg_queue[msg.query_id] = msg;
		}else{
			if(query_msg_queue[msg.query_id].getObjectsNum() < msg.getObjectsNum()){
				query_msg_queue[msg.query_id] = msg;
			}
		}
	}else{
		if(query_msg_queue.find(msg.query_id) == query_msg_queue.end()){
			if(msg.getDestinationVehicleId() == getVehicleId()){
				return;
			}
			query_msg_queue[msg.query_id] = msg;
		}else{
			if(msg.getDestinationVehicleId() == getVehicleId())updateLocalCacheFromMsg(msg);
		}
	}
	return;
}

trajectory vehicle::getQueryTrajaectory()
{
	return query_trajectory;
}

void vehicle::updateLocalCacheFromMsg( query_msg msg)
{
	if (msg.query_type == KNN_QUERY)
	{
		map<int, node> ids = msg.objectlist;// can be faster
		if((int)ids.size() + local_cache.objects.size() <= Grid::CASIZE){
			for(map<int, node>::iterator iter = ids.begin(); iter != ids.end(); iter++){
				if(local_cache.isIdExisted(iter->first) == false){
					local_cache.insertNewNode(iter->second);
					local_cache.updateEdgesig(iter->second.eid);
				}
			}
		}
		else{
			vector<pair <double, node> > CandidateObj;
			for(map<int, node>::iterator iter = ids.begin(); iter != ids.end(); iter++){
				if(local_cache.isIdExisted(iter->first) == false){
					double dist = Grid::caculateRoadNetworkDist(getCurrentTrajectory(), trajectory(iter->second.eid, iter->second.lat, iter->second.lng));
					CandidateObj.push_back(make_pair(dist, iter->second));
				}
			}
			for (tr1::unordered_map<int, node>::iterator iter = local_cache.objects.begin(); iter != local_cache.objects.end(); iter ++)
			{
				double dist = Grid::caculateRoadNetworkDist(getCurrentTrajectory(), trajectory(iter->second.eid, iter->second.lat, iter->second.lng));
				CandidateObj.push_back(make_pair(dist, iter->second));
			}
			sort(CandidateObj.begin(), CandidateObj.end(), candidateSort());
			local_cache.objects.clear();
			for (map<int, int>::iterator iter = local_cache.edge_signature.begin(); iter != local_cache.edge_signature.end();)
			{
				if (iter->second != 0)local_cache.edge_signature.erase(iter ++);
				else
					iter ++;
			}
			for (int i = 0; i < MIN(Grid::CASIZE, CandidateObj.size()); i ++)
			{
				local_cache.insertNewNode(CandidateObj[i].second);
				local_cache.updateEdgesig(CandidateObj[i].second.eid);
			}
			for(map<int, vector<pair<double, double> > >::iterator it = local_cache.roadSegments.begin(); it != local_cache.roadSegments.end();){
				if(local_cache.edge_signature.find(it->first) == local_cache.edge_signature.end())
					local_cache.roadSegments.erase(it ++);
				else
					it ++;
			}
		}
		
	}
	if(msg.query_type == RANGE_QUERY){
		map<int, node> ids = msg.objectlist;// can be faster
		if((int)ids.size() + local_cache.objects.size() <= Grid::CASIZE){
			for(map<int, node>::iterator iter = ids.begin(); iter != ids.end(); iter++){
				if(local_cache.isIdExisted(iter->first) == false){
					local_cache.insertNewNode(iter->second);
					local_cache.updateEdgesig(iter->second.eid);
				}
			}
			for(map<int, vector<pair<double, double> > >::iterator it = msg.roadSegments.begin(); it != msg.roadSegments.end(); it ++){
				local_cache.updateRoadSegments(it->first, msg.roadSegments[it->first]);
				if (local_cache.edge_signature.find(it->first) == local_cache.edge_signature.end())local_cache.edge_signature[it->first] = 0;
			}
		}
		else{
			vector<pair <double, node> > CandidateObj;
			set<int> obj_signature;
			set<int> msg_sig;
			for(map<int, node>::iterator iter = ids.begin(); iter != ids.end(); iter++){
				if(local_cache.isIdExisted(iter->first) == false){
					double dist = Grid::caculateRoadNetworkDist(getCurrentTrajectory(), trajectory(iter->second.eid, iter->second.lat, iter->second.lng));
					CandidateObj.push_back(make_pair(dist, iter->second));
				}
				if (msg_sig.find(iter->second.eid) == msg_sig.end())msg_sig.insert(iter->second.eid);
			}
			for (tr1::unordered_map<int, node>::iterator iter = local_cache.objects.begin(); iter != local_cache.objects.end(); iter ++)
			{
				double dist = Grid::caculateRoadNetworkDist(getCurrentTrajectory(), trajectory(iter->second.eid, iter->second.lat, iter->second.lng));
				CandidateObj.push_back(make_pair(dist, iter->second));
				obj_signature.insert(iter->second.nid);
			}
			sort(CandidateObj.begin(), CandidateObj.end(), candidateSort());
			for (int i = 0; i < MIN(Grid::CASIZE, CandidateObj.size()); i ++)
			{
				if(obj_signature.find(CandidateObj[i].second.nid) != obj_signature.end())
					obj_signature.erase(CandidateObj[i].second.nid);
				else{
					local_cache.insertNewNode(CandidateObj[i].second);
					local_cache.updateEdgesig(CandidateObj[i].second.eid);
				}
			}
			for (set<int>::iterator it = obj_signature.begin(); it != obj_signature.end(); it ++)
			{
				int teid = local_cache.objects[*it].eid;
				local_cache.edge_signature[teid] --;
				if(local_cache.edge_signature[teid] == 0)
				{
					local_cache.roadSegments.erase(teid);
					local_cache.edge_signature.erase(teid);
				}
				if(local_cache.edge_signature[teid] < 0)printf("Oops, edge_signature < 0\n");
				local_cache.objects.erase(*it);
			}
			for (map<int, vector<pair<double, double> > >::iterator iter = msg.roadSegments.begin(); iter != msg.roadSegments.end(); iter ++)
			{
				if (local_cache.edge_signature.find(iter->first) != local_cache.edge_signature.end())
					local_cache.updateRoadSegments(iter->first, msg.roadSegments[iter->first]);
				else
					if(msg_sig.find(iter->first) == msg_sig.end()){
						local_cache.updateRoadSegments(iter->first, msg.roadSegments[iter->first]);
						if (local_cache.edge_signature.find(iter->first) == local_cache.edge_signature.end())local_cache.edge_signature[iter->first] = 0;
					}
			}
			//printf("%d %d\n", vid, (int)local_cache.objects.size());
		}
	}
	return;
}

void vehicle::updateMsgFromLocalCache( query_msg& msg )
{
	if(msg.query_type == KNN_QUERY){
		for( tr1::unordered_map<int, node>::iterator iter = local_cache.objects.begin(); iter != local_cache.objects.end(); iter ++){
			//if(msg.isIdExisted(iter->first) || iter->second.isObjectTimeExpired())continue;
			if(msg.isIdExisted(iter->first)){
				if(iter->second.time_to_live > msg.objectlist[iter->first].time_to_live)
					msg.objectlist[iter->first].time_to_live = iter->second.time_to_live;
				else
					iter->second.time_to_live = msg.objectlist[iter->first].time_to_live;
				continue;
			}
			if(msg.isKnnQueryFinished())break;
			//if (iter->second.time_to_live <= 0)continue;
			double dist = Grid::caculateRoadNetworkDist(msg.getQueryTrajectory(), trajectory(iter->second.eid, iter->second.getLoc()));
			if(MathFun::D(dist - msg.cache_range) <= 0){
				msg.insertNewObject(iter->second);
			}
		}
	}else{
		set<int> msg_sig;
		for( tr1::unordered_map<int, node>::iterator iter = local_cache.objects.begin(); iter != local_cache.objects.end(); iter ++){
			//if(msg.isIdExisted(iter->first) || iter->second.isObjectTimeExpired())continue;
			if(msg.isIdExisted(iter->first) == true){
				if(iter->second.time_to_live > msg.objectlist[iter->first].time_to_live)
					msg.objectlist[iter->first].time_to_live = iter->second.time_to_live;
				else
					iter->second.time_to_live = msg.objectlist[iter->first].time_to_live;
				//msg.objectlist[iter->first].time_to_live = iter->second.time_to_live;
				continue;
			}
			//if (iter->second.time_to_live <= 0)continue;//by 0729
			double dist = Grid::caculateRoadNetworkDist(msg.getQueryTrajectory(), trajectory(iter->second.eid, iter->second.getLoc()));
			if(MathFun::D(dist - msg.range) <= 0){
				msg.insertNewObject(iter->second);
				if (msg_sig.find(iter->second.eid) == msg_sig.end())msg_sig.insert(iter->second.eid);
			}
		}
		/*for(map<int, vector<pair<double, double> > >::iterator iter = local_cache.roadSegments.begin(); iter != local_cache.roadSegments.end(); iter ++){
			if (msg_sig.find(iter->first) != msg_sig.end())msg.updateRoadSegments(iter->first, iter->second);
			else
				if(local_cache.edge_signature[iter->first] == 0)msg.updateRoadSegments(iter->first, iter->second);
		}*/
		map<int, vector<pair<double, double> > > roadSegments = local_cache.getRoadSementsCache(msg.getQueryTrajectory(), msg.range, nodemap, edgemap);
		for(map<int, vector<pair<double, double> > >::iterator iter = roadSegments.begin(); iter != roadSegments.end(); iter ++){
			if (msg_sig.find(iter->first) != msg_sig.end())msg.updateRoadSegments(iter->first, iter->second);
			else
				if(local_cache.edge_signature[iter->first] == 0)msg.updateRoadSegments(iter->first, iter->second);
			//msg.updateRoadSegments(iter->first, iter->second);
		}
	}
}

void vehicle::reSet()
{
		//queue<trajectory> empty;
		//swap ( trajectorys, empty );
		trajectorys = trajectory();// load changes at 5.30
		local_cache.reSet();
		query_msg_queue.clear();
		query_state = 0;
		last_trajectory = trajectory();
}

void vehicle::updateVehiclesfromRoadsideunit(Roadsideunit * rsumap, Grid& mainGrid, overhead& rsu_statistic)
{
	vector<pair <double, int> > CandidateObj;
	vector<pair <double, int> > CandidateRsu;
	set<int> obj_signature;
	map<int, RsuObj> tObj;
	set<int> candidate;
	int temp = RSUOBJBIT * PUSH_NUM + HEADER;
	pair<int, int> pos = Grid::getLocFromGeo(getCurrentTrajectory().location);
	int tRsu = (unsigned int)mainGrid.gridCell[pos.first][pos.second].Rsu.size();
	if (tRsu != 0){
		for (int k = 0; k < tRsu; k ++){
			int rsuid = mainGrid.gridCell[pos.first][pos.second].Rsu[k] - 1;
			if(rsumap[rsuid].BroBegin == false)continue;
			double mindist = Grid::calculateDist(getCurrentPosition(), rsuLoc[rsuid]);
			if(MathFun::D(mindist - Grid::COMRANGE) > 0)continue;
			CandidateRsu.push_back(make_pair(mindist, rsuid));
			//printf("# %lf %d\n", mindist, rsuid);
		}
	}
	int SelectRsu = 0;
	if(CandidateRsu.empty() == true)return;
	if((int)CandidateRsu.size() > 1){
		sort(CandidateRsu.begin(), CandidateRsu.end(), rsuSort());
	}
	SelectRsu = CandidateRsu[0].second;
	if ((!rsumap[SelectRsu].objects.empty())){
		vector<RsuObj> tRsuObject;
		rsumap[SelectRsu].PushObjectstoVehicle(tRsuObject);
		for (int i = 0; i < tRsuObject.size(); i ++){
			if(tObj.find(tRsuObject[i].nid) == tObj.end())
				tObj[tRsuObject[i].nid] = tRsuObject[i];
		}
	}
	rsu_statistic.power += 0.5 * temp / 8 + 56;
	rsu_statistic.transtime += 1.0 * temp/ RSUBIT;

	for (map<int, RsuObj>::iterator iter = tObj.begin(); iter != tObj.end();)
	{
		if (iter->first >= EMPTY_EDGE)
		{
			local_cache.updateRoadSegment(iter->second.eid, make_pair(iter->second.roadSegments.lat, iter->second.roadSegments.lng));
			if(local_cache.edge_signature.find(iter->second.eid) == local_cache.edge_signature.end())
				local_cache.edge_signature[iter->second.eid] = 0;
			tObj.erase(iter ++);
		}
		else{
			if(local_cache.isIdExisted(iter->first) == false){
				double dist = Grid::caculateRoadNetworkDist(getCurrentTrajectory(), trajectory(iter->second.eid, iter->second.lat, iter->second.lng));
				CandidateObj.push_back(make_pair(dist, iter->first));
			}
			else{
				if (local_cache.objects[iter->first].time_to_live < iter->second.time_to_live){
					local_cache.objects[iter->first].time_to_live = iter->second.time_to_live;
				}
			}
			iter ++;
		}
	}
	if((int)tObj.size() + local_cache.objects.size() <= Grid::CASIZE){
		for(map<int, RsuObj>::iterator iter = tObj.begin(); iter != tObj.end(); iter++){
			if(local_cache.isIdExisted(iter->first) == false){
				local_cache.insertNewNode(node(iter->first, iter->second.eid, iter->second.lat, iter->second.lng, iter->second.time_to_live));
				local_cache.updateEdgesig(iter->second.eid);
			}
			local_cache.updateRoadSegment(tObj[iter->first].eid, make_pair(tObj[iter->first].roadSegments.lat, tObj[iter->first].roadSegments.lng));
		}
	}
	else{
		for (tr1::unordered_map<int, node>::iterator iter = local_cache.objects.begin(); iter != local_cache.objects.end(); iter ++)
		{
			double dist = Grid::caculateRoadNetworkDist(getCurrentTrajectory(), trajectory(iter->second.eid, iter->second.lat, iter->second.lng));
			CandidateObj.push_back(make_pair(dist, iter->first));
		}
		sort(CandidateObj.begin(), CandidateObj.end(), rsuSort());
		for (int i = 0; i < MIN(Grid::CASIZE, CandidateObj.size()); i ++)
		{
			int tid = CandidateObj[i].second;
			candidate.insert(tid);
		}
		CandidateObj.clear();
		for (tr1::unordered_map<int, node>::iterator iter = local_cache.objects.begin(); iter != local_cache.objects.end();){
			if (candidate.find(iter->first) == candidate.end())
			{
				int teid = iter->second.eid;
				local_cache.edge_signature[teid] --;
				if (local_cache.edge_signature[teid] == 0)
				{
					local_cache.roadSegments.erase(teid);
					local_cache.edge_signature.erase(teid);
				}
				if(local_cache.edge_signature[teid] < 0)printf("Oops, edge_signature < 0 at RSU\n");
				local_cache.objects.erase(iter ++);
			}
			else{
				obj_signature.insert(iter->first);
				iter ++;
			}
		}
		for (map<int, RsuObj>::iterator iter = tObj.begin(); iter != tObj.end(); iter ++)
		{
			if (candidate.find(iter->first) != candidate.end())
			{
				if (obj_signature.find(iter->first) != obj_signature.end())
					local_cache.updateRoadSegment(tObj[iter->first].eid, make_pair(tObj[iter->first].roadSegments.lat, tObj[iter->first].roadSegments.lng));
				else{
					local_cache.insertNewNode(node(iter->first, iter->second.eid, iter->second.lat, iter->second.lng, iter->second.time_to_live));
					local_cache.updateEdgesig(iter->second.eid);
					local_cache.updateRoadSegment(tObj[iter->first].eid, make_pair(tObj[iter->first].roadSegments.lat, tObj[iter->first].roadSegments.lng));
				}
			}
		}
	}
	return;
}
/***********************************************************************/
/*          GridCell                                                     */
/************************************************************************/
void GridCell::reSet()
{
	Veh.clear();
}
GridCell::GridCell()
{
}
/************************************************************************/
/*          Grid                                                            */
/************************************************************************/

void Grid::reSet()
{
	for(int i = 0; i < GRID_H; i ++)
		for(int j = 0; j < GRID_W; j ++)
		{
			gridCell[i][j].reSet();
		}
}

pair<int, int> Grid::getLocFromGeo( double x, double y )
{
	return getLocFromGeo(loc(x, y));
}

pair<int, int> Grid::getLocFromGeo( loc pos )
{
	int x = (int)((pos.lat - trans_low.lat) / (trans_high.lat - trans_low.lat) * GRID_H);
	int y = (int)((pos.lng - trans_low.lng) / (trans_high.lng - trans_low.lng) * GRID_W);
	//cout << "x" << x << "y" << y << endl;
	return make_pair(x, y);
}

double Grid::calculateCircleDist(loc u, loc v)
{
	return calculateCircleDist(u.lat, u.lng, v.lat, v.lng);
}

double Grid::calculateCircleDist( double startLat, double startLng, double endLat, double endLng )
{
	double ratio = 0.01745327; // pi/180 
	double x = (endLng*ratio - startLng*ratio)*cos((startLat*ratio +endLat*ratio)/2);
	double y=(endLat*ratio-startLat*ratio);
	double circleDist = 6372800.0 * sqrt(x*x+y*y);
	//circleDist = circleDist<0 ? -circleDist : circleDist;
	return circleDist;
}

double Grid::calculateDist( loc u, loc v )
{
	return calculateDist(u.lat, u.lng, v.lat, v.lng);
}

double Grid::calculateDist( double startLat, double startLng, double endLat, double endLng )
{
	/*double pi = atan2(1.0, 1.0) * 4;
	//scale back to radians
	double startLatInRadians = startLat*pi/180.0;
	double endLatInRadians = endLat*pi/180.0;
	double startLngInRadians = startLng*pi/180.0;
	double endLngInRadians = endLng*pi/180.0;
	double radians = acos(cos(startLatInRadians)*cos(startLngInRadians)*cos(endLatInRadians)*
		cos(endLngInRadians) + cos(startLatInRadians)*sin(startLngInRadians)*
		cos(endLatInRadians)*sin(endLngInRadians)+sin(startLatInRadians)*sin(endLatInRadians));

	//assume the radius of earth is 6372.8 km
	double dist = 6372800.0 * radians;
	gettimeofday(&eucEnd, 0);
	tcalD += (double)(eucEnd.tv_sec - eucStart.tv_sec + (double)(eucEnd.tv_usec - eucStart.tv_usec) / CLOCKS_PER_SEC);

	return dist<0 ? -dist : dist;*/

	//double ratio = 0.01745327; // pi/180 
	//double x = (endLng*ratio - startLng*ratio)*cos((startLat*ratio +endLat*ratio)/2);
	//double y=(endLat*ratio-startLat*ratio);
	//double circleDist = 6372800.0 * sqrt(x*x+y*y);
	////circleDist = circleDist<0 ? -circleDist : circleDist;
	//return circleDist;
	double x = startLat - endLat;
	double y = startLng - endLng;
	return sqrt(x * x + y * y);
}

bool Grid::is_Inrange( int x, int y )
{
	//return x >= 0 && x < GRID_W && y >= 0 && y < GRID_H;
	return x >= 0 && x < GRID_H && y >= 0 && y < GRID_W;
}

void Grid::add_vehicle( loc lo, int vid, int s_time )
{
	pair<int, int> pos = Grid::getLocFromGeo(lo);
	/*if ( s_time > 8850 ) 
	{
		cout << pos.first << ' ' << pos.second << endl;
		cout << lo.lat << ' ' << lo.lng << endl;
	}*/
	if(gridCell[pos.first][pos.second].Veh.find(vid) == gridCell[pos.first][pos.second].Veh.end()){
		gridCell[pos.first][pos.second].Veh.insert(vid);
	}
	return;
}

void Grid::remove_vehicle( loc lo, int vid )
{
	pair<int, int> pos = Grid::getLocFromGeo(lo);
	//cerr << "pos" << ' ' << pos.first << ' ' << pos.second << endl;
	if(gridCell[pos.first][pos.second].Veh.find(vid) != gridCell[pos.first][pos.second].Veh.end()){
		gridCell[pos.first][pos.second].Veh.erase(vid);
	}
}

set<int> Grid::getAllVehicleInCell( loc lo )
{
	pair<int, int> pos = Grid::getLocFromGeo(lo);
	return getAllVehicleInCell(pos);
}

void Grid::getAllVehicleInCell( int x, int y, set<int>*& ids )
{
	ids = &gridCell[x][y].Veh;
}

set<int> Grid::getAllVehicleInCell( pair<int, int> pos )
{
	return gridCell[pos.first][pos.second].Veh;
}

set<int> Grid::getAllObjectsInCell( int x, int y )
{
	return gridCell[x][y].Node;
}

void Grid::getAllObjectsInCell( int x, int y, set<int>*& ids )
{
	ids = &gridCell[x][y].Node;
}

set<int> Grid::getAllObjectsInCell( loc lo )
{
	pair<int, int> pos = Grid::getLocFromGeo(lo);
	return getAllVehicleInCell(pos);
}

set<int> Grid::getAllObjectsInCell( pair<int, int> pos )
{
	return gridCell[pos.first][pos.second].Node;
}

double Grid::getRoadNetworkDist( int idx, int idy )
{
	
	LL id = (LL)idx * NODE_NUM + idy;
	//printf("%lld\n", id);
	if(roadNetworkDist.find(id) != roadNetworkDist.end()){
		return roadNetworkDist[id];
	}
	id = (LL)idy * NODE_NUM + idx;
	if(roadNetworkDist.find(id) != roadNetworkDist.end()){
		return roadNetworkDist[id];
	}
	return RoadDistCal + 1;
}

void Grid::initRoadNetworkDist(map<int, vector<pair<int, int> > >& nodeToEdgemap, node* nodemap)
{
	int count = 0;
	for(map<int, vector<pair<int, int> > > :: iterator iter = nodeToEdgemap.begin(); iter != nodeToEdgemap.end(); iter ++){
		if(count % 100 == 0)cerr << ".";
		if(count % 2000 == 0)cerr << endl;
		count ++;
		queue<int> Q;
		Q.push(iter->first);
		addRoadNetworkDist(iter->first, iter->first, 0.0);
		while(!Q.empty()){
			int top = Q.front(); Q.pop();
			vector<pair<int, int> >* adj = &(nodeToEdgemap[top]);
			node* node1 = &(nodemap[top]);
			double dist1 = getRoadNetworkDist(iter->first, top);
			for(int i = 0; i < adj->size(); i++){
				node* node2 = &(nodemap[(*adj)[i].second]);
				double dist2 = Grid::calculateDist(node1->getLoc(), node2->getLoc());
				if(dist1 + dist2 >= RoadDistCal)continue;
				if(getRoadNetworkDist(iter->first, (*adj)[i].second) <= dist1 + dist2)continue;
				Q.push((*adj)[i].second);
				addRoadNetworkDist(iter->first, (*adj)[i].second, dist1 + dist2);
			}
		}
	}
}

double Grid::shortestPathDist(int idx, int idy, map<int, vector<pair<int, int> > >& nodeToEdgemap, node* nodemap)
{
	double shortestDist[NODE_NUM];
	set<int> cnt;
	int minid;
	for (int i = 0; i < NODE_NUM; i ++)
	{
		shortestDist[i] = 1e50;
	}
	shortestDist[idx] = 0;

	while (cnt.size() != NODE_NUM)
	{
		double minDist = 1e50;
		for (int i = 0; i < NODE_NUM; i ++)
		{
			if (cnt.find(i) == cnt.end())
			{
				if (shortestDist[i] < minDist)
				{
					minDist = shortestDist[i];
					minid = i;
				}
			}
		}
		if (minDist >= 1e50)return minDist;
		if (minid == idy)return minDist;
		cnt.insert(minid);
		vector<pair<int, int> >* neighbor = &(nodeToEdgemap[minid]);
		for(int i = 0; i < neighbor->size(); i++){
			node* tnode = &(nodemap[(*neighbor)[i].second]);
			double len = Grid::calculateDist(nodemap[minid].getLoc(), tnode->getLoc());
			if (shortestDist[minid] + len < shortestDist[tnode->nid])
			{
				shortestDist[tnode->nid] = shortestDist[minid] + len;
			}
		}
	}
	return shortestDist[idy];
}

void Grid::addRoadNetworkDist( int idx, int idy, double dist )
{
	LL id = idx * NODE_NUM + idy;
	roadNetworkDist[id] = dist;
	//printf("%d %d %.8lf", idx, idy, dist);
}

double Grid::caculateRoadNetworkDist( trajectory a, trajectory b)
{
	edge e1 = edgemap[a.eid];
	edge e2 = edgemap[b.eid];
	node nu[2] = {nodemap[e1.startnid], nodemap[e1.endnid]};
	node nv[2] = {nodemap[e2.startnid], nodemap[e2.endnid]};
	double dist = RoadDistCal + 1;
	if(a.eid == b.eid){
		dist = min(dist, Grid::calculateDist(a.location, b.location));
	}
	for(int i = 0; i < 2; i++){
		double tt = Grid::calculateDist(nu[i].getLoc(), a.location);
		double tmp = tt;
		if(tmp >= RoadDistCal)continue;
		for(int j = 0; j < 2; j++){
			tmp += Grid::calculateDist(nv[j].getLoc(), b.location);
			if(tmp >= RoadDistCal)continue;
			tmp += getRoadNetworkDist(nu[i].nid, nv[j].nid);
			if(tmp < dist) dist = tmp;
			tmp = tt;
		}
	}
	return dist;
}

double Grid::caculateRoadNetworkDist( trajectory& a, int b )
{
	edge e1 = edgemap[a.eid];
	node nu[2] = {nodemap[e1.startnid], nodemap[e1.endnid]};
	double dist = RoadDistCal;
	for(int i = 0; i < 2; i++){
		double tmp = Grid::calculateDist(nu[i].getLoc(), a.location);
		if(tmp >= RoadDistCal)continue;
		tmp += getRoadNetworkDist(nu[i].nid, b);
		if(tmp < dist)dist = tmp;
	}
	return dist;
}

vector<pair<double, double> > Grid::getCoveredRoadSegment( trajectory a, int eid, double range )
{
	edge e1 = edgemap[eid];
	node nu = nodemap[e1.startnid];
	node nv = nodemap[e1.endnid];
	double dist1 = caculateRoadNetworkDist(a, trajectory(eid, nu.getLoc()));
	double dist2 = caculateRoadNetworkDist(a, trajectory(eid, nv.getLoc()));
	double road_len = calculateDist(nu.getLoc(), nv.getLoc());
	//bool is_reverse = false;
	vector<pair<double, double> > ans;
	pair<double, double> ans1, ans2;
	if( a.eid == eid ){
		ans.push_back(make_pair( max(0.0, dist1 - range), min(road_len - dist2 + range, road_len)));
		return ans;
	}
	if(MathFun::D(dist1 - range) > 0){
		ans1 = make_pair(-1, -1);
	}else{
		ans1 = make_pair(0, min(range - dist1, road_len));
	}
	if(MathFun::D(dist2 - range) > 0){
		ans2 = make_pair(-1, -1);
	}else{
		ans2 = make_pair(0, range - dist2);
		ans2 = make_pair(road_len - ans2.second, road_len - 0);
		if(ans1.first == -1){
			ans.push_back(ans2);
		}
		else{
			if(MathFun::D(ans1.second - ans2.first) >= 0){
				ans.push_back(make_pair(ans1.first, max(ans1.second , ans2.second)));
			}else{
				ans.push_back(ans1);
				ans.push_back(ans2);
			}
		}
		return ans;
	}
	return ans;
}

set<int> Grid::knnQueryInGrid( trajectory tra, int kNum, double r )
{
	priority_queue<cDist> Q;
	priority_queue<neigh> Obj;
	set<int> cnt;
	set<int> ans;
	set<int> edgecnt;
	double d = 0.0;
	int tempeid = tra.eid;
	int tstart = edgemap[tempeid].startnid;
	int tend = edgemap[tempeid].endnid;
	double len = Grid::getRoadNetworkDist(tstart, tend);
	double dist_s = Grid::calculateDist(tra.location, nodemap[tstart].getLoc());
	double dist_e = Grid::calculateDist(tra.location, nodemap[tend].getLoc());
	Q.push(cDist(tstart, tempeid, dist_s));
	Q.push(cDist(tend, tempeid, dist_e));

	while (!Q.empty()){
		int id = Q.top().id;
		int eid = Q.top().eid;
		double length = Q.top().dist;
		//printf("dist pop %d %lf\n", eid, length);
		Q.pop();
		if(cnt.find(id) != cnt.end()){
			//if(eid == 297)printf("yes\n");
			if(edgecnt.find(eid) != edgecnt.end())continue;
		}
		else
			cnt.insert(id);

		if(edgecnt.find(eid) == edgecnt.end()){
			edgecnt.insert(eid);
			vector<int> ids = Grid::objectsOnEdgeMap[eid];
			for(int i = 0; i < ids.size(); i++){
				node tnode = nodemap[ids[i]];
				trajectory ttra = trajectory(eid, tnode.getLoc());
				double dist = Grid::caculateRoadNetworkDist(tra, ttra);
				if(MathFun::D(dist - r) > 0)continue;
				Obj.push(neigh(tnode.nid, dist));
				//printf("& %d %lf %lf\n", tnode.nid, dist, r);
			}
		}
		if(length >= r)continue;
		vector<pair<int, int> > *tmpEdgepair = &((*Grid::nodeToEdgemap)[id]);
		for(int i = 0; i < tmpEdgepair->size(); i++){
			//if(cnt.find((*tmpEdgepair)[i].second) != cnt.end())continue;
			double tmp = Grid::getRoadNetworkDist(id, (*tmpEdgepair)[i].second);
			Q.push(cDist((*tmpEdgepair)[i].second, (*tmpEdgepair)[i].first, length + tmp));
		}
	}
	while (!Obj.empty()){
		neigh top = Obj.top();
		Obj.pop();
		if(ans.size() >= kNum && MathFun::D(top.dist - d) > 0)return ans;
		ans.insert(top.id);
		if (ans.size() == kNum) d = top.dist;
		//cout << "std ans" << top.id << "#" << ' ' << top.dist << endl;// for test at 6.1
	}
	return ans;
}

void Grid::initObjectOnEdge()
{
	for(int nodeId = 0; nodeId < NODE_NUM; nodeId ++){
		if(nodemap[nodeId].isObject()){
			int edgeId = nodemap[nodeId].eid;
			Grid::objectsOnEdgeMap[edgeId].push_back(nodeId);
		}
	}
}
bool Grid::testSelectPeers(double _vx, double _vy, loc _pos, double& t)
{
	double x = 6;
	double y = 2;
	//printf("%lf %lf %lf %lf\n", x, y, last_trajectory.location.lat, last_trajectory.location.lng);
	double vx = (x - 3) / 0.1;//0.1s time step
	double vy = (y - 1) / 0.1;
	//printf("%lf %lf %lf %lf %lf %lf\n", _vx, _vy, vx, vy, _vx * _vx + _vy * _vy, vx * vx + vy * vy);
	double A = _vx - vx;
	double B = _pos.lat - x;
	double C = _vy - vy;
	double D = _pos.lng - y;
	//printf("* %lf %lf %lf %lf\n", A, B, C, D);
	double a = A * A + C * C;
	double b = 2 * (A * B + C * D);
	double c = B * B + D * D - 5 * 5;//r = 5
	if(MathFun::D(c) > 0)return true;
	//printf("%lf %lf %lf\n", a, b, c);
	if(MathFun::D(a) == 0){
		t = PERIOD;
	}
	else{
		double delat = b * b - 4 * a * c;
		if(MathFun::D(delat) > 0){
			double sqrt_delat = sqrt(delat);
			double tmp = a + a;
			double t1 = (-1 * b + sqrt_delat) / tmp;
			double t2 = (-1 * b - sqrt_delat) / tmp;
			t = MAX(t1, t2) * 10;
			printf("%lf %lf %lf\n", t1, t2, t);
			//if(MathFun::D(c / a) > 0){
			//	printf("Oops %lf %lf %lf\n", t1, t2, c);
			//	//printf("%lf %lf %lf %lf %lf %lf\n", _vx, _vy, vx, vy, _vx * _vx + _vy * _vy, vx * vx + vy * vy);
			//}
		}
	/*	else
			printf("%lf\n", delat);*/
	}
	return true;
}
//int Grid::TTL = 0;
int Grid::KNUM = 0;
bool Grid::testExchange = false;
bool Grid::RSU_SWITCH = false;
int Grid::RANGE = 0;
double Grid::COMRANGE = 0;
int Grid::SPEED = 0;
int Grid::KnnTime = 0;
int Grid::TIME = 0;
int Grid::FILLRSU = 0;
int Grid::BRORSU = 0;
int Grid::BRO_PERIOD = 0;
int Grid::FRESH = 0;
int Grid::CASIZE = 0;
int Grid::RSU_N = 0;
double Grid::COVERAGE = 0;
double Grid::PERIOD = 0;
int Grid::TOP = 0;
map<int, vector<int> > Grid::objectsOnEdgeMap;
int Grid::TYPE = 0;
char* Grid::NAME = NULL;
//loc Grid::high = loc(39.95, 116.445);

//loc Grid::low = loc(39.865, 116.34);
loc Grid::high = loc(39.948, 116.438);
loc Grid::low = loc(39.915, 116.395);

loc Grid::trans_high = loc(0, 0);
loc Grid::trans_low = loc(0, 0);

hash_map Grid::roadNetworkDist;
traffic_map Grid::traffic;

map <int, object> * Grid::objectmap = NULL;

vehicle * Grid::vehiclemap = NULL;

map <int, vector<pair<int, int> > > * Grid::nodeToEdgemap = NULL;

map <int, vector<pair<int, int> > > * Grid::edgeOfGrids = NULL;

map <int, pair<double, double> > * Grid::nodeToDistmap = NULL;

set<int> * Grid::emptyEdgemap = NULL;

edge * Grid::edgemap = NULL;

node * Grid::nodemap = NULL;
loc * Grid::rsuLoc = NULL;

/************************************************************************/
/* loc                                                                     */
/************************************************************************/
loc::loc()
{

}

loc::loc( double _lat, double _lng )
{
	lat = _lat;
	lng = _lng;
}

bool loc::operator==( const loc& b ) const
{
	if(MathFun::D(lat - b.lat) == 0 && MathFun::D(lng - b.lng) == 0)return true;
	return false;
}
/************************************************************************/
/* line                                                                */
/************************************************************************/
line::line()
{

}
line::line(loc _u, loc _v){
	u = _u;
	v = _v;
}
/************************************************************************/
/* circle                                                                */
/************************************************************************/
circle::circle()
{

}
circle::circle(loc _c, double _r)
{
	c = _c;
	r = _r;
}
/************************************************************************/
/* Mathfun                                                                     */
/************************************************************************/
int MathFun::D( double v )
{
	return v < -eps ? -1 : v > eps; 
}

double MathFun::Det(loc a, loc b, loc c)
{
	return (b.lat - a.lat) * (c.lng - a.lng) - (c.lat - a.lat) * (b.lng - a.lng);
}

double MathFun::innerProduct(loc a, loc b, loc c, loc d)// ab, cd
{
	return (b.lat - a.lat) * (d.lat - c.lat) + (b.lng - a.lng) * (d.lng - c.lng);
}

bool MathFun::oppoSide(loc u1, loc u2, loc v1, loc v2)
{
	return D(Det(u1, v1, v2) * Det(u2, v1, v2)) < 0;
}

bool MathFun::isInsideRect(loc x, loc* rect)
{
	if(D(Det(x, rect[0], rect[1]) * Det(x, rect[3], rect[2])) > 0)return false;
	if(D(Det(x, rect[0], rect[3]) * Det(x, rect[1], rect[2])) > 0)return false;
	return true;
}

bool MathFun::isCross(loc u1, loc u2, loc v1, loc v2)
{
	return oppoSide(u1, u2, v1, v2) && ( D(Det(v1, u1, u2) * Det(v2, u1, u2)) <= 0 ) || ( D(Det(u1, v1, v2) * Det(u2, v1, v2)) <= 0 ) && oppoSide(v1, v2, u1, u2);
}

bool MathFun::isSegmentIntersectWithRect(line l, loc* rect)
{
	for(int i = 0; i < 4; i++){
		if(isCross(l.u, l.v, rect[i], rect[(i + 1) % 4])){
			return true;
		}
	}
	return false;
}

bool MathFun::isSegmentInterOrInsideWithRect(line l, loc* rect)
{
	if(isInsideRect(l.u, rect) || isInsideRect(l.v, rect) || isSegmentIntersectWithRect(l, rect)){
		return true;
	}
	return false;
}

double MathFun::dis(loc a, loc b)
{
	double t1 = a.lat - b.lat;
	double t2 = a.lng - b.lng;
	return sqrt(t1 * t1 + t2 * t2);
}

loc MathFun::lineCrossLine(const loc& a, const loc& b, const loc& c, const loc& d)
{
	//printf("%lf %lf %lf %lf\n", a.lat, a.lng, b.lat, b.lng);
	loc ret = a;
	double t =  ((c.lat - a.lat) * (d.lng - c.lng) - (c.lng - a.lng) * (d.lat - c.lat))/
		((b.lat - a.lat) * (d.lng - c.lng) - (b.lng - a.lng) * (d.lat - c.lat));
	ret.lat += (b.lat - a.lat) * t;
	ret.lng += (b.lng - a.lng) * t;
	//if(ret.lat < Grid::trans_low.lat|| ret.lat > Grid::trans_high.lat)printf("lat wrong %lf\n", ret.lat);
	//if(ret.lng < Grid::trans_low.lng|| ret.lng > Grid::trans_high.lng)printf("lng wrong %lf\n", ret.lng);
	//printf("@ %lf %lf\n", ret.lat, ret.lng);
	//if(ret.lat < Grid::trans_low.lat|| ret.lat > Grid::trans_high.lat){
		//printf("@ %lf %lf\n", ret.lat, ret.lng);
		//printf("@ %lf %lf %lf %lf %lf\n", b.lat, b.lng, ret.lat, ret.lng, dis(b, ret));
		//printf("start %lf %lf end %lf %lf\n", c.lat, c.lng, d.lat, d.lng);
	//}
	//if(ret.lng < Grid::trans_low.lng|| ret.lng > Grid::trans_high.lng)printf("& %lf %lf\n", ret.lat, ret.lng);
	return ret;
}
double MathFun::point_seg_dist(loc p, loc s0, loc s1) {
	loc v_s0_p = loc(p.lat - s0.lat, p.lng - s0.lng);
	loc v_s0_s1 = loc(s1.lat - s0.lat, s1.lng - s0.lng);
	loc v_s1_p = loc(p.lat - s1.lat, p.lng - s1.lng);
	loc v_s1_s0 = loc(s0.lat - s1.lat, s0.lng - s1.lng);
	if ( fabs(Det(s0, p, s1)) > 0 && fabs(Det(s1, p, s0)) > 0){
		//printf("! %lf %lf %lf %lf\n", innerProduct(s0, p, s0, s1), innerProduct(s1, p, s1, s0), Det(p, s0, s1), dis(s0, s1));
		return fabs(Det(p, s0, s1)) / dis(s0, s1);
	}
	else
		return min(dis(p, s0), dis(p, s1));
}

void MathFun::lineCrossCircle(const circle& b, const loc& l1, const loc& l2, loc& p1, loc& p2)
{
	loc p = b.c;
	double t;
	p.lat += l1.lng - l2.lng;
	p.lng += l2.lat - l1.lat;
	p = lineCrossLine( p, b.c, l1, l2);
	double t1 = p.lat - b.c.lat;
	double t2 = p.lng - b.c.lng;
	t = sqrt(b.r * b.r - t1 * t1 - t2 * t2) / dis(l1, l2);
	double t3 = l2.lat - l1.lat;
	double t4 = l2.lng - l1.lng;
	p2.lat = p.lat + t3 * t;
	p2.lng = p.lng + t4 * t;
	p1.lat = p.lat - t3 * t;
	p1.lng = p.lng - t4 * t;
};
const double MathFun::eps = 1e-3;

/************************************************************************/
/* objectCmp                                                                     */
/************************************************************************/
//objectCmp::objectCmp()
//{
//
//}
//
//objectCmp::objectCmp(trajectory _query_tra)
//{
//	query_tra = _query_tra;
//}
//
//bool objectCmp::operator()( const node &x, const node &y )
//{
//	double dist1 = Grid::caculateRoadNetworkDist(query_tra, trajectory(x.eid, loc(x.lat, x.lng)));
//	double dist2 = Grid::caculateRoadNetworkDist(query_tra, trajectory(y.eid, loc(y.lat, y.lng)));
//	return dist1 < dist2;
//}

/************************************************************************/
/* Statistic                                                                    */
/************************************************************************/
Statistic::Statistic()
{
	number = 0;
	msg_size = 0;
	power = 0;
	carry_time = 0;
	transtime = 0;
	accuracy = 0;
	precision = 0;
	completeknn = 0;
	ttl = 0;
	dist = 0;
	range = 0;
	coverage = 0;
}
void Statistic::reSet()
{
	number = 0;
	msg_size = 0;
	power = 0;
	carry_time = 0;
	transtime = 0;
	accuracy = 0;
	precision = 0;
	completeknn = 0;
	ttl = 0;
	dist = 0;
	range = 0;
	coverage = 0;
}
overhead::overhead()
{
	number = 0;
	msg_size = 0;
	power = 0;
	carry_time = 0;
	transtime = 0;
}
void overhead::reSet()
{
	number = 0;
	msg_size = 0;
	power = 0;
	carry_time = 0;
	transtime = 0;
}
msg_statistic::msg_statistic()
{
	number = 0;
	road_size = 0;
	obj_size = 0;
	road_num = 0;
}
void msg_statistic::reSet()
{
	number = 0;
	road_size = 0;
	obj_size = 0;
	road_num = 0;
}
/************************************************************************/
/* Road side unit                                                       */
/************************************************************************/
RsuObj:: RsuObj(int _nid, int _eid, double _lat, double _lng, loc _roadSegments, double _time)
{
	nid = _nid;
	eid = _eid;
	lat = _lat;
	lng = _lng;
	roadSegments = _roadSegments;
	time_to_live = _time; //5s
}

Roadsideunit::Roadsideunit()
{
	num = 0;
	lable = 0;
	mark = 0;
	predict = 0;
	BroBegin = false;
}

void Roadsideunit::PushObjectstoVehicle(vector<RsuObj>& RsuObject)
{
	lable = mark;
	for (int i = 0; i < PUSH_NUM; i ++)
	{
		RsuObject.push_back(objects[lable]);
		//printf("%d\n", lable);
		int t = lable;
		lable = (lable + 1) % ((int)objects.size());
		if(lable == 0 && t != 0){
			break;
		}
	}
	//printf("@%d\n", lable);
}

void Roadsideunit::Rsubroadcast()
{
	mark = num;
	for (int i = 0; i < PUSH_NUM; i ++)
	{
		//printf("%d\n", num);
		int t = num;
		num = (num + 1) % ((int)objects.size());
		if(num == 0 && t != 0){
			break;
		}
	}
	//printf("#%d\t%d\n", (int)objects.size(), num);
}
