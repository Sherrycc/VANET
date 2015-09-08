#ifndef __loadOperation_H__
#define __loadOperation_H__
#include "struct.h"

loc coordTrans(double lat, double lng);
void getnodes(node* nodemap, Grid& mainGrid, map<int, vector<node> >& edgeToObjmap);
void getedges(node* nodemap, edge* edgemap, map<int, vector<pair<int, int> > >& nodeToEdgemap,  map<int, vector<node> >& edgeToObjmap, set<int>& emptyEdgemap, Grid& mainGrid, set<int>& edgeInGridmap);
//void getobjects(map<int, object>& objectmap, node* nodemap, edge* edgemap);
void getrsu(Roadsideunit * rsumap, Grid& mainGrid, map<int, vector<node> >& edgeToObjmap, set<int>& emptyEdgemap, edge * edgemap, node * nodemap, map<int, pair<double, double> >& nodeToDistmap, loc * rsuLoc);
void lableRSU(Grid& mainGrid, loc* rsuLoc, int rsu_cover_range, int lat_range, int lng_range);
void getCoverArea(Grid* mainGrid, pair<int, int> center, int rsu_cover_range, int color);
void labelBro(Grid& mainGrid, int rsu_cover_range, int lat_range, int lng_range);

#endif