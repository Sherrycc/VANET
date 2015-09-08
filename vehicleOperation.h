#include "struct.h"
#include <ctime>
using namespace std;

void queryingVehicleSelect(int num);
void initial_q_vehicle(vehicle * vehiclemap, edge* edgemap, node* nodemap, int cur_time, Grid& mainGrid, Statistic& statistic);
void initial_w_vehicle(vehicle * vehiclemap, edge* edgemap, node* nodemap, int cur_time, Grid& mainGrid, Statistic& statistic);
void add_to_vtable();
void add_to_otable();
int findneighbor(object k);