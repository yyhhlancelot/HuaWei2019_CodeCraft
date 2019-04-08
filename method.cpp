#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <stack>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "method.h"
#include "math.h"
using namespace std;
#define INF 100000
static const int maxWeight = INF;


int Car::search_around_point(int from, int cross_size)
{
	int width = sqrt(cross_size);
	int x = this->cross_id_index_map[from].first;
	int y = this->cross_id_index_map[from].second;
	vector<int> point_restore_vec;
	if ((x - 2 >= 0) && (x + 2 <= width - 1) && (y - 2 >= 0) && (y + 2 <= width - 1)) // middle
	{
		for (int i = x - 2; i <= x + 2; i++)
		{
			for (int j = y - 2; j <= y + 2; j++)
			{
				if (this->cross_map[i][j] != from)
				{
					point_restore_vec.push_back(this->cross_map[i][j]);
				}
			}
		}
	}
	else if ((x - 2 < 0) && (y >= 5)) // upright
	{
		for (int i = 0; i <= 4; i++)
		{
			for (int j = y - 5; j <= y - 1; j++)
			{
				if (this->cross_map[i][j] != from)
				{
					point_restore_vec.push_back(this->cross_map[i][j]);
				}
			}
		}
	}
	else if ((x - 2 < 0) && (y < 5)) // upleft
	{
		for (int i = 0; i <= 4; i++)
		{
			for (int j = y + 1; j <= min(y + 5, width - 1); j++)
			{
				if (this->cross_map[i][j] != from)
				{
					point_restore_vec.push_back(this->cross_map[i][j]);
				}
			}
		}
	}
	else if ((x + 2 > width - 1) && (y >= 5)) // downright
	{
		for (int i = x - 5; i <= x - 1; i++)
		{
			for (int j = y - 5; j <= y - 1; j++)
			{
				if (this->cross_map[i][j] != from)
				{
					point_restore_vec.push_back(this->cross_map[i][j]);
				}
			}
		}
	}
	else if ((x + 2 > width - 1) && (y < 5)) // downleft
	{
		for (int i = x - 5; i <= x - 1; i++)
		{
			for (int j = y + 1; j <= min(y + 5, width - 1); j++)
			{
				if (this->cross_map[i][j] != from)
				{
					point_restore_vec.push_back(this->cross_map[i][j]);
				}
			}
		}
	}
	else if ((y - 2 < 0) && (x >= 5)) // downleft
	{
		for (int i = x - 5; i <= x - 1; i++)
		{
			for (int j = 0; j <= 4; j++)
			{
				if (this->cross_map[i][j] != from)
				{
					point_restore_vec.push_back(this->cross_map[i][j]);
				}
			}
		}
	}
	else if ((y - 2 < 0) && (x < 5)) // upleft
	{
		for (int i = x + 1; i <= min(x + 5, width - 1); i++)
		{
			for (int j = 0; j <= 4; j++)
			{
				if (this->cross_map[i][j] != from)
				{
					point_restore_vec.push_back(this->cross_map[i][j]);
				}
			}
		}
	}
	else if ((y + 2 > width - 1) && (x >= 5)) // downright
	{
		for (int i = x - 5; i <= x - 1; i++)
		{
			for (int j = 0; j <= 4; j++)
			{
				if (this->cross_map[i][j] != from)
				{
					point_restore_vec.push_back(this->cross_map[i][j]);
				}
			}
		}
	}
	else if ((y + 2 > width - 1) && (x < 5)) // upright
	{
		for (int i = x + 1; i <= min(x + 5, width - 1); i++)
		{
			for (int j = 0; j <= 4; j++)
			{
				if (this->cross_map[i][j] != from)
				{
					point_restore_vec.push_back(this->cross_map[i][j]);
				}
			}
		}
	}
	for (int i = 0; i <= point_restore_vec.size() - 1; i++)
	{
		cout << point_restore_vec[i] << " ";
	}
	cout << endl;
	random_shuffle(point_restore_vec.begin(), point_restore_vec.end());
	for (int i = 0; i <= point_restore_vec.size() - 1; i++)
	{
		cout << point_restore_vec[i] << " ";
	}
	cout << endl;
	vector<int> pause_vec = { 27, 35, 18, 11, 2, 1, 25, 33, 9, 3, 10, 26, 34, 19, 17 };
	if (point_restore_vec == pause_vec)
	{
		int a = 0;
	}
	return point_restore_vec[0];
}
void Car::two_path_merge(float **adjacent_mat, const int cross_size, map<pair<int, int>, int> cross_cross_road_map)
{
	int mid = search_around_point(this->from, cross_size);

	vector<int> start_to_mid = Dijkstra_t_single(adjacent_mat, cross_size, cross_cross_road_map, this->from, mid);

	vector<int> mid_to_end = Dijkstra_t_single(adjacent_mat, cross_size, cross_cross_road_map, mid, this->to);
	if (mid_to_end.size() != 0)
	{
		while (mid_to_end[0] == start_to_mid.back())
		{
			mid = search_around_point(this->from, cross_size);
			start_to_mid = Dijkstra_t_single(adjacent_mat, cross_size, cross_cross_road_map, this->from, mid);
			mid_to_end = Dijkstra_t_single(adjacent_mat, cross_size, cross_cross_road_map, mid, this->to);
			if (mid_to_end.size() == 0)
			{
				this->plan_route = start_to_mid;
				return;
			}
		}
		start_to_mid.insert(start_to_mid.end(), mid_to_end.begin(), mid_to_end.end());
		this->plan_route = start_to_mid;
		return;
	}
	else
	{
		this->plan_route = start_to_mid;
		return;
	}
}

vector<int> Car::Dijkstra_t_single(float **adjacent_mat, const int cross_size, map<pair<int, int>, int> cross_cross_road_map, int start, int end)
{
	vector<int> half_route;
	const int maxnum = 2000;
	const int infinity = 10000;
	int count = 0;
	bool s[maxnum];
	int pre[maxnum]; // store forward shortest
	float distance[maxnum]; // store now shortest

	// initialize
	for (int i = 1; i <= cross_size; i++) //from index 1
	{
		distance[i] = adjacent_mat[start][i]; //adjacent mat
		s[i] = 0;
		if (distance[i] == maxWeight)
		{
			pre[i] = 0;
		}
		else
		{
			pre[i] = start;
		}

	}
	distance[start] = 0;
	s[start] = 1;

	for (int i = 2; i <= cross_size; i++)
	{
		int tmp = maxWeight;
		int u = start;
		for (int j = 1; j <= cross_size; j++)
		{
			if ((!s[j]) && (distance[j] < tmp))
			{
				u = j;
				tmp = distance[j];
			}
		}
		s[u] = 1; // have visited
		for (int j = 1; j <= cross_size; j++)
		{
			if ((!s[j]) && (adjacent_mat[u][j] < maxWeight))
			{
				float new_dist = distance[u] + adjacent_mat[u][j]; // no start point and old point because of s[j] 
				if (new_dist < distance[j])
				{
					distance[j] = new_dist;
					pre[j] = u;
				}
			}
		}
		if (u == end)
		{
			break;
		}
	}
	int cross_pre = end;
	while (cross_pre != start)
	{
		half_route.insert(half_route.begin(), cross_cross_road_map[make_pair(pre[cross_pre], cross_pre)]);
		cross_pre = pre[cross_pre];
	}
	return half_route;
}

void Car::Dijkstra_t(float **adjacent_mat, const int cross_size, map<pair<int, int>, int> cross_cross_road_map, vector<vector<int>> cross_tab, map<int, int> cross_index_map, map<int, int> index_cross_map)
{

	const int maxnum = 200;
	const int infinity = 10000;
	int count = 0;
	bool s[maxnum];
	int pre[maxnum]; // store forward shortest
	float distance[maxnum]; // store now shortest
	if (this->car_id == 103420)
	{
		int pause = 0;
	}
	int start = this->from;
	int end = this->to;
	// initialize
	for (int i = 1; i <= cross_size; i++) //from index 1
	{
		distance[i] = adjacent_mat[cross_index_map[start]][i]; //adjacent mat
		s[i] = 0;
		if (distance[i] == maxWeight)
		{
			pre[i] = 0;
		}
		else
		{
			pre[i] = cross_index_map[start];
		}

	}
	distance[cross_index_map[start]] = 0;
	s[cross_index_map[start]] = 1;

	for (int i = 2; i <= cross_size; i++)
	{
		int tmp = maxWeight;
		int u = cross_index_map[start];
		for (int j = 1; j <= cross_size; j++)
		{
			if ((!s[j]) && (distance[j] < tmp)) // find the not visited minimum distance
			{
				u = j;
				tmp = distance[j];
			}
		}
		s[u] = 1; // have visited
		for (int j = 1; j <= cross_size; j++)
		{
			if ((!s[j]) && (adjacent_mat[u][j] < maxWeight))
			{
				float new_dist = distance[u] + adjacent_mat[u][j]; // no start point and old point because of s[j] 
				if (new_dist < distance[j])
				{
					distance[j] = new_dist;
					pre[j] = u;
				}
			}
		}
		if (u == cross_index_map[end])
		{
			break;
		}
	}
	int cross_pre = cross_index_map[end];
	while (cross_pre != cross_index_map[start])
	{
		this->plan_route.insert(plan_route.begin(), cross_cross_road_map[make_pair(index_cross_map[pre[cross_pre]], index_cross_map[cross_pre])]);
		cross_pre = pre[cross_pre];
	}
	delete[] this->time_weight;
}
void Car::Dijkstra(int **adjacent_length_mat, const int cross_size, map<pair<int, int>, int> cross_cross_road_map)
{
	const int maxnum = 2000;
	const int infinity = 10000;
	int count = 0;
	bool s[maxnum];
	int pre[maxnum]; // store forward shortest
	int distance[maxnum]; // store now shortest
	int start = this->from;
	int end = this->to;
	// initialize
	for (int i = 1; i <= cross_size; i++) //from index 1  // i = 0 cross_size - 1
	{
		distance[i] = adjacent_length_mat[start][i]; //adjacent mat // 
		s[i] = 0;
		if (distance[i] == maxWeight)
		{
			pre[i] = 0;
		}
		else
		{
			pre[i] = start;
		}
		
	}
	distance[start] = 0;
	s[start] = 1;

	for (int i = 2; i <= cross_size; i++)
	{
		int tmp = maxWeight;
		int u = start;
		for (int j = 1; j <= cross_size; j++)
		{
			if ((!s[j]) && (distance[j] < tmp))
			{
				u = j;
				tmp = distance[j];
			}
		}
		s[u] = 1; // have visited
		for (int j = 1; j <= cross_size; j++)
		{
			if ((!s[j]) && (adjacent_length_mat[u][j] < maxWeight))
			{
				int new_dist = distance[u] + adjacent_length_mat[u][j]; // no start point and old point because of s[j] 
				if (new_dist < distance[j])
				{
					distance[j] = new_dist;
					pre[j] = u;
				}
			}
		}
		if (u == end)
		{
			break;
		}
	}
	int cross_pre = end;
	while (cross_pre != start)
	{
		this->plan_route.insert(plan_route.begin(), cross_cross_road_map[make_pair(pre[cross_pre], cross_pre)]);
		cross_pre = pre[cross_pre];
	}
}

int Car::find_connect_road(Cross cross_back, Cross cross_pre)
{
	int back_road_1;
	int back_road_2;
	int back_road_3;
	int back_road_4;
	if (cross_back.road_1_p != NULL)
	{
		back_road_1 = cross_back.road_1_id;
	}
	if (cross_back.road_2_p != NULL)
	{
		back_road_2 = cross_back.road_2_id;
	}
	if (cross_back.road_3_p != NULL)
	{
		back_road_3 = cross_back.road_3_id;
	}
	if (cross_back.road_4_p != NULL)
	{
		back_road_4 = cross_back.road_4_id;
	}
	if (cross_pre.road_1_p != NULL)
	{
		if (back_road_1 == cross_pre.road_1_id)
		{
			return back_road_1;
		}
		else if (back_road_2 == cross_pre.road_1_id)
		{
			return back_road_2;
		}
		else if (back_road_2 == cross_pre.road_1_id)
		{
			return back_road_2;
		}
		else if (back_road_2 == cross_pre.road_1_id)
		{
			return back_road_2;
		}

	}
}

void Car::set(vector<int> car_vec, int **Length, int **Speed, int cross_size)
{

	this->car_id = car_vec[0];
	if (this->car_id == 23832)
	{
		int pause = 0;
	}
	this->from = car_vec[1];
	this->to = car_vec[2];
	this->speed = car_vec[3];
	this->plan_time = car_vec[4];
	//ans_vec.erase(ans_vec.begin(), ans_vec.begin() + 2); // 2 means the front 2
	//this->plan_route = ans_vec;


	//this->cross_map = new int*[map_width];
	//for (int i = 0; i <= map_width - 1; i++) // cross map has bug
	//{
	//	this->cross_map[i] = new int[map_width];
	//}
	//int c = 1;
	//for (int j = 0; j <= map_width - 1; j++)
	//{
	//	for (int i = map_width - 1; i >= 0; i--)
	//	{
	//		cross_map[i][j] = c;
	//		this->cross_id_index_map.insert(pair<int, pair<int, int>>(c, make_pair(i, j)));
	//		c++;
	//	}
	//}
	

	//this->time_weight = new float*[cross_size + 1]; // 0 - cross_size
	this->time_weight = new float*[cross_size + 1];
	int sz = cross_size + 1;
	//for (int i = 0; i <= cross_size; i++)
	for (int i = 0; i <= sz - 1; i++)
	{
		//this->time_weight[i] = new float[cross_size + 1];
		this->time_weight[i] = new float[sz];
	}
	//for (int i = 0; i <= cross_size; i++)
	for (int i = 0; i <= sz - 1; i++)
	{
		//for (int j = 0; j <= cross_size; j++)
		for (int j = 0; j <= sz - 1; j++)
		{
			this->time_weight[i][j] = (i == j) ? 0 : maxWeight;
		}
	}
	// time weight matrix
	//for (int i = 1; i <= cross_size; i++)
	for (int i = 1; i <= sz - 1; i++)
	{
		//for (int j = 1; j <= cross_size; j++)
		for (int j = 1; j <= sz - 1; j++)
		{
			if (i != j)
			{
				if (Length[i][j] != maxWeight)
				{
					this->time_weight[i][j] = Length[i][j] / min(Speed[i][j], this->speed);
				}
				else
				{
					this->time_weight[i][j] = maxWeight;
				}
			}
			else
				continue;

		}
	}
}

void Car::point_road(map<int, int> road_set_map, Road *road_set)
{
	this->now_road = &road_set[road_set_map[this->plan_route[0]]];
}

void Car::point_cross()
{
	this->from_cross = now_road->from_cross;
	this->to_cross = now_road->to_cross;
}

void Road::build_all(vector<int> road_vec)
{
	this->road_id = road_vec[0]; 
	this->length = road_vec[1]; 
	this->speed = road_vec[2]; 
	this->channel = road_vec[3];
	this->from = road_vec[4]; 
	this->to = road_vec[5]; 
	this->isDuplex = road_vec[6];
}

void Road::init()
{
	this->to_cross_mat = new Car **[channel];
	this->from_cross_mat = new Car **[channel];

	for (int i = 0; i < channel; i++)
	{
		to_cross_mat[i] = new Car*[length];
		memset(to_cross_mat[i], 0, length * sizeof(Car*)) ;
		from_cross_mat[i] = new Car*[length];
		memset(from_cross_mat[i], 0, length * sizeof(Car*)) ;
	}
	for (int i = 0; i < channel; i++)
	{
		for (int j = 0; j <= length - 1; j++)
		{
			to_cross_mat[i][j] = NULL;
			from_cross_mat[i][j] = NULL;
		}
	}
}

void Road::point_from_to(vector<int> road_vec, Road road, map<int, int> cross_set_map, Cross *cross_set)
{
	if (road.road_id == road.from_cross->road_1_id)
	{
		road.from_cross_mat = cross_set[cross_set_map[road_vec[5]]].road_3_p->to_cross_mat;
	}
	else if (road.road_id == road.from_cross->road_2_id)
	{
		road.from_cross_mat = cross_set[cross_set_map[road_vec[5]]].road_4_p->to_cross_mat;
	}
	else if (road.road_id == road.from_cross->road_3_id)
	{
		road.from_cross_mat = cross_set[cross_set_map[road_vec[5]]].road_1_p->to_cross_mat;
	}
	else if (road.road_id == road.from_cross->road_4_id)
	{
		road.from_cross_mat = cross_set[cross_set_map[road_vec[5]]].road_2_p->to_cross_mat;
	}
	
}

void Road::point_cross(vector<int> road_vec, map<int, int> cross_set_map, Cross *cross_set)
{
	this->from_cross = &cross_set[cross_set_map[road_vec[4]]];
	this->to_cross = &cross_set[cross_set_map[road_vec[5]]];
}


void Cross::point_road(vector<int> cross_vec, map<int, int> road_set_map, Road *road_set)
{
	if (cross_vec[1] != -1)
	{
		road_1_p = &road_set[road_set_map[cross_vec[1]]];
	}
	if(cross_vec[2] != -1)
	{
		road_2_p = &road_set[road_set_map[cross_vec[2]]];
	}
	if(cross_vec[3] != -1)
	{
		road_3_p = &road_set[road_set_map[cross_vec[3]]];
	}
	if(cross_vec[4] != -1)
	{
		road_4_p = &road_set[road_set_map[cross_vec[4]]];
	}
}
void Cross::get_road_tab(vector<vector<int>> road_tab)
{
	this->road_tab = road_tab;
}
void Cross::build_all(vector<int> cross_vec)
{
	this->cross_id = cross_vec[0];
	this->road_1_id = cross_vec[1];
	this->road_2_id = cross_vec[2];
	this->road_3_id = cross_vec[3];
	this->road_4_id = cross_vec[4];
}
