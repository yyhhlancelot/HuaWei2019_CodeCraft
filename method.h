#pragma once
#ifndef METHOD_H
#define METHOD_H
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
using namespace std;

class Car;
class Road;
class Cross;

class Car
{
friend class Road;
friend class Cross;
public:

	
	bool car_pos = 0; // if road mat exist this car
	int system_time = 0;
	int car_id;
	int from; //start cross 
	int to; // dest cross
	int speed;
	int plan_time;

	float **time_weight; // adjcent mat - time
	int **cross_map;
	map<int, pair<int, int>> cross_id_index_map;

	bool arrived = false;

	bool on_to = 0; //1 means on to cross mat
	bool on_from = 0; // 1 means on from cross mat

	int aim_cross_id;
	Cross *aim_cross = NULL; // next cross want to through
	Road *now_road = NULL; //store road pointer

	Cross *to_cross = NULL; //store cross pointer	
	Cross *from_cross = NULL; //store cross pointer	

	bool wait = 0; //can go out the cross -> 1
	bool end = 0; //no obstacle and can not go out the cross -> 1

	bool D = 0; //go straight
	bool L = 0; //turn left
	bool R = 0; //turn right
	int dir = 0; // D = 3; L = 2; R = 1;

	int now_cross_id; // to which cross now
	int now_road_id;
	int now_road_index = 0;
	int now_x;
	int now_y;
	int now_road_length;
	int now_road_limit_speed;
	
	int next_road_id; //always the plan_route[1]	
	
	int actual_time;
	int start_road_id;
	vector<int> plan_route;
	bool on_the_road = 0;
	int delay = 0; //if delay, plan time++
	
	void set(vector<int> car_vec, int **Length, int **Speed, int cross_size);

	void point_road(map<int, int> road_set_map, Road *road_set);

	void point_cross();

	void Dijkstra(int **adjacent_mat, const int cross_size, map<pair<int, int>, int> cross_cross_road_map);

	void Dijkstra_t(float **adjacent_mat, const int cross_size, map<pair<int, int>, int> cross_cross_road_map, vector<vector<int>> cross_tab, map<int, int> cross_index_map, map<int, int> index_cross_map);

	vector<int> Dijkstra_t_single(float **adjacent_mat, const int cross_size, map<pair<int, int>, int> cross_cross_road_map, int start, int end);

	int search_around_point(int from, int cross_size);

	void two_path_merge(float **adjacent_mat, const int cross_size, map<pair<int, int>, int> cross_cross_road_map);

	int find_connect_road(Cross cross_back, Cross cross_pre);
	Car(){};
	~Car(){};

};

class Road
{
friend class Car;
friend class Cross;
public:
	int road_id;
	int length;
	int speed;
	int channel;
	int from;
	int to;
	bool isDuplex;
	Car ***to_cross_mat; // store car pointer
	Car ***from_cross_mat; // store car pointer
	Cross *to_cross = NULL;
	Cross *from_cross = NULL; 
	Car *prior_car = NULL;
	Road()
	{
	}
	void build_all(vector<int> road_vec);

	void init();

	void point_from_to(vector<int> road_vec, Road road, map<int, int> cross_set_map, Cross *cross_set);

	void point_cross(vector<int> road_vec, map<int, int> cross_set_map, Cross *cross_set);

	
	~Road()
	{
		//for (int i = 0; i < channel; i++)
		//{	
		//	delete[] to_cross_mat[i];
		//	delete[] from_cross_mat[i];
		//}
	}

};


class Cross
{
friend class Road;
friend class Car;
public:
	Cross(){};
	~Cross(){};
	int cross_id;	

	Road *road_1_p = NULL;
	int road_1_id;

	Road *road_2_p = NULL;
	int road_2_id;

	Road *road_3_p = NULL;
	int road_3_id;

	Road *road_4_p = NULL;
	int road_4_id;

	vector<int> cross_vec;
	
	void get_road_tab(vector<vector<int>> road_tab);

	void build_all(vector<int> cross_vec); //build 0 road mats
	
	void point_road(vector<int> cross_vec, map<int, int> road_set_map, Road *road_set);

private:
	vector<vector<int>> road_tab;
};

void Cross::point_road(vector<int> cross_vec, map<int, int> road_set_map, Road *road_set)
{
	if (cross_vec[1] != -1)
	{
		road_1_p = &road_set[road_set_map[cross_vec[1]]];
	}	else if(cross_vec[2] != -1)
	{
		road_2_p = &road_set[road_set_map[cross_vec[2]]];
	}
	else if(cross_vec[3] != -1)
	{
		road_3_p = &road_set[road_set_map[cross_vec[3]]];
	}
	else if(cross_vec[4] != -1)
	{
		road_4_p = &road_set[road_set_map[cross_vec[4]]];
	}
}
void Cross::get_road_tab(vector<vector<int>> road_tab)
{
	this->road_tab = road_tab;
}

void Car::set(vector<int> ans_vec, vector<int> car_vec)
{
	this->car_id = car_vec[0];
	this->from = car_vec[1];
	this->to = car_vec[2];
	this->speed = car_vec[3];
	this->plan_time = car_vec[4];
	ans_vec.erase(ans_vec.begin(), ans_vec.begin() + 2); // 2 means the front 2
	this->plan_route = ans_vec;
	this->start_road_id = this->plan_route[0];
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

#endif
