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
/*
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

void Road::point_cross(vector<int> road_vec, map<int, int> cross_set_map, Cross *cross_set)
{
	this->from_cross = &cross_set[cross_set_map[road_vec[4]]];
	this->to_cross = &cross_set[cross_set_map[road_vec[5]]];
}*/
/*bool Cross::check_cross_exists_car()
{
	if (this->road_1_exist_car == 1)
	{
		for (int j = road_1_n_col - 1; j >= 0; j--)
		{
			for (int i = 0; i <= road_1_n_row - 1; i++)
			{
				if (mat_1_in[i][j] != 0)
				{
					mat_1_in[i][j];// car_id
				}
			}
		}
	}
	
}

int Cross::get_road_4_priorest_car_id(int **mat_in, int n_row, int n_col)
{

	if (this->road_4_exist_car == 1)
	{
		for (int j = n_col - 1; j >= 0; j++)
		{
			for (int i = 0; i <= n_row - 1; i++)
			{
				if (mat_in[i][j] != 0)
				{
					return mat_in[i][j];
				}
			}
		}
		this->road_4_exist_car = 0;
		return 0;
	}
	
}

int Cross::search_priorest_car_id_in_cross()
{	
	// check road order 4 3 1 2, form small to big
	if (this->road_4_exist_car != 1)
	{
		continue;	
	}
	else
	{
		get_priorest_car_id(this->mat_4_in, this->road_4_n_row, this->road_4_n_col);
	}
	
	
	/*
	// check if car exists in this cross from road1 to road4
	if (check_cross_exists_car() != true)
	{
		return;
	}	

	if (car.on_the_road != 1)
	{
		find_start_road_and_put(car, &garage);
		return;
	}
	// if wait to go
	
	// if prior
	if ()


	car.plan_route;*/
//}

/*void Cross::find_start_road_and_put(Car car, vector<int> &garage)
{
	car.system_time++; // -> 1
	int g_size = garage.size();
	if (car.system_time != car.plan_time)
	{
		return;
	}
	if (this->road_1_id == car.start_road_id)
	{
		if (put_car_mat_1(car))
		{	
			for (int g = 0; g <= g_size - 1; g++) // if put, then erase the car in the garage
			{
				if (garage[g] == car.car_id)
				{
					garage.erase(garage.begin() + g);
					break;
				}
			}
			car.now_road_id = this->road_1_id;
			car.on_the_road = 1;
			car.actual_time = car.plan_time;
			car.now_cross_id = this->cross_id;


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
		}
		else
		{
			car.delay++;
			car.actual_time = car.plan_time + car.delay;
		}
	}
	if (this->road_2_id == car.start_road_id)
	{
		if (put_car_mat_2(car))
		{
			for (int g = 0; g <= g_size - 1; g++) // if put, then erase the car in the garage
			{
				if (garage[g] == car.car_id)
				{
					garage.erase(garage.begin() + g);
					break;
				}
			}
			car.now_road_id = this->road_2_id;
			car.on_the_road = 1;
			car.actual_time = car.plan_time;
		}
		else
		{
			car.delay++;
			car.actual_time = car.plan_time + car.delay;
		}
	}
	if (this->road_3_id == car.start_road_id)
	{
		if (put_car_mat_3(car))
		{
			for (int g = 0; g <= g_size - 1; g++) // if put, then erase the car in the garage
			{
				if (garage[g] == car.car_id)
				{
					garage.erase(garage.begin() + g);
					break;
				}
			}
			car.now_road_id = this->road_3_id;
			car.on_the_road = 1;
			car.actual_time = car.plan_time;
		}
		else
		{
			car.delay++;
			car.actual_time = car.plan_time + car.delay;
		}
	}
	if (this->road_4_id == car.start_road_id)
	{
		if (put_car_mat_4(car))
		{
			for (int g = 0; g <= g_size - 1; g++) // if put, then erase the car in the garage
			{
				if (garage[g] == car.car_id)
				{
					garage.erase(garage.begin() + g);
					break;
				}
			}
			car.now_road_id = this->road_4_id;
			car.on_the_road = 1;
			car.actual_time = car.plan_time;
		}
		else
		{
			car.delay++;
			car.actual_time = car.plan_time + car.delay;
		}
	}
}

bool Cross::put_car_mat_1(Car car)
{
	this->road_1_n_row = this->road_1_vec[3]; //channel //mat_1_in.size();
	this->road_1_n_col = this->road_1_vec[1]; //length
	for (int i = 0; i <= this->road_1_n_row - 1; i++)
	{
		for (int j = this->road_1_n_col - 1; j >= 0; j--)
		{
			if (mat_1_in[i][j] == 0) //find a empty place for car
			{
				this->mat_1_in[i][j] = car.car_id;
				this->road_1_exist_car = 1;
				car.now_cross_id = this->cross_id;
				car.now_x = i;
				car.now_y = j;
				car.now_road_length = this->road_1_vec[1];
				car.now_road_limit_speed = this->road_1_vec[2];
				return true;
			}
		}
	}
	
	// if full, delay
	return false;
}

bool Cross::put_car_mat_2(Car car)
{
	this->road_2_n_row = this->road_2_vec[3]; //channel //mat_1_in.size();
	this->road_2_n_col = this->road_2_vec[1]; //length
	for (int i = 0; i <= this->road_2_n_row - 1; i++)
	{
		for (int j = this->road_2_n_col - 1; j >= 0; j--)
		{
			if (mat_2_in[i][j] == 0) //find a empty place for car
			{
				this->mat_2_in[i][j] = car.car_id;
				this->road_2_exist_car = 1;
				car.now_cross_id = this->cross_id;
				car.now_x = i;
				car.now_y = j;
				car.now_road_length = this->road_2_vec[1];
				car.now_road_limit_speed = this->road_2_vec[2];
				return true;
			}
		}
	}
	// if full, delay
	return false;	
}

bool Cross::put_car_mat_3(Car car)
{
	this->road_3_n_row = this->road_3_vec[3]; //channel //mat_1_in.size();
	this->road_3_n_col = this->road_3_vec[1]; //length
	for (int i = 0; i <= this->road_3_n_row - 1; i++)
	{
		for (int j = this->road_3_n_col - 1; j >= 0; j--)
		{
			if (mat_3_in[i][j] == 0) //find a empty place for car
			{
				this->mat_3_in[i][j] = car.car_id;
				this->road_3_exist_car = 1;
				car.now_cross_id = this->cross_id;
				car.now_x = i;
				car.now_y = j;
				car.now_road_length = this->road_3_vec[1];
				car.now_road_limit_speed = this->road_3_vec[2];
				return true;
			}
		}
	}
	// if full, delay
	return false;
}

bool Cross::put_car_mat_4(Car car)
{
	this->road_4_n_row = this->road_4_vec[3]; //channel //mat_1_in.size();
	this->road_4_n_col = this->road_4_vec[1]; //length
	for (int i = 0; i <= this->road_4_n_row - 1; i++)
	{
		for (int j = this->road_4_n_col - 1; j >= 0; j--)
		{
			if (mat_4_in[i][j] == 0) //find a empty place for car
			{
				this->mat_4_in[i][j] = car.car_id;
				this->road_4_exist_car = 1;
				car.now_cross_id = this->cross_id;
				car.now_x = i;
				car.now_y = j;
				car.now_road_length = this->road_4_vec[1];
				car.now_road_limit_speed = this->road_4_vec[2];
				return true;
			}
		}
	}
	// if full, delay
	return false;
}*/


/*
vector<int> Cross::search_road_tab(int road_id)
{
	vector<int>::iterator location_index;
	vector<int> tmp_vec;
	for (int i = 0; i <= this->road_tab.size() - 1; i++)
	{
		tmp_vec = this->road_tab[i];
		location_index = find(tmp_vec.begin(), tmp_vec.end(), road_id);
		if (location_index == tmp_vec.end())
		{
			return tmp_vec;
		}
		tmp_vec.clear();
	}
}
bool ** Cross::construct_cross_prior_mat(vector<int> road_vec)
{
	bool **mat;
	mat = new bool*[road_vec[3]];
	for (int i = 0; i <= road_vec[3] - 1; i++)
	{
		mat[i] = new bool[road_vec[1]];
	}
	for (int i = 0; i <= road_vec[3] - 1; i++)
	{
		for (int j = 0; j <= road_vec[1] - 1; j++)
		{
			if (j == road_vec[1] - 1)
			{
				mat[i][j] = 1; // prior position 
			}
			else
			{
				mat[i][j] = 0;
			}
			
		}
	}
	return mat;
}

int ** Cross::construct_cross_mat(vector<int> road_vec)
{
	int **mat;
	mat = new int*[road_vec[3]];
	for (int i = 0; i <= road_vec[3] - 1; i++)
	{
		mat[i] = new int[road_vec[1]];
	}
	for (int i = 0; i <= road_vec[3] - 1; i++)
	{
		for (int j = 0; j <= road_vec[1] - 1; j++)
		{
			mat[i][j] = 0;
		}
	}
	return mat;
}*/
/*
void Cross::build_all(vector<int> cross_vec)
{
	this->cross_id = cross_vec[0];
	if (cross_vec[1] != -1) // first road
	{
		road_1_id = cross_vec[1];
		this->road_1_vec = search_road_tab(road_1_id);
		if (this->road_1_vec[6] == 1) //is duplex
		{
			this->mat_1_in = construct_cross_mat(this->road_1_vec);
			//this->mat_prior_1 = construct_cross_prior_mat(this->road_1_vec);
		}
		else
		{
			if (this->road_1_vec[6] == 0)
			{
				if (this->road_1_vec[5] == cross_vec[0] ) //to
				{
					this->mat_1_in = construct_cross_mat(this->road_1_vec);
					//this->mat_prior_1 = construct_cross_prior_mat(this->road_1_vec);
					//this->mat_prior
				}
			}
		}		
	}

	if (cross_vec[2] != -1) // second road
	{
		road_2_id = cross_vec[2];
		this->road_2_vec = search_road_tab(road_2_id);
		if (this->road_2_vec[6] == 1) //is duplex
		{
			this->mat_2_in = construct_cross_mat(this->road_2_vec);
			//this->mat_prior_2 = construct_cross_prior_mat(this->road_2_vec);
		}
		else
		{
			if (this->road_2_vec[6] == 0)
			{
				if (this->road_2_vec[5] == cross_vec[0] ) //to
				{
					this->mat_2_in = construct_cross_mat(this->road_2_vec);
					//this->mat_prior_2 = construct_cross_prior_mat(this->road_2_vec);
				}
			}
		}			
	}

	if (cross_vec[3] != -1) // third road
	{
		road_3_id = cross_vec[3];
		this->road_3_vec = search_road_tab(road_3_id);
		if (this->road_3_vec[6] == 1) //is duplex
		{
			this->mat_3_in = construct_cross_mat(this->road_3_vec);
			//this->mat_prior_3 = construct_cross_prior_mat(this->road_3_vec);
		}
		else
		{
			if (this->road_3_vec[6] == 0)
			{
				if (this->road_3_vec[5] == cross_vec[0] ) //to
				{
					this->mat_3_in = construct_cross_mat(this->road_3_vec);
					//this->mat_prior_3 = construct_cross_prior_mat(this->road_3_vec);
				}
			}
		}			
	}

	if (cross_vec[4] != -1) // forth road
	{
		road_4_id = cross_vec[3];
		this->road_4_vec = search_road_tab(road_4_id);
		if (this->road_4_vec[6] == 1) //is duplex
		{
			this->mat_4_in = construct_cross_mat(this->road_4_vec);
			this->mat_prior_4 = construct_cross_prior_mat(this->road_4_vec);
		}
		else
		{
			if (this->road_4_vec[6] == 0)
			{
				if (this->road_4_vec[5] == cross_vec[0] ) //to
				{
					this->mat_4_in = construct_cross_mat(this->road_4_vec);
					//this->mat_prior_4 = construct_cross_prior_mat(this->road_4_vec);
				}
			}
		}			
	}

}*/





#endif
