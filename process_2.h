#pragma once
#ifndef PROCESS_2_H
#define PROCESS_2_H
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <map>
#include <algorithm>
#include "method.h"

using namespace std;
void process_2(vector<vector<int>> ans_tab, Car *car_set, const int car_size, vector<vector<int>> car_tab, map<int, int> car_set_map, Road *road_set, const int road_size, map<int, int> road_set_map, Cross *cross_set, const int cross_size); 

bool all_car_arrived(Car *car_set, const int car_size);

bool road_is_null(Road *road_set, const int road_size);

void drive_car_in_garage(vector<vector<int>> &ans_tab, Car *car_set, map<int, int> car_set_map, vector<vector<int>> &car_tab, const int car_size, Road *road_set, map<int, int> road_set_map, int system_time);

void put_car_on_prior_place(Road road, vector<int> &ans_vec, vector<int> &car_vec, Car* car_set, map<int, int> car_set_map, Road *road_set, map<int, int> road_set_map);

int search_aim_cross_id(Road road_1, Road road_2);

void reset_all(Road *road_set, int road_size, int system_time);

bool still_wait_car(Car *car_set, int car_size);

Car *find_road_prior_car(Road *road_p, int tmp_cross_id);

bool cross_exist_car(Cross cross);

void move_wait_car(Cross *cross_set, int cross_size, Road *road_set, map<int, int> road_set_map, int road_size);

void move_wait_car_inner(vector<int> &id_vec, vector<int> &l_id_vec, vector<int> &r_id_vec, Road *road_p, Car *car_p_now, Road *road_set, map<int, int> road_set_map);

bool can_not_go_out_cross(vector<int> &id_vec, int s1, Road *road_p, Car *car_p);

void can_go_out_cross(vector<int> &id_vec, vector<int> &l_id_vec, vector<int> &r_id_vec, Road *road_p, Cross *cross, Road *road_set, map<int, int> road_set_map);

void find_and_erase(vector<int> &id_vec, int id);

void run_car_through_cross(vector<int> &id_vec, vector<int> &l_id_vec, vector<int> &r_id_vec, Road *road_p, Car *now_car_p, Road *d_road_p, Road *l_road_p, Road *r_road_p, Road *road_set, map<int, int> road_set_map);

bool not_break_out(Car ***from_cross_mat, int channel);

void search_back_car_turn_end(Road *road, int channel_index, bool on_to); // if on_to is false, means on_from is true

void search_wait_end_car_in_speed_range(Road *road, int v1, int i, int j, bool on_to);

int find_min(vector<int> &id_vec);

bool check_car_has_none_status(Car *car_set, int car_size);

void mark_end_wait(Road *road_set, int road_size);

void make_wait_dir_flag(Road road, Car *car);

void back_wait_car(Car ***cross_mat, int i, int j, Road road);

bool back_end_car(Car ***cross_mat, int i, int j, int length);

int back_end_car_dis(Car ***cross_mat, int i, int j, int length);

bool no_front_car(Car ***cross_mat, int i, int j, int length);

#endif
