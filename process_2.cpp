#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "method.h"
#include "process_2.h"
using namespace std;

extern int count_car = 0;

void process_2(vector<vector<int>> ans_tab, Car *car_set, const int car_size, vector<vector<int>> car_tab, map<int, int> car_set_map, Road *road_set, const int road_size, map<int, int> road_set_map, Cross *cross_set, const int cross_size)
{
	/////*********** build all *********//////


	
	// car point to cross

	/// first put
	

	//////////***** TODO:process ********///////////

	for (int t = 0; ; t++)
	{
		while ((check_car_has_none_status(car_set, car_size)) && (t != 0)) // when t = 0, there is no car, so do not mark
		{
			mark_end_wait(road_set, road_size); // end car was marked and moved at same time
		}

		while ((still_wait_car(car_set, car_size)) && (t != 0))
		{
			move_wait_car(cross_set, cross_size, road_set, road_set_map, road_size);
			//check_dead_lock		
		}	

		reset_all(road_set, road_size, t);
		
		drive_car_in_garage(ans_tab, car_set, car_set_map, car_tab, car_size, road_set, road_set_map, t + 1); 

		if (all_car_arrived(car_set, car_size))
		{
			break;
		}	
	}
	
}
bool all_car_arrived(Car *car_set, const int car_size)
{
	for (int k = 0; k <= car_size - 1; k++)
	{
		if (car_set[k].arrived == false)
		{
			return false;
		}
	}
	return true;
}
bool road_is_null(Road *road_set, const int road_size)
{
	for (int r = 0; r <= road_size - 1; r++)
	{
		for (int i = 0; i <= road_set[r].channel - 1; i++)
		{
			for (int j = 0; j <= road_set[r].length - 1; j++)
			{
				if ((road_set[r].to_cross_mat[i][j] != NULL) || (road_set[r].from_cross_mat[i][j] != NULL))
				{
					return false;
				} 
			}
		}
	}
	return true;
}

void drive_car_in_garage(vector<vector<int>> &ans_tab, Car *car_set, map<int, int> car_set_map, vector<vector<int>> &car_tab, const int car_size, Road *road_set, map<int, int> road_set_map, int system_time)
{
	for (int i = 0; i <= car_size - 1; i++)
	{
		if (ans_tab[i][1] == system_time)
		{
			put_car_on_prior_place(road_set[road_set_map[ans_tab[i][2]]], ans_tab[i], car_tab[i], car_set, car_set_map, road_set, road_set_map);
		}
	} 
}

void put_car_on_prior_place(Road road, vector<int> &ans_vec, vector<int> &car_vec, Car* car_set, map<int, int> car_set_map, Road *road_set, map<int, int> road_set_map)
{
	int car_id = ans_vec[0];
	int aim_cross_id;
	if (car_set[car_set_map[car_id]].plan_route.size() >= 2)
	{
		aim_cross_id = search_aim_cross_id(road_set[road_set_map[car_set[car_set_map[car_id]].plan_route[0]]], road_set[road_set_map[car_set[car_set_map[car_id]].plan_route[1]]]);
		car_set[car_set_map[car_id]].aim_cross_id = aim_cross_id;
	}
	else // plan route size is 1
	{
		aim_cross_id = car_vec[2];
		car_set[car_set_map[car_id]].aim_cross_id = aim_cross_id;
	}
	if (aim_cross_id == road.to_cross->cross_id) // aim is to cross
	{
		for (int j = road.length - 1; j >= 0; j--)
		{
			for (int i = 0; i <= road.channel - 1; i++)
			{
				if (road.to_cross_mat[i][j] == NULL)
				{
					road.to_cross_mat[i][j] = &car_set[car_set_map[car_id]];
					car_set[car_set_map[car_id]].car_pos = true;
					car_set[car_set_map[car_id]].now_x = i;
					car_set[car_set_map[car_id]].now_y = j;
					car_set[car_set_map[car_id]].on_to = true;
					return;
				}
			}
		}
	}
	else // aim is from cross, do not need to worry about duplex, because to cross is first(ans is suitable)
	{
		for (int j = road.length - 1; j >= 0; j--)
		{
			for (int i = 0; i <= road.channel - 1; i++)
			{
				if (road.from_cross_mat[i][j] == NULL)
				{
					road.from_cross_mat[i][j] = &car_set[car_set_map[car_id]];
					car_set[car_set_map[car_id]].car_pos = true;
					car_set[car_set_map[car_id]].now_x = i;
					car_set[car_set_map[car_id]].now_y = j;
					car_set[car_set_map[car_id]].on_from = true;
					return;
				}
			}
		}
	}

	car_set[car_set_map[car_id]].delay += 1;     /////////////////// continue thinking
	ans_vec[1] += 1;
}

int search_aim_cross_id(Road road_1, Road road_2)
{
	int road_1_to = road_1.to_cross->cross_id;
	int road_1_from = road_1.from_cross->cross_id;
	int road_2_to = road_2.to_cross->cross_id;
	int road_2_from = road_2.from_cross->cross_id;
	if (road_1_to == road_2_to)
	{
		return road_1_to;
	}
	else if (road_1_to == road_2_from)
	{
		return road_1_to;
	}
	else if (road_1_from == road_2_to)
	{
		return road_1_from;
	}	
	else if (road_1_from == road_2_from)
	{
		return road_1_from;
	}
}

void reset_all(Road *road_set, int road_size, int system_time)
{
	for (int r = 0; r <= road_size - 1; r++)
	{
		for (int i = 0; i <= road_set[r].channel - 1; i++)
		{
			for (int j = 0; j <= road_set[r].length - 1; j++)
			{
				if (road_set[r].to_cross_mat[i][j] != NULL)
				{
					road_set[r].to_cross_mat[i][j]->wait = 0;
					road_set[r].to_cross_mat[i][j]->end = 0;
					road_set[r].to_cross_mat[i][j]->system_time = system_time;
				}
				if (road_set[r].from_cross_mat[i][j] != NULL)
				{
					road_set[r].from_cross_mat[i][j]->wait = 0;
					road_set[r].from_cross_mat[i][j]->end = 0;
					road_set[r].from_cross_mat[i][j]->system_time = system_time;
				}
			}
		}
		
	}
}

bool still_wait_car(Car *car_set, int car_size)
{
	for (int k = 0; k <= car_size - 1; k++)
	{
		if (car_set[k].wait == 1)
		{	
			return true;
		}
	}
	return false;
}

Car *find_road_prior_car(Road *road_p, int tmp_cross_id)
{
	for (int j = road_p->length - 1; j >= 0; j--)
	{
		for (int i = 0; i <= road_p->channel - 1; i++) 
		{
			if ((road_p->to_cross_mat[i][j] != NULL) && (road_p->to_cross_mat[i][j]->wait == 1) && (road_p->to_cross_mat[i][j]->aim_cross_id == tmp_cross_id))
			{
				road_p->to_cross_mat[i][j]->now_x = i;
				road_p->to_cross_mat[i][j]->now_y = j;
				road_p->to_cross_mat[i][j]->on_to = true;
				road_p->to_cross_mat[i][j]->on_from = false;
				return road_p->to_cross_mat[i][j];
			}
			else if ((road_p->from_cross_mat[i][j] != NULL) && (road_p->from_cross_mat[i][j]->wait == 1) && (road_p->from_cross_mat[i][j]->aim_cross_id == tmp_cross_id))
			{
				road_p->from_cross_mat[i][j]->now_x = i;
				road_p->from_cross_mat[i][j]->now_y = j;
				road_p->from_cross_mat[i][j]->on_to = false;
				road_p->from_cross_mat[i][j]->on_from = true;
				return road_p->from_cross_mat[i][j];
			}
		}
	}
	return NULL;
}

bool cross_exist_car(Cross cross)
{
	if (cross.road_1_p != NULL)
	{
		for (int j = cross.road_1_p->length - 1; j >= 0; j--)
		{
			for (int i = 0; i <= cross.road_1_p->channel - 1; i++)
			{
				if (cross.road_1_p->to_cross_mat[i][j] != NULL)
				{
					return true;
				}
				if (cross.road_1_p->from_cross_mat[i][j] != NULL)
				{
					return true;
				}
			}
		}
	}
	if (cross.road_2_p != NULL)
	{
		for (int j = cross.road_2_p->length - 1; j >= 0; j--)
		{
			for (int i = 0; i <= cross.road_2_p->channel - 1; i++)
			{
				if (cross.road_2_p->to_cross_mat[i][j] != NULL)
				{
					return true;
				}
				if (cross.road_2_p->from_cross_mat[i][j] != NULL)
				{
					return true;
				}
			}
		}
	}
	if (cross.road_3_p != NULL)
	{
		for (int j = cross.road_3_p->length - 1; j >= 0; j--)
		{
			for (int i = 0; i <= cross.road_3_p->channel - 1; i++)
			{
				if (cross.road_3_p->to_cross_mat[i][j] != NULL)
				{
					return true;
				}
				if (cross.road_3_p->from_cross_mat[i][j] != NULL)
				{
					return true;
				}
			}
		}
	}
	if (cross.road_4_p != NULL)
	{
		for (int j = cross.road_4_p->length - 1; j >= 0; j--)
		{
			for (int i = 0; i <= cross.road_4_p->channel - 1; i++)
			{
				if (cross.road_4_p->to_cross_mat[i][j] != NULL)
				{
					return true;
				}
				if (cross.road_4_p->from_cross_mat[i][j] != NULL)
				{
					return true;
				}
			}
		}
	}
	return false;
}

void move_wait_car(Cross *cross_set, int cross_size, Road *road_set, map<int, int> road_set_map, int road_size) // note the prior
{
	for (int c = 0; c <= cross_size - 1; c++)
	{
		if (cross_exist_car(cross_set[c]))
		{
			int tmp_cross_id = cross_set[c].cross_id; // now aim cross id
			vector<int> id_vec;
			vector<int> l_id_vec;
			vector<int> r_id_vec;
			if (cross_set[c].road_1_p != NULL)
			{
				cross_set[c].road_1_p->prior_car = find_road_prior_car(cross_set[c].road_1_p, tmp_cross_id);
				if (cross_set[c].road_1_p->prior_car != NULL)
				{
					id_vec.push_back(cross_set[c].road_1_p->road_id);
				}
			}
			if (cross_set[c].road_2_p != NULL)
			{
				cross_set[c].road_2_p->prior_car = find_road_prior_car(cross_set[c].road_2_p, tmp_cross_id);
				if (cross_set[c].road_2_p->prior_car != NULL)
				{
					id_vec.push_back(cross_set[c].road_2_p->road_id);
				}
			}
			if (cross_set[c].road_3_p != NULL)
			{
				cross_set[c].road_3_p->prior_car = find_road_prior_car(cross_set[c].road_3_p, tmp_cross_id);
				if (cross_set[c].road_3_p->prior_car != NULL)
				{
					id_vec.push_back(cross_set[c].road_3_p->road_id);
				}
			}
			if (cross_set[c].road_4_p != NULL)
			{
				cross_set[c].road_4_p->prior_car = find_road_prior_car(cross_set[c].road_4_p, tmp_cross_id);
				if (cross_set[c].road_4_p->prior_car != NULL)
				{
					id_vec.push_back(cross_set[c].road_4_p->road_id);
				}
			}
			int id_tmp;
			while (!id_vec.empty())
			{
				id_tmp = find_min(id_vec); // do not erase here

				if ((cross_set[c].road_1_p != NULL) && (id_tmp == cross_set[c].road_1_p->road_id) && (cross_set[c].road_1_p->prior_car != NULL))
				{
					move_wait_car_inner(id_vec, l_id_vec, r_id_vec, cross_set[c].road_1_p, cross_set[c].road_1_p->prior_car, road_set, road_set_map);
				}
				else if ((cross_set[c].road_2_p != NULL) && (id_tmp == cross_set[c].road_2_p->road_id) && (cross_set[c].road_2_p->prior_car != NULL))
				{
					move_wait_car_inner(id_vec, l_id_vec, r_id_vec, cross_set[c].road_2_p, cross_set[c].road_2_p->prior_car, road_set, road_set_map);
				}
				else if ((cross_set[c].road_3_p != NULL) && (id_tmp == cross_set[c].road_3_p->road_id) && (cross_set[c].road_3_p->prior_car != NULL))
				{
					move_wait_car_inner(id_vec, l_id_vec, r_id_vec, cross_set[c].road_3_p, cross_set[c].road_3_p->prior_car, road_set, road_set_map);
				}
				else if ((cross_set[c].road_4_p != NULL) && (id_tmp == cross_set[c].road_4_p->road_id) && (cross_set[c].road_4_p->prior_car != NULL))
				{
					move_wait_car_inner(id_vec, l_id_vec, r_id_vec, cross_set[c].road_4_p, cross_set[c].road_4_p->prior_car, road_set, road_set_map);
				}
			}
			while (!l_id_vec.empty())
			{

				id_tmp = find_min(l_id_vec); // do not erase here

				if ((cross_set[c].road_1_p != NULL) && (id_tmp == cross_set[c].road_1_p->road_id) && (cross_set[c].road_1_p->prior_car != NULL))
				{
					move_wait_car_inner(l_id_vec, l_id_vec, r_id_vec, cross_set[c].road_1_p, cross_set[c].road_1_p->prior_car, road_set, road_set_map);
				}
				else if ((cross_set[c].road_2_p != NULL) && (id_tmp == cross_set[c].road_2_p->road_id) && (cross_set[c].road_2_p->prior_car != NULL))
				{
					move_wait_car_inner(l_id_vec, l_id_vec, r_id_vec, cross_set[c].road_2_p, cross_set[c].road_2_p->prior_car, road_set, road_set_map);
				}
				else if ((cross_set[c].road_3_p != NULL) && (id_tmp == cross_set[c].road_3_p->road_id) && (cross_set[c].road_3_p->prior_car != NULL))
				{
					move_wait_car_inner(l_id_vec, l_id_vec, r_id_vec, cross_set[c].road_3_p, cross_set[c].road_3_p->prior_car, road_set, road_set_map);
				}
				else if ((cross_set[c].road_4_p != NULL) && (id_tmp == cross_set[c].road_4_p->road_id) && (cross_set[c].road_4_p->prior_car != NULL))
				{
					move_wait_car_inner(l_id_vec, l_id_vec, r_id_vec, cross_set[c].road_4_p, cross_set[c].road_4_p->prior_car, road_set, road_set_map);
				}
			}
			while (!r_id_vec.empty())
			{

				id_tmp = find_min(r_id_vec); // do not erase here

				if ((cross_set[c].road_1_p != NULL) && (id_tmp == cross_set[c].road_1_p->road_id) && (cross_set[c].road_1_p->prior_car != NULL))
				{
					move_wait_car_inner(r_id_vec, l_id_vec, r_id_vec, cross_set[c].road_1_p, cross_set[c].road_1_p->prior_car, road_set, road_set_map);
				}
				else if ((cross_set[c].road_2_p != NULL) && (id_tmp == cross_set[c].road_2_p->road_id) && (cross_set[c].road_2_p->prior_car != NULL))
				{
					move_wait_car_inner(r_id_vec, l_id_vec, r_id_vec, cross_set[c].road_2_p, cross_set[c].road_2_p->prior_car, road_set, road_set_map);
				}
				else if ((cross_set[c].road_3_p != NULL) && (id_tmp == cross_set[c].road_3_p->road_id) && (cross_set[c].road_3_p->prior_car != NULL))
				{
					move_wait_car_inner(r_id_vec, l_id_vec, r_id_vec, cross_set[c].road_3_p, cross_set[c].road_3_p->prior_car, road_set, road_set_map);
				}
				else if ((cross_set[c].road_4_p != NULL) && (id_tmp == cross_set[c].road_4_p->road_id) && (cross_set[c].road_4_p->prior_car != NULL))
				{
					move_wait_car_inner(r_id_vec, l_id_vec, r_id_vec, cross_set[c].road_4_p, cross_set[c].road_4_p->prior_car, road_set, road_set_map);
				}
			}
		}
	}
}

void move_wait_car_inner(vector<int> &id_vec, vector<int> &l_id_vec, vector<int> &r_id_vec, Road *road_p, Car *car_p_now, Road *road_set, map<int, int> road_set_map)
{
	int s1 = road_p->length - 1 - car_p_now->now_y;
	if (!can_not_go_out_cross(id_vec, s1, road_p, car_p_now)) // wait car can not go out cross / can go out cross
	{
		if (car_p_now->aim_cross_id == road_p->to_cross->cross_id)
		{
			can_go_out_cross(id_vec, l_id_vec, r_id_vec, road_p, road_p->to_cross, road_set, road_set_map);
		}
		else if (car_p_now->aim_cross_id == road_p->from_cross->cross_id)
		{
			can_go_out_cross(id_vec, l_id_vec, r_id_vec, road_p, road_p->from_cross, road_set, road_set_map);
		}
	}
}

bool can_not_go_out_cross(vector<int> &id_vec, int s1, Road *road_p, Car *car_p)
{
	int sp = min(road_p->speed, car_p->speed);
	if (car_p->on_to == true)
	{
		if (no_front_car(road_p->to_cross_mat, car_p->now_x, car_p->now_y, road_p->length))
		{
			if (sp <= s1)
			{
				car_p->wait = 0;
				car_p->end = 1;
				road_p->to_cross_mat[car_p->now_x][car_p->now_y + sp] = road_p->to_cross_mat[car_p->now_x][car_p->now_y];
				road_p->to_cross_mat[car_p->now_x][car_p->now_y] = NULL;
				car_p->now_y = car_p->now_y + sp;
				search_back_car_turn_end(road_p, car_p->now_x, car_p->on_to);
				return true;
			}
			else
			{
				return false;
			}
		}
		else // ahead of this channel has car
		{
			for (int j = car_p->now_y + 1; j <= min(car_p->now_y + sp, road_p->length - 1); j++) ///////////////////////////////////////////////////
			{
				if (road_p->to_cross_mat[car_p->now_x][j] == NULL) // because of front car, so do not worry about out of range
				{
					continue;
				}
				// sp range exists end car
				else if ((road_p->to_cross_mat[car_p->now_x][j] != NULL) && (road_p->to_cross_mat[car_p->now_x][j]->end == 1))
				{
					car_p->wait = 0;
					car_p->end = 1;
					if (j - 1 > car_p->now_y)
					{
						road_p->to_cross_mat[car_p->now_x][j - 1] = road_p->to_cross_mat[car_p->now_x][car_p->now_y];
						road_p->to_cross_mat[car_p->now_x][car_p->now_y] = NULL;
						car_p->now_y = j - 1;
						car_p->on_from = false;
						car_p->on_to = true;
						find_and_erase(id_vec, road_p->road_id);
						search_back_car_turn_end(road_p, car_p->now_x, car_p->on_to);
					}
					else if (j - 1 == car_p->now_y)
					{
						car_p->on_from = false;
						car_p->on_to = true;
						find_and_erase(id_vec, road_p->road_id);
						search_back_car_turn_end(road_p, car_p->now_x, car_p->on_to);
					}
					return true;
				}
				// sp range exists car
				// still wait
				else if ((road_p->to_cross_mat[car_p->now_x][j] != NULL) && (road_p->to_cross_mat[car_p->now_x][j]->wait == 1))
				{
					car_p->wait = 1;
					car_p->end = 0;
					find_and_erase(id_vec, road_p->road_id);
					return true;
				}
			}
			// sp range no car -> turn end
			if (car_p->end == 0)
			{
				car_p->wait = 0;
				car_p->end = 1;
				road_p->to_cross_mat[car_p->now_x][car_p->now_y + sp] = road_p->to_cross_mat[car_p->now_x][car_p->now_y];
				road_p->to_cross_mat[car_p->now_x][car_p->now_y] = NULL;
				car_p->now_y = car_p->now_y + sp;
				find_and_erase(id_vec, road_p->road_id);
				search_back_car_turn_end(road_p, car_p->now_x, car_p->on_to);
				return true;
			}

		}
	}
	else if (car_p->on_from == true)
	{
		if (no_front_car(road_p->from_cross_mat, car_p->now_x, car_p->now_y, road_p->length))
		{
			if (sp <= s1)
			{
				car_p->wait = 0;
				car_p->end = 1;
				road_p->from_cross_mat[car_p->now_x][car_p->now_y + sp] = road_p->from_cross_mat[car_p->now_x][car_p->now_y];
				road_p->from_cross_mat[car_p->now_x][car_p->now_y] = NULL;
				car_p->now_y = car_p->now_y + sp;
				search_back_car_turn_end(road_p, car_p->now_x, car_p->on_to);
				return true;
			}
			else
			{
				return false;
			}
		}
		else // ahead of this channel has car
		{
			for (int j = car_p->now_y + 1; j <= min(car_p->now_y + sp, road_p->length - 1); j++)
			{
				if (road_p->from_cross_mat[car_p->now_x][j] == NULL) // because of front car, so do not worry about out of range
				{
					continue;
				}
				// sp range exists car
				else if ((road_p->from_cross_mat[car_p->now_x][j] != NULL) && (road_p->from_cross_mat[car_p->now_x][j]->end == 1))
				{
					car_p->wait = 0;
					car_p->end = 1;
					if (j - 1 > car_p->now_y)
					{
						road_p->from_cross_mat[car_p->now_x][j - 1] = road_p->from_cross_mat[car_p->now_x][car_p->now_y];
						road_p->from_cross_mat[car_p->now_x][car_p->now_y] = NULL;
						car_p->now_y = j - 1;
						car_p->on_from = true;
						car_p->on_to = false;
						find_and_erase(id_vec, road_p->road_id);
						search_back_car_turn_end(road_p, car_p->now_x, car_p->on_to);
					}
					else if (j - 1 == car_p->now_y)
					{
						car_p->on_from = true;
						car_p->on_to = false;
						find_and_erase(id_vec, road_p->road_id);
						search_back_car_turn_end(road_p, car_p->now_x, car_p->on_to);
					}
					return true;
				}
				// sp range exists car
				// still wait
				else if ((road_p->from_cross_mat[car_p->now_x][j] != NULL) && (road_p->from_cross_mat[car_p->now_x][j]->wait == 1))
				{
					car_p->wait = 1;
					car_p->end = 0;
					find_and_erase(id_vec, road_p->road_id);
					return true;
				}
			}
			// sp range no car -> turn end
			if (car_p->end == 0)
			{
				car_p->wait = 0;
				car_p->end = 1;
				road_p->from_cross_mat[car_p->now_x][car_p->now_y + sp] = road_p->from_cross_mat[car_p->now_x][car_p->now_y];
				road_p->from_cross_mat[car_p->now_x][car_p->now_y] = NULL;
				car_p->now_y = car_p->now_y + sp;
				find_and_erase(id_vec, road_p->road_id);
				search_back_car_turn_end(road_p, car_p->now_x, car_p->on_to);
				return true;
			}
		}
	}

}

void can_go_out_cross(vector<int> &id_vec, vector<int> &l_id_vec, vector<int> &r_id_vec, Road *road_p, Cross* aim_cross, Road *road_set, map<int, int> road_set_map) //aim_cross : to cross or from cross
{
	if(road_p->road_id == aim_cross->road_1_id)
	{
		run_car_through_cross(id_vec, l_id_vec, r_id_vec, road_p, road_p->prior_car, aim_cross->road_3_p, aim_cross->road_2_p, aim_cross->road_4_p, road_set, road_set_map);
	}
	else if(road_p->road_id == aim_cross->road_2_id)
	{
		run_car_through_cross(id_vec, l_id_vec, r_id_vec, road_p, road_p->prior_car, aim_cross->road_4_p, aim_cross->road_3_p, aim_cross->road_1_p, road_set, road_set_map);
	}
	else if(road_p->road_id == aim_cross->road_3_id)
	{
		run_car_through_cross(id_vec, l_id_vec, r_id_vec, road_p, road_p->prior_car, aim_cross->road_1_p, aim_cross->road_4_p, aim_cross->road_2_p, road_set, road_set_map);
	}
	else if(road_p->road_id == aim_cross->road_4_id)
	{
		run_car_through_cross(id_vec, l_id_vec, r_id_vec, road_p, road_p->prior_car, aim_cross->road_2_p, aim_cross->road_1_p, aim_cross->road_3_p, road_set, road_set_map);
	}
}

void find_and_erase(vector<int> &id_vec, int id)
{
	std::vector<int>::iterator ite = find(id_vec.begin(), id_vec.end(), id);
	int position = distance(begin(id_vec), ite);
	id_vec.erase(id_vec.begin() + position);
}

void run_car_through_cross(vector<int> &id_vec, vector<int> &l_id_vec, vector<int> &r_id_vec, Road *road_p, Car *now_car_p, Road *d_road_p, Road *l_road_p, Road *r_road_p, Road *road_set, map<int, int> road_set_map)
{
	// here make wait car back wait car dir
	if (now_car_p->dir == 0)
	{
		make_wait_dir_flag(road_set[road_set_map[road_p->road_id]], now_car_p);
	}

	int s1 = road_p->length - 1 - now_car_p->now_y;
	if (now_car_p->dir == 3) // d
	{
		int sp2 = min(d_road_p->speed, now_car_p->speed);
		int s2 = sp2 - s1;
		if (s2 <= 0) // can not go out cross
		{

			s2 = 0;
			if (now_car_p->on_to == true)
			{
				road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y + s1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
				road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL; // s1 is impossible to be 0, if 0 s2 must > 0
			}
			else if (now_car_p->on_from == true)
			{
				road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y + s1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
				road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
			}
			now_car_p->wait = 0;
			now_car_p->end = 1;
			now_car_p->now_y += s1;

			find_and_erase(id_vec, road_p->road_id);
			road_p->prior_car = NULL;
			search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to);
			return;
			
		}
		else // can go out cross directly // if next road is full , stay wait
		{	
			if (now_car_p->plan_route.size() >= 2)
			{
				int tmp_next_aim_cross_id;
				if (now_car_p->plan_route.size() >= 3)
				{
					tmp_next_aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[1]]], road_set[road_set_map[now_car_p->plan_route[2]]]);
				}
				else if (now_car_p->plan_route.size() == 2)
				{
					tmp_next_aim_cross_id = now_car_p->to;
				}
				
				
				if (tmp_next_aim_cross_id == d_road_p->to_cross->cross_id)
				{
					for (int i = 0; i <= d_road_p->channel - 1; i++)
					{
						for (int j = 0; j <= s2 - 1; j++)
						{
							if (d_road_p->to_cross_mat[i][0] != NULL)
							{
								break;
							}
							else if (d_road_p->to_cross_mat[i][j] == NULL)
							{
								continue;
							}
							else
							{
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 0;
								now_car_p->end = 1;
								now_car_p->dir = 0;

								if (now_car_p->on_to == true)
								{
									d_road_p->to_cross_mat[i][j - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}
								else if (now_car_p->on_from == true)
								{
									d_road_p->to_cross_mat[i][j - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}

								
								now_car_p->now_x = i;
								now_car_p->now_y = j - 1;
								now_car_p->now_road = d_road_p;
								//now_car_p->to_cross = d_road_p->to_cross;
								now_car_p->plan_route.erase(now_car_p->plan_route.begin());
								if (now_car_p->plan_route.size() >= 2)
								{
									now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
								}
								else if (now_car_p->plan_route.size() == 1)
								{
									now_car_p->aim_cross_id = now_car_p->to;
								}
								road_p->prior_car = NULL;
								
								return;
							}
						}
					}
					if ((d_road_p->to_cross_mat[0][s2 - 1] == NULL) && (not_break_out(d_road_p->to_cross_mat, d_road_p->channel))) //after all continue -> all null
					{
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0;

						if (now_car_p->on_to == true)
						{
							d_road_p->to_cross_mat[0][s2 - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = true;
							now_car_p->on_from = false;
						}
						else if (now_car_p->on_from == true)
						{
							d_road_p->to_cross_mat[0][s2 - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = true;
							now_car_p->on_from = false;
						}


						now_car_p->now_x = 0;
						now_car_p->now_y = s2 - 1;
						now_car_p->now_road = d_road_p;
						//now_car_p->to_cross = d_road_p->to_cross;
						now_car_p->plan_route.erase(now_car_p->plan_route.begin()); // update plan route
						if (now_car_p->plan_route.size() >= 2)
						{
							now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
						}
						else if (now_car_p->plan_route.size() == 1)
						{
							now_car_p->aim_cross_id = now_car_p->to;
						}
						road_p->prior_car = NULL; 
						return;
					}
					else if (!not_break_out(d_road_p->to_cross_mat, d_road_p->channel)) // because of break out // every road'tail has car
					{
						// if tail are all end car -> turn end
						for (int i = 0; i <= d_road_p->channel - 1; i++)
						{

							
							if ((d_road_p->to_cross_mat[i][0] != NULL) && (d_road_p->to_cross_mat[i][0]->end == 1))
							{
								continue;
							}
							else if ((d_road_p->to_cross_mat[i][0] != NULL) && (d_road_p->to_cross_mat[i][0]->wait == 1))
							{
								road_p->prior_car = NULL;
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 1;
								now_car_p->end = 0;
								now_car_p->dir = 0;
								return;// else if tail has wait car -> hold on wait  //so not move still wait and erase the road
							}
							
						}
						road_p->prior_car = NULL;
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0; // not move and turn to end
						search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to);
						return;
					}
				}
				else if (tmp_next_aim_cross_id == d_road_p->from_cross->cross_id)
				{
					for (int i = 0; i <= d_road_p->channel - 1; i++)
					{
						for (int j = 0; j <= s2 - 1; j++)
						{
							if (d_road_p->from_cross_mat[i][0] != NULL)
							{
								break;
							}
							else if (d_road_p->from_cross_mat[i][j] == NULL)
							{
								continue;
							}
							else
							{
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 0;
								now_car_p->end = 1;
								now_car_p->dir = 0;

								if (now_car_p->on_to == true)
								{
									d_road_p->from_cross_mat[i][j - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}
								else if (now_car_p->on_from == true)
								{
									d_road_p->from_cross_mat[i][j - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}

										
								now_car_p->now_x = i;
								now_car_p->now_y = j - 1;
								now_car_p->now_road = d_road_p;
								//now_car_p->to_cross = d_road_p->to_cross;
								now_car_p->plan_route.erase(now_car_p->plan_route.begin());
								if (now_car_p->plan_route.size() >= 2)
								{
									now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
								}
								else if (now_car_p->plan_route.size() == 1)
								{
									now_car_p->aim_cross_id = now_car_p->to;
								}
								road_p->prior_car = NULL;
								return;
							}
						}
					}
					if ((d_road_p->from_cross_mat[0][s2 - 1] == NULL) && (not_break_out(d_road_p->from_cross_mat, d_road_p->channel))) //after all continue -> all null
					{
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0;

						if (now_car_p->on_to == true)
						{
							d_road_p->from_cross_mat[0][s2 - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = false;
							now_car_p->on_from = true;
						}
						else if (now_car_p->on_from == true)
						{
							d_road_p->from_cross_mat[0][s2 - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = false;
							now_car_p->on_from = true;
						}


						now_car_p->now_x = 0;
						now_car_p->now_y = s2 - 1;
						now_car_p->now_road = d_road_p;
						//now_car_p->to_cross = d_road_p->to_cross;
						now_car_p->plan_route.erase(now_car_p->plan_route.begin()); // update plan route
						if (now_car_p->plan_route.size() >= 2)
						{
							now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
						}
						else if (now_car_p->plan_route.size() == 1)
						{
							now_car_p->aim_cross_id = now_car_p->to;
						}
						road_p->prior_car = NULL;
						return;
					}
					else if (!not_break_out(d_road_p->from_cross_mat, d_road_p->channel)) // because of break out // every road'tail has car
					{
						// if tail are all end car -> turn end
						for (int i = 0; i <= d_road_p->channel - 1; i++)
						{

							if ((d_road_p->from_cross_mat[i][0] != NULL) && (d_road_p->from_cross_mat[i][0]->end == 1))
							{
								continue;
							}
							else if ((d_road_p->from_cross_mat[i][0] != NULL) && (d_road_p->from_cross_mat[i][0]->wait == 1))
							{
								road_p->prior_car = NULL;
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 1;
								now_car_p->end = 0;
								now_car_p->dir = 0;
								return;// else if tail has wait car -> hold on wait  //so not move still wait and erase the road
							}
							
						}
						road_p->prior_car = NULL;
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0; // not move and turn to end
						search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to);
						return;
					}
				}
			}
			else //route size == 1, go to dest, set null
			{
				find_and_erase(id_vec, road_p->road_id);
				now_car_p->to_cross = NULL;
				now_car_p->car_pos = false;
				now_car_p->arrived = true;
				now_car_p->wait = 0;
				now_car_p->end = 0;
				int old_channel_index = now_car_p->now_x;
				if (now_car_p->on_to == true)
				{
					road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
				}
				else if (now_car_p->on_from == true)
				{
					road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
				}
				
				cout << now_car_p->car_id << " arrived and arrived time is " << now_car_p->system_time << endl;
				count_car++;
				cout << "now arrived " << count_car << "cars" << endl;
				road_p->prior_car = NULL;
				search_back_car_turn_end(road_p, old_channel_index, now_car_p->on_to);
			}

		}
	}
	else if (now_car_p->dir == 2)
	{
		int sp2 = min(l_road_p->speed, now_car_p->speed);
		int s2 = sp2 - s1;
		if (s2 <= 0) // can not go out cross
		{	
			find_and_erase(id_vec, road_p->road_id);
			s2 = 0;
			if (now_car_p->on_to == true)
			{
				road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y + s1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
				road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL; // s1 must > 0
			}
			else if (now_car_p->on_from == true)
			{
				road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y + s1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
				road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
			}
			now_car_p->wait = 0;
			now_car_p->end = 1;
			now_car_p->now_y += s1;
			road_p->prior_car = NULL;
			search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to);
			return;
		}
		else if ((r_road_p != NULL) && (r_road_p->prior_car != NULL) && (r_road_p->prior_car->dir == 3))  // hold on // go left need to check "right road" 's prior car
		{
			find_and_erase(id_vec, road_p->road_id);
			l_id_vec.push_back(road_p->road_id); // car want to go left
			return;
		}
		else // can go out cross directly // if next road is full , stay wait
		{
			if (now_car_p->plan_route.size() >= 2)
			{
				int tmp_next_aim_cross_id;
				if (now_car_p->plan_route.size() >= 3)
				{
					tmp_next_aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[1]]], road_set[road_set_map[now_car_p->plan_route[2]]]);
				}
				else if (now_car_p->plan_route.size() == 2)
				{
					tmp_next_aim_cross_id = now_car_p->to;
				}

				
				if (tmp_next_aim_cross_id == l_road_p->to_cross->cross_id)
				{
					for (int i = 0; i <= l_road_p->channel - 1; i++)
					{
						for (int j = 0; j <= s2 - 1; j++)
						{
							if (l_road_p->to_cross_mat[i][0] != NULL)
							{
								break;
							}
							else if (l_road_p->to_cross_mat[i][j] == NULL)
							{
								continue;
							}
							else
							{
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 0;
								now_car_p->end = 1;
								now_car_p->dir = 0;

								if (now_car_p->on_to == true)
								{
									l_road_p->to_cross_mat[i][j - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}
								else if (now_car_p->on_from == true)
								{
									l_road_p->to_cross_mat[i][j - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}


								now_car_p->now_x = i;
								now_car_p->now_y = j - 1;
								now_car_p->now_road = l_road_p;
								//now_car_p->to_cross = l_road_p->to_cross;
								now_car_p->plan_route.erase(now_car_p->plan_route.begin());
								if (now_car_p->plan_route.size() >= 2)
								{
									now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
								}
								road_p->prior_car = NULL;
								return;
							}
						}
					}
					if ((l_road_p->to_cross_mat[0][s2 - 1] == NULL) && (not_break_out(l_road_p->to_cross_mat, l_road_p->channel))) //after all continue -> all null
					{
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0;

						if (now_car_p->on_to == true)
						{
							l_road_p->to_cross_mat[0][s2 - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = true;
							now_car_p->on_from = false;
						}
						else if (now_car_p->on_from == true)
						{
							l_road_p->to_cross_mat[0][s2 - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = true;
							now_car_p->on_from = false;
						}


						now_car_p->now_x = 0;
						now_car_p->now_y = s2 - 1;
						now_car_p->now_road = l_road_p;
						//now_car_p->to_cross = l_road_p->to_cross;
						now_car_p->plan_route.erase(now_car_p->plan_route.begin()); // update plan route
						if (now_car_p->plan_route.size() >= 2)
						{
							now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
						}
						road_p->prior_car = NULL;
						return;
					}
					else if (!not_break_out(l_road_p->to_cross_mat, l_road_p->channel)) // because of break out // every road'tail has car
					{
						// if tail are all end car -> turn end
						for (int i = 0; i <= l_road_p->channel - 1; i++)
						{

							if ((l_road_p->to_cross_mat[i][0] != NULL) && (l_road_p->to_cross_mat[i][0]->end == 1))
							{
								continue;
							}
							else if ((l_road_p->to_cross_mat[i][0] != NULL) && (l_road_p->to_cross_mat[i][0]->wait == 1))
							{
								road_p->prior_car = NULL;
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 1;
								now_car_p->end = 0;
								now_car_p->dir = 0;
								return;// else if tail has wait car -> hold on wait  //so not move still wait and erase the road
							}
							
						}
						road_p->prior_car = NULL;
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0; // not move and turn to end
						search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to);
						return;
					}
				}
				else if (tmp_next_aim_cross_id == l_road_p->from_cross->cross_id)
				{
					for (int i = 0; i <= l_road_p->channel - 1; i++)
					{
						for (int j = 0; j <= s2 - 1 ; j++)
						{
							if (l_road_p->from_cross_mat[i][0] != NULL)
							{
								break;
							}
							else if (l_road_p->from_cross_mat[i][j] == NULL)
							{
								continue;
							}
							else
							{
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 0;
								now_car_p->end = 1;
								now_car_p->dir = 0;

								if (now_car_p->on_to == true)
								{
									l_road_p->from_cross_mat[i][j - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}
								else if (now_car_p->on_from == true)
								{
									l_road_p->from_cross_mat[i][j - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}


								now_car_p->now_x = i;
								now_car_p->now_y = j - 1;
								now_car_p->now_road = l_road_p;
								//now_car_p->to_cross = l_road_p->to_cross;
								now_car_p->plan_route.erase(now_car_p->plan_route.begin());
								if (now_car_p->plan_route.size() >= 2)
								{
									now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
								}
								road_p->prior_car = NULL;
								return;
							}
						}
					}
					if ((l_road_p->from_cross_mat[0][s2 - 1] == NULL) && (not_break_out(l_road_p->from_cross_mat, l_road_p->channel))) //after all continue -> all null
					{
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0;

						if (now_car_p->on_to == true)
						{
							l_road_p->from_cross_mat[0][s2 - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = false;
							now_car_p->on_from = true;
						}
						else if (now_car_p->on_from == true)
						{
							l_road_p->from_cross_mat[0][s2 - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = false;
							now_car_p->on_from = true;
						}


						now_car_p->now_x = 0;
						now_car_p->now_y = s2 - 1;
						now_car_p->now_road = l_road_p;
						//now_car_p->to_cross = l_road_p->to_cross;
						now_car_p->plan_route.erase(now_car_p->plan_route.begin()); // update plan route
						if (now_car_p->plan_route.size() >= 2)
						{
							now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
						}
						road_p->prior_car = NULL;
						return;
					}
					else if (!not_break_out(l_road_p->from_cross_mat, l_road_p->channel)) // because of break out // every road'tail has car
					{
						// if tail are all end car -> turn end
						for (int i = 0; i <= l_road_p->channel - 1; i++)
						{

							if ((l_road_p->from_cross_mat[i][0] != NULL) && (l_road_p->from_cross_mat[i][0]->end == 1))
							{
								continue;
							}
							else if ((l_road_p->from_cross_mat[i][0] != NULL) && (l_road_p->from_cross_mat[i][0]->wait == 1))
							{
								road_p->prior_car = NULL;
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 1;
								now_car_p->end = 0;
								now_car_p->dir = 0;
								return;// else if tail has wait car -> hold on wait  //so not move still wait and erase the road
							}
							
						}
						road_p->prior_car = NULL;
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0; // not move and turn to end
						search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to);
						return;
					}
				}
			}
			else //route size == 1, go to dest, set null
			{
				find_and_erase(id_vec, road_p->road_id);
				now_car_p->to_cross = NULL;
				now_car_p->car_pos = false;
				now_car_p->arrived = true;
				now_car_p->wait = 0;
				now_car_p->end = 0;
				int old_channel_index = now_car_p->now_x;
				if (now_car_p->on_to == true)
				{
					road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
				}
				else if (now_car_p->on_from == true)
				{
					road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
				}

				cout << now_car_p->car_id << " arrived and arrived time is " << now_car_p->system_time << endl;
				count_car++;
				cout << "now arrived " << count_car << "cars" << endl;
				road_p->prior_car = NULL;
				search_back_car_turn_end(road_p, old_channel_index, now_car_p->on_to);
			}
		}
	}
	else if (now_car_p->dir == 1)
	{
		int sp2 = min(r_road_p->speed, now_car_p->speed);
		int s2 = sp2 - s1;
		if (s2 <= 0) // can not go out cross
		{	
			find_and_erase(id_vec, road_p->road_id);
			s2 = 0;
			road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y + s1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
			road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL; // s1 must > 0
			now_car_p->wait = 0;
			now_car_p->end = 1;
			now_car_p->now_y += s1;
			road_p->prior_car = NULL;
			search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to);
			return;
		}
		else if (((l_road_p != NULL) && (l_road_p->prior_car != NULL) && (l_road_p->prior_car->dir == 3)) || (d_road_p != NULL)&&((d_road_p->prior_car != NULL) && (d_road_p->prior_car->dir == 2)))  // hold on // go right need to check "direct and left road" 's prior car
		{
			find_and_erase(id_vec, road_p->road_id);
			r_id_vec.push_back(road_p->road_id); // car want to go right
			return;
		}
		else // can go out cross directly // if next road is full , stay wait
		{
			if (now_car_p->plan_route.size() >= 2)
			{
				int tmp_next_aim_cross_id;
				if (now_car_p->plan_route.size() >= 3)
				{
					tmp_next_aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[1]]], road_set[road_set_map[now_car_p->plan_route[2]]]);
				}
				else if (now_car_p->plan_route.size() == 2)
				{
					tmp_next_aim_cross_id = now_car_p->to;
				}

				if (tmp_next_aim_cross_id == r_road_p->to_cross->cross_id)
				{
					for (int i = 0; i <= r_road_p->channel - 1; i++)
					{
						for (int j = 0; j <= s2 - 1; j++)
						{
							if (r_road_p->to_cross_mat[i][0] != NULL)
							{
								break;
							}
							else if (r_road_p->to_cross_mat[i][j] == NULL)
							{
								continue;
							}
							else
							{
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 0;
								now_car_p->end = 1;
								now_car_p->dir = 0;

								if (now_car_p->on_to == true)
								{
									r_road_p->to_cross_mat[i][j - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}
								else if (now_car_p->on_from == true)
								{
									r_road_p->to_cross_mat[i][j - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}


								now_car_p->now_x = i;
								now_car_p->now_y = j - 1;
								now_car_p->now_road = r_road_p;
								//now_car_p->to_cross = r_road_p->to_cross;
								now_car_p->plan_route.erase(now_car_p->plan_route.begin());
								if (now_car_p->plan_route.size() >= 2)
								{
									now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
								}
  								road_p->prior_car = NULL;
								return;
							}
						}
					}
					if ((r_road_p->to_cross_mat[0][s2 - 1] == NULL) && (not_break_out(r_road_p->to_cross_mat, r_road_p->channel))) //after all continue -> all null
					{
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0;

						if (now_car_p->on_to == true)
						{
							r_road_p->to_cross_mat[0][s2 - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = true;
							now_car_p->on_from = false;
						}
						else if (now_car_p->on_from == true)
						{
							r_road_p->to_cross_mat[0][s2 - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = true;
							now_car_p->on_from = false;
						}


						now_car_p->now_x = 0;
						now_car_p->now_y = s2 - 1;
						now_car_p->now_road = r_road_p;
						//now_car_p->to_cross = r_road_p->to_cross;
						now_car_p->plan_route.erase(now_car_p->plan_route.begin()); // update plan route
						if (now_car_p->plan_route.size() >= 2)
						{
							now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
						}
						road_p->prior_car = NULL;
						return;
					}
					else if (!not_break_out(r_road_p->to_cross_mat, r_road_p->channel)) // because of break out // every road'tail has car
					{
						// if tail are all end car -> turn end

						
						for (int i = 0; i <= r_road_p->channel - 1; i++)
						{
							if ((r_road_p->to_cross_mat[i][0] != NULL) && (r_road_p->to_cross_mat[i][0]->end == 1))
							{
								continue;
							}
							else if ((r_road_p->to_cross_mat[i][0] != NULL) && (r_road_p->to_cross_mat[i][0]->wait == 1))
							{
								road_p->prior_car = NULL;
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 1;
								now_car_p->end = 0;
								now_car_p->dir = 0;
								return;// else if tail has wait car -> hold on wait  //so not move still wait and erase the road
							}
						}
						road_p->prior_car = NULL;
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0; // not move and turn to end
						search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to);
						return;
					}
				}
				else if (tmp_next_aim_cross_id == r_road_p->from_cross->cross_id)
				{
					for (int i = 0; i <= r_road_p->channel - 1; i++)
					{
						for (int j = 0; j <= s2 - 1; j++)
						{
							if (r_road_p->from_cross_mat[i][0] != NULL)
							{
								break;
							}
							else if (r_road_p->from_cross_mat[i][j] == NULL)
							{
								continue;
							}
							else
							{
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 0;
								now_car_p->end = 1;
								now_car_p->dir = 0;

								if (now_car_p->on_to == true)
								{
									r_road_p->from_cross_mat[i][j - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}
								else if (now_car_p->on_from == true)
								{
									r_road_p->from_cross_mat[i][j - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
									road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
									search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
									now_car_p->on_to = false;
									now_car_p->on_from = true;
								}


								now_car_p->now_x = i;
								now_car_p->now_y = j - 1;
								now_car_p->now_road = r_road_p;
								//now_car_p->to_cross = r_road_p->to_cross;
								now_car_p->plan_route.erase(now_car_p->plan_route.begin());
								if (now_car_p->plan_route.size() >= 2)
								{
									now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
								}
								road_p->prior_car = NULL;
								return;
							}
						}
					}
					if ((r_road_p->from_cross_mat[0][s2 - 1] == NULL) && (not_break_out(r_road_p->from_cross_mat, r_road_p->channel))) //after all continue -> all null
					{
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0;

						if (now_car_p->on_to == true)
						{
							r_road_p->from_cross_mat[0][s2 - 1] = road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = false;
							now_car_p->on_from = true;
						}
						else if (now_car_p->on_from == true)
						{
							r_road_p->from_cross_mat[0][s2 - 1] = road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y];
							road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
							search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to); // now_x is old index
							now_car_p->on_to = false;
							now_car_p->on_from = true;
						}


						now_car_p->now_x = 0;
						now_car_p->now_y = s2 - 1;
						now_car_p->now_road = r_road_p;
						//now_car_p->to_cross = r_road_p->to_cross;
						now_car_p->plan_route.erase(now_car_p->plan_route.begin()); // update plan route
						if (now_car_p->plan_route.size() >= 2)
						{
							now_car_p->aim_cross_id = search_aim_cross_id(road_set[road_set_map[now_car_p->plan_route[0]]], road_set[road_set_map[now_car_p->plan_route[1]]]);
						}
						road_p->prior_car = NULL;
						return;
					}
					else if (!not_break_out(r_road_p->from_cross_mat, r_road_p->channel)) // because of break out // every road'tail has car
					{
						// if tail are all end car -> turn end
						for (int i = 0; i <= r_road_p->channel - 1; i++)
						{

							if ((r_road_p->from_cross_mat[i][0] != NULL) && (r_road_p->from_cross_mat[i][0]->end == 1))
							{
								continue;
							}
							else if ((r_road_p->from_cross_mat[i][0] != NULL) && (r_road_p->from_cross_mat[i][0]->wait == 1))
							{
								road_p->prior_car = NULL;
								find_and_erase(id_vec, road_p->road_id);
								now_car_p->wait = 1;
								now_car_p->end = 0;
								now_car_p->dir = 0;
								return;// else if tail has wait car -> hold on wait  //so not move still wait and erase the road
							}
							
						}
						road_p->prior_car = NULL;
						find_and_erase(id_vec, road_p->road_id);
						now_car_p->wait = 0;
						now_car_p->end = 1;
						now_car_p->dir = 0; // not move and turn to end
						search_back_car_turn_end(road_p, now_car_p->now_x, now_car_p->on_to);
						return;
					}
				}
			}
			else //route size == 1, go to dest, set null
			{
			find_and_erase(id_vec, road_p->road_id);
			now_car_p->to_cross = NULL;
			now_car_p->car_pos = false;
			now_car_p->arrived = true;
			now_car_p->wait = 0;
			now_car_p->end = 0;
			int old_channel_index = now_car_p->now_x;
			if (now_car_p->on_to == true)
			{
				road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
			}
			else if (now_car_p->on_from == true)
			{
				road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
			}

			cout << now_car_p->car_id << " arrived and arrived time is " << now_car_p->system_time << endl;
			count_car++;
			cout << "now arrived " << count_car << "cars" << endl;
			road_p->prior_car = NULL;
			search_back_car_turn_end(road_p, old_channel_index, now_car_p->on_to);
			}
		}
	}
	else if (now_car_p->dir == 0) // means plan_route size is 1, and arrive
	{
		find_and_erase(id_vec, road_p->road_id);
		now_car_p->to_cross = NULL;
		now_car_p->car_pos = false;
		now_car_p->arrived = true;
		now_car_p->wait = 0;
		now_car_p->end = 0;
		int old_channel_index = now_car_p->now_x;
		if (now_car_p->on_to == true)
		{
			road_p->to_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
		}
		else if (now_car_p->on_from == true)
		{
			road_p->from_cross_mat[now_car_p->now_x][now_car_p->now_y] = NULL;
		}

		cout << now_car_p->car_id << " arrived and arrived time is " << now_car_p->system_time << endl;
		count_car++;
		cout << "now arrived " << count_car << "cars" << endl;
		road_p->prior_car = NULL;
		search_back_car_turn_end(road_p, old_channel_index, now_car_p->on_to);
	}
}

bool not_break_out(Car ***from_cross_mat, int channel)
{
	for (int i = 0; i <= channel - 1; i++)
	{
		if (from_cross_mat[i][0] != NULL)
		{
			return false;
		}
	}
	return true;
}

void search_back_car_turn_end(Road *road, int channel_index, bool on_to)
{
	if (on_to == true)
	{
		for (int iy = road->length - 1; iy >= 0; iy--)
		{
			if (road->to_cross_mat[channel_index][iy] != NULL)
			{
				if (road->to_cross_mat[channel_index][iy]->wait == 1)
				{
					int v1 = min(road->to_cross_mat[channel_index][iy]->speed, road->speed);
					search_wait_end_car_in_speed_range(road, v1, channel_index, iy, on_to);
				}
			}
		}
	}
	else // on_to == false, on_from
	{
		for (int iy = road->length - 1; iy >= 0; iy--)
		{
			if (road->from_cross_mat[channel_index][iy] != NULL)
			{
				if (road->from_cross_mat[channel_index][iy]->wait == 1)
				{
					int v1 = min(road->from_cross_mat[channel_index][iy]->speed, road->speed);
					search_wait_end_car_in_speed_range(road, v1, channel_index, iy, on_to);
				}
			}
		}
	}

	

}

void search_wait_end_car_in_speed_range(Road *road, int v1, int i, int j, bool on_to) // and found end, change
{
	if (on_to == true)
	{
		if (j + v1 <= road->length - 1)
		{
			for (int iy = j + 1; iy <= j + v1; iy++)
			{
				if (road->to_cross_mat[i][iy] != NULL)
				{
					if (road->to_cross_mat[i][iy]->wait == 1)
					{
						return;
					}
					else if (road->to_cross_mat[i][iy]->end == 1)
					{
						road->to_cross_mat[i][j]->end = 1;
						road->to_cross_mat[i][j]->wait = 0;
						if (j < iy - 1) // just in front one place 
						{
							road->to_cross_mat[i][iy - 1] = road->to_cross_mat[i][j];
							road->to_cross_mat[i][j] = NULL;
						}
						search_back_car_turn_end(road, road->to_cross_mat[i][iy - 1]->now_x, road->to_cross_mat[i][iy - 1]->on_to);
						return;
					}
				}
			}
			road->to_cross_mat[i][j]->end = 1;
			road->to_cross_mat[i][j]->wait = 0;
			road->to_cross_mat[i][j + v1] = road->to_cross_mat[i][j];
			road->to_cross_mat[i][j] = NULL;
			search_back_car_turn_end(road, road->to_cross_mat[i][j + v1]->now_x, road->to_cross_mat[i][j + v1]->on_to);
			return;
		}
	}
	else // on_to == false
	{
		if (j + v1 <= road->length - 1)
		{
			for (int iy = j + 1; iy <= j + v1; iy++)
			{
				if (road->from_cross_mat[i][iy] != NULL)
				{
					if (road->from_cross_mat[i][iy]->wait == 1)
					{
						return;
					}
					else if (road->from_cross_mat[i][iy]->end == 1)
					{
						road->from_cross_mat[i][j]->end = 1;
						road->from_cross_mat[i][j]->wait = 0;
						if (j < iy - 1) // just in front one place 
						{
							road->from_cross_mat[i][iy - 1] = road->from_cross_mat[i][j];
							road->from_cross_mat[i][j] = NULL;
						}
						search_back_car_turn_end(road, road->from_cross_mat[i][iy - 1]->now_x, road->from_cross_mat[i][iy - 1]->on_to);
						return;
					}
				}
			}
			road->from_cross_mat[i][j]->end = 1;
			road->from_cross_mat[i][j]->wait = 0;
			road->from_cross_mat[i][j + v1] = road->from_cross_mat[i][j];
			road->from_cross_mat[i][j] = NULL;
			search_back_car_turn_end(road, road->from_cross_mat[i][j + v1]->now_x, road->from_cross_mat[i][j + v1]->on_to);
			return;
		}
	}

}

int find_min(vector<int> &id_vec)
{

	//int position = distance(begin(id_vec), min_);
	//id_vec.erase(id_vec.begin() + position);
	int min = id_vec[0];
	for (int i = 1; i <= id_vec.size() - 1; i++)
	{
		if (id_vec[i] < min)
		{
			min = id_vec[i];
		}
	}
	return min;
}


bool check_car_has_none_status(Car *car_set, int car_size)
{
	for (int k = 0; k <= car_size - 1; k++)
	{
		if (car_set[k].car_pos == true)
		{
			if ((car_set[k].wait == 1) || (car_set[k].end == 1))
			{
				int tmp_check = car_set[k].aim_cross_id;
				Road *now_road_check = car_set[k].now_road;
				int sp = min(now_road_check->speed, car_set[k].speed);
				bool tmp_to = car_set[k].on_to;
				bool tmp_from = car_set[k].on_from;
				int now_x = car_set[k].now_x;
				int now_y = car_set[k].now_y;
				continue;
			}
			else
			{
				return true;
			}
		}
	}
	return false;
}

void mark_end_wait(Road *road_set, int road_size)
{
	for (int r = 0; r <= road_size - 1; r++) //traversal every road
	{
		for (int j = road_set[r].length - 1; j >= 0; j--)   // mark end
		{
			for (int i = 0; i <= road_set[r].channel - 1; i++)
			{
				if ((road_set[r].to_cross_mat[i][j] != NULL)  && (road_set[r].to_cross_mat[i][j]->end == 0) ) // if not end car// to cross mat
				{
					int car_speed = road_set[r].to_cross_mat[i][j]->speed;
					if (no_front_car(road_set[r].to_cross_mat, i, j, road_set[r].length)) // no front car turn end
					{
						int now_speed = min(car_speed, road_set[r].speed);
						if (now_speed <= road_set[r].length - 1 - j)
						{
							road_set[r].to_cross_mat[i][j + now_speed] = road_set[r].to_cross_mat[i][j];
							road_set[r].to_cross_mat[i][j] = NULL;
							// update status
							road_set[r].to_cross_mat[i][j + now_speed]->end = 1;
							road_set[r].to_cross_mat[i][j + now_speed]->wait = 0; // do not need to know the old wait
							road_set[r].to_cross_mat[i][j + now_speed]->now_x = i;
							road_set[r].to_cross_mat[i][j + now_speed]->now_y = j + now_speed;
							road_set[r].to_cross_mat[i][j + now_speed]->on_to = true;
							road_set[r].to_cross_mat[i][j + now_speed]->on_from = false;
							search_back_car_turn_end(&road_set[r], road_set[r].to_cross_mat[i][j + now_speed]->now_x, road_set[r].to_cross_mat[i][j + now_speed]->on_to);
						}
					}
					else if (back_end_car(road_set[r].to_cross_mat, i, j, road_set[r].length) != 0) // back end car mark end
					{
						int dis = back_end_car_dis(road_set[r].to_cross_mat, i, j, road_set[r].length);
						int can_move = min(dis, car_speed);
						if (can_move > 0)
						{
							road_set[r].to_cross_mat[i][j + can_move] = road_set[r].to_cross_mat[i][j];
							road_set[r].to_cross_mat[i][j] = NULL;
							// update status
							road_set[r].to_cross_mat[i][j + can_move]->end = 1;
							road_set[r].to_cross_mat[i][j + can_move]->wait = 0; // do not need to know the old wait
							road_set[r].to_cross_mat[i][j + can_move]->now_x = i;
							road_set[r].to_cross_mat[i][j + can_move]->now_y = j + can_move;
							road_set[r].to_cross_mat[i][j + can_move]->on_to = true;
							road_set[r].to_cross_mat[i][j + can_move]->on_from = false;
							search_back_car_turn_end(&road_set[r], road_set[r].to_cross_mat[i][j + can_move]->now_x, road_set[r].to_cross_mat[i][j + can_move]->on_to);
						}
						else // can move = 0, set end directly
						{
							road_set[r].to_cross_mat[i][j]->end = 1;
							road_set[r].to_cross_mat[i][j]->wait = 0; // do not need to know the old wait
							road_set[r].to_cross_mat[i][j]->now_x = i;
							road_set[r].to_cross_mat[i][j]->now_y = j;
							road_set[r].to_cross_mat[i][j]->on_to = true;
							road_set[r].to_cross_mat[i][j]->on_from = false;
							search_back_car_turn_end(&road_set[r], road_set[r].to_cross_mat[i][j]->now_x, road_set[r].to_cross_mat[i][j]->on_to);
						}
					}
				}

				if ((road_set[r].from_cross_mat[i][j] != NULL) && (road_set[r].from_cross_mat[i][j]->end == 0)) // from cross mat
				{
					int car_speed = road_set[r].from_cross_mat[i][j]->speed;
					if (no_front_car(road_set[r].from_cross_mat, i, j, road_set[r].length)) // no front car turn end
					{
						int now_speed = min(car_speed, road_set[r].speed);
						if (now_speed <= road_set[r].length - 1 - j)
						{
							road_set[r].from_cross_mat[i][j + now_speed] = road_set[r].from_cross_mat[i][j];
							road_set[r].from_cross_mat[i][j] = NULL;
							// update status
							road_set[r].from_cross_mat[i][j + now_speed]->end = 1;
							road_set[r].from_cross_mat[i][j + now_speed]->wait = 0; // do not need to know the old wait
							road_set[r].from_cross_mat[i][j + now_speed]->now_x = i;
							road_set[r].from_cross_mat[i][j + now_speed]->now_y = j + now_speed;
							road_set[r].from_cross_mat[i][j + now_speed]->on_to = false;
							road_set[r].from_cross_mat[i][j + now_speed]->on_from = true;
							search_back_car_turn_end(&road_set[r], road_set[r].from_cross_mat[i][j + now_speed]->now_x, road_set[r].from_cross_mat[i][j + now_speed]->on_to);
						}
					}
					else if (back_end_car(road_set[r].from_cross_mat, i, j, road_set[r].length) != 0) // back end car mark end
					{
						int dis = back_end_car_dis(road_set[r].from_cross_mat, i, j, road_set[r].length);
						int can_move = min(dis, car_speed);
						can_move = min(can_move, road_set[r].speed);
						if (can_move > 0)
						{
							road_set[r].from_cross_mat[i][j + can_move] = road_set[r].from_cross_mat[i][j];
							road_set[r].from_cross_mat[i][j] = NULL;
							// update status
							road_set[r].from_cross_mat[i][j + can_move]->end = 1;
							road_set[r].from_cross_mat[i][j + can_move]->wait = 0; // do not need to know the old wait
							road_set[r].from_cross_mat[i][j + can_move]->now_x = i;
							road_set[r].from_cross_mat[i][j + can_move]->now_y = j + can_move;
							road_set[r].from_cross_mat[i][j + can_move]->on_to = false;
							road_set[r].from_cross_mat[i][j + can_move]->on_from = true;
							search_back_car_turn_end(&road_set[r], road_set[r].from_cross_mat[i][j + can_move]->now_x, road_set[r].from_cross_mat[i][j + can_move]->on_to);
						}
						else // can move = 0, set end directly
						{
							road_set[r].from_cross_mat[i][j]->end = 1;
							road_set[r].from_cross_mat[i][j]->wait = 0; // do not need to know the old wait
							road_set[r].from_cross_mat[i][j]->now_x = i;
							road_set[r].from_cross_mat[i][j]->now_y = j;
							road_set[r].from_cross_mat[i][j]->on_to = false;
							road_set[r].from_cross_mat[i][j]->on_from = true;
							search_back_car_turn_end(&road_set[r], road_set[r].from_cross_mat[i][j]->now_x, road_set[r].from_cross_mat[i][j]->on_to);
						}

					}
				}
			}
		}

		for (int j = road_set[r].length - 1; j >= 0; j--) // mark wait
		{
			for (int i = 0; i <= road_set[r].channel - 1; i++)
			{
				if ((road_set[r].to_cross_mat[i][j] != NULL) && (road_set[r].to_cross_mat[i][j]->end == 0) && (road_set[r].to_cross_mat[i][j]->wait == 0)) // if not end car and wait car // to cross mat // if already wait dont need to mark again
				{
					int car_speed = road_set[r].to_cross_mat[i][j]->speed;
					if (no_front_car(road_set[r].to_cross_mat, i, j, road_set[r].length)) // no front car turn wait
					{
						int now_speed = min(car_speed, road_set[r].speed);
						if (now_speed > road_set[r].length - 1 - j)
						{

							if (road_set[r].to_cross_mat[i][j]->plan_route.size() > 1)
							{
								road_set[r].to_cross_mat[i][j]->wait = 1;
								road_set[r].to_cross_mat[i][j]->end = 0;
								make_wait_dir_flag(road_set[r], road_set[r].to_cross_mat[i][j]); // will go out cross / make dir
							}
							else //size == 1  arrive
							{
								/////arrive
								//find_and_erase(id_vec, road_p->road_id);
								road_set[r].to_cross_mat[i][j]->to_cross = NULL;
								road_set[r].to_cross_mat[i][j]->car_pos = false;
								road_set[r].to_cross_mat[i][j]->arrived = true;
								road_set[r].to_cross_mat[i][j]->wait = 0;
								road_set[r].to_cross_mat[i][j]->end = 0;
								road_set[r].to_cross_mat[i][j]->on_to = false;
								road_set[r].to_cross_mat[i][j]->on_from = false;
								cout << road_set[r].to_cross_mat[i][j]->car_id << " arrived and arrived time is " << road_set[r].to_cross_mat[i][j]->system_time << endl;
								count_car++;
								cout << "now arrived " << count_car << "cars" << endl;
								road_set[r].to_cross_mat[i][j] = NULL;
								bool tmp_on_to = true;
								search_back_car_turn_end(&road_set[r], i, tmp_on_to);
							}
						}
					}
					else // back wait car mark wait/end
					{
						back_wait_car(road_set[r].to_cross_mat, i, j, road_set[r]); // back wait car dont have dir
					}
				}

				if ((road_set[r].from_cross_mat[i][j] != NULL) && (road_set[r].from_cross_mat[i][j]->end == 0) && (road_set[r].from_cross_mat[i][j]->wait == 0)) // if not end car and wait car // to cross mat
				{
					int car_speed = road_set[r].from_cross_mat[i][j]->speed;
					if (no_front_car(road_set[r].from_cross_mat, i, j, road_set[r].length)) // no front car turn wait
					{
						int now_speed = min(car_speed, road_set[r].speed);
						if (now_speed > road_set[r].length - 1 - j)
						{
							if (road_set[r].from_cross_mat[i][j]->plan_route.size() > 1)
							{
								road_set[r].from_cross_mat[i][j]->wait = 1;
								road_set[r].from_cross_mat[i][j]->end = 0;
								make_wait_dir_flag(road_set[r], road_set[r].from_cross_mat[i][j]);// will go out cross / make dir
							}
							else //size == 1  arrive
							{
								/////arrive
								//find_and_erase(id_vec, road_p->road_id);
								road_set[r].from_cross_mat[i][j]->to_cross = NULL;
								road_set[r].from_cross_mat[i][j]->car_pos = false;
								road_set[r].from_cross_mat[i][j]->arrived = true;
								road_set[r].from_cross_mat[i][j]->wait = 0;
								road_set[r].from_cross_mat[i][j]->end = 0;
								road_set[r].from_cross_mat[i][j]->on_to = false;
								road_set[r].from_cross_mat[i][j]->on_from = false;
								cout << road_set[r].from_cross_mat[i][j]->car_id << " arrived and arrived time is " << road_set[r].from_cross_mat[i][j]->system_time << endl;
								count_car++;
								cout << "now arrived " << count_car << "cars" << endl;
								road_set[r].from_cross_mat[i][j] = NULL;
								bool tmp_on_to = false;
								search_back_car_turn_end(&road_set[r], i, tmp_on_to);
							}

						}
					}
					else // back wait car mark wait/end
					{
						back_wait_car(road_set[r].from_cross_mat, i, j, road_set[r]);
					}
				}
			}	
		}	
	}
}

void make_wait_dir_flag(Road road, Car *car)
{
	if (car->plan_route.size() >= 2)
	{
		if (car->aim_cross_id == road.to_cross->cross_id)
		{
			if (road.road_id == road.to_cross->road_1_id)
			{
				if (car->plan_route[1] == road.to_cross->road_2_id) //L
				{
					car->dir = 2;
				}
				else if (car->plan_route[1] == road.to_cross->road_3_id) //D
				{
					car->dir = 3;
				}
				else if (car->plan_route[1] == road.to_cross->road_4_id) //R
				{
					car->dir = 1;
				}
			}
			else if (road.road_id == road.to_cross->road_2_id)
			{
				if (car->plan_route[1] == road.to_cross->road_1_id)
				{
					car->dir = 1;
				}
				else if (car->plan_route[1] == road.to_cross->road_3_id)
				{
					car->dir = 2;
				}
				else if (car->plan_route[1] == road.to_cross->road_4_id)
				{
					car->dir = 3;
				}
			}
			else if (road.road_id == road.to_cross->road_3_id)
			{
				if (car->plan_route[1] == road.to_cross->road_1_id) //L
				{
					car->dir = 3;
				}
				else if (car->plan_route[1] == road.to_cross->road_2_id) //D
				{
					car->dir = 1;
				}
				else if (car->plan_route[1] == road.to_cross->road_4_id) //R
				{
					car->dir = 2;
				}
			}
			else if (road.road_id == road.to_cross->road_4_id)
			{
				if (car->plan_route[1] == road.to_cross->road_1_id) //L
				{
					car->dir = 2;
				}
				else if (car->plan_route[1] == road.to_cross->road_2_id) //D
				{
					car->dir = 3;
				}
				else if (car->plan_route[1] == road.to_cross->road_3_id) //R
				{
					car->dir = 1;
				}
			}
		}
		// if dir still = 0, which means the next road id is in from_cross
		if (car->aim_cross_id == road.from_cross->cross_id)
		{
			if (road.road_id == road.from_cross->road_1_id)
			{
				if (car->plan_route[1] == road.from_cross->road_2_id) //L
				{
					car->dir = 2;
				}
				else if (car->plan_route[1] == road.from_cross->road_3_id) //D
				{
					car->dir = 3;
				}
				else if (car->plan_route[1] == road.from_cross->road_4_id) //R
				{
					car->dir = 1;
				}
			}
			else if (road.road_id == road.from_cross->road_2_id)
			{
				if (car->plan_route[1] == road.from_cross->road_1_id)
				{
					car->dir = 1;
				}
				else if (car->plan_route[1] == road.from_cross->road_3_id)
				{
					car->dir = 2;
				}
				else if (car->plan_route[1] == road.from_cross->road_4_id)
				{
					car->dir = 3;
				}
			}
			else if (road.road_id == road.from_cross->road_3_id)
			{
				if (car->plan_route[1] == road.from_cross->road_1_id) //L
				{
					car->dir = 3;
				}
				else if (car->plan_route[1] == road.from_cross->road_2_id) //D
				{
					car->dir = 1;
				}
				else if (car->plan_route[1] == road.from_cross->road_4_id) //R
				{
					car->dir = 2;
				}
			}
			else if (road.road_id == road.from_cross->road_4_id)
			{
				if (car->plan_route[1] == road.from_cross->road_1_id) //L
				{
					car->dir = 2;
				}
				else if (car->plan_route[1] == road.from_cross->road_2_id) //D
				{
					car->dir = 3;
				}
				else if (car->plan_route[1] == road.from_cross->road_3_id) //R
				{
					car->dir = 1;
				}
			}
		}
	}
	else // plan_route size is 1, dir is 0
	{
		return;
	}
	
}


void back_wait_car(Car ***cross_mat, int i, int j, Road road)
{
	for (int yj = j + 1; yj <= road.length - 1; yj++)
	{
		if (cross_mat[i][yj] != NULL)
		{
			if (cross_mat[i][yj]->wait == 1) //back wait car
			{
				int dis = yj - j - 1;
				int speed = min(cross_mat[i][yj]->speed, road.speed);
				if (dis >= speed) // update to end
				{
										
					cross_mat[i][j + speed] = cross_mat[i][j];
					cross_mat[i][j] = NULL;
					cross_mat[i][j + speed]->end = 1;
					cross_mat[i][j + speed]->wait = 0;
					cross_mat[i][j + speed]->now_x = i;
					cross_mat[i][j + speed]->now_y = j + speed;
					search_back_car_turn_end(&road, i, cross_mat[i][j + speed]->on_to);
					return;
				}
				else // update to wait
				{
					cross_mat[i][j]->wait = 1; // will not go out cross / no need dir
					cross_mat[i][j]->end = 0;
					return;
				}
			}
		}
	}
}



bool back_end_car(Car ***to_cross_mat, int i, int j, int length)
{
	for (int yj = j + 1; yj <= length - 1; yj++)
	{
		if (to_cross_mat[i][yj] != NULL)
		{
			if (to_cross_mat[i][yj]->end == 1)
			{
				
				return true;//yj - j - 1; // can move 
			}
			else
				return false;
		}
	}
	return false;
}

int back_end_car_dis(Car ***to_cross_mat, int i, int j, int length)
{
	for (int yj = j + 1; yj <= length - 1; yj++)
	{
		if (to_cross_mat[i][yj] != NULL)
		{
			if (to_cross_mat[i][yj]->end == 1)
			{

				return yj - j - 1; // can move 
			}
		}
	}
}

bool no_front_car(Car ***to_cross_mat, int i, int j, int length) //not means speed range, means ahead of this channel 
{
	for (int yj = length - 1; yj > j; yj--)
	{
		if (to_cross_mat[i][yj] != NULL)
		{
			return false;
		}
	}
	return true;
	
}

