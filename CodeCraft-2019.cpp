#include "stdafx.h"
#include <iostream>
#include "method.h"
#include "read_write.h"
#include "graph.h"
#include "math.h"
#include "process_2.h"
#include "addition.h"
#include <algorithm>
#include <ctime>
using namespace std;

void test_func(Cross &cross);
void test_func_2(Car **road);
int main(int argc, char *argv[])
{
	clock_t startTime, endTime;
	startTime = clock();
	std::cout << "Begin" << std::endl;

	if(argc < 5){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}
	//char *argv[5];
	//argv[0] = " ";
	//argv[1] = "D://project//HuaWei//Huawei2019//exam_1//car.txt";
	//argv[2] = "D://project//HuaWei//Huawei2019//exam_1//road.txt";
	//argv[3] = "D://project//HuaWei//Huawei2019//exam_1//cross.txt";
	//argv[4] = "D://project//HuaWei//Huawei2019//exam_1//answer.txt";
	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string answerPath(argv[4]);

	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;


	//////////////////// TODO:read input filebuf
	read_write_txt t;
	//vector<vector<int>> cross_tab = t.read_txt(crossPath, "cross");
	vector<vector<int>> cross_tab = t.read_txt(crossPath, "cross");
	vector<vector<int>> road_tab = t.read_txt(roadPath, "road");
	vector<vector<int>> car_tab = t.read_txt(carPath, "car");


	/////////////////// TODO:process
	// cross_tab index map
	int cross_size = cross_tab.size();
	map<int, int> cross_index_map;
	map<int, int> index_cross_map;
	for (int i = 1; i <= cross_size; i++) // index from 1 start
	{
		cross_index_map.insert(pair<int, int>(cross_tab[i - 1][0], i)); // cross_id, i 
		index_cross_map.insert(pair<int, int>(i, cross_tab[i - 1][0])); // i, cross_id
	}

	// generate adjacency matrixes
	//Graphmtx<int, int> graph(cross_tab.size() + 1); // cross id from 1 start
	Graphmtx<int, int> graph(cross_size); // cross_id not from 1 start
	graph.generate_adjacency_mat(road_tab, cross_index_map);
	//graph.outputGraph();


	// preprocess
	// cross object set
	
	Cross *cross_set = new Cross[cross_size];  // in dump, need delete at last
	for (int j = 0; j <= cross_size - 1; j++) // construct cross set
	{
		cross_set[j].get_road_tab(road_tab);
		cross_set[j].build_all(cross_tab[j]); // need rewrite
	}
	//cross map
	map<int, int> cross_set_map; // cross_id, index
	for (int j = 0; j <= cross_size - 1; j++)
	{
		cross_set_map.insert(pair<int, int>(cross_set[j].cross_id, j));
	}

	// build all cross
	const int road_size = road_tab.size();


	// cross-cross road map
	map<pair<int, int>, int> cross_cross_road_map;
	for (int j = 0; j <= road_size - 1; j++)
	{
		if (road_tab[j][6] == true)
		{
			cross_cross_road_map.insert(pair<pair<int, int>, int>(make_pair(road_tab[j][4], road_tab[j][5]), road_tab[j][0]));
			cross_cross_road_map.insert(pair<pair<int, int>, int>(make_pair(road_tab[j][5], road_tab[j][4]), road_tab[j][0]));
		}
		else
		{
			cross_cross_road_map.insert(pair<pair<int, int>, int>(make_pair(road_tab[j][4], road_tab[j][5]), road_tab[j][0]));
		}
	}

	Road *road_set = new Road[road_size];
	for (int r = 0; r <= road_size - 1; r++)
	{
		road_set[r].build_all(road_tab[r]);
		road_set[r].init(); // build car pointer mat
	}
	 //road map
	map<int, int> road_set_map; // cross_id, index
	for (int r = 0; r <= road_size - 1; r++)
	{
		road_set_map.insert(pair<int, int>(road_set[r].road_id, r));
	}
	 //cross point to roads
	for (int j = 0; j <= cross_size - 1; j++) // construct cross set
	{
		cross_set[j].point_road(cross_tab[j], road_set_map, road_set);
	}
	 //road point to cross
	for (int r = 0; r <= road_size - 1; r++)
	{
		road_set[r].point_cross(road_tab[r], cross_set_map, cross_set);
	}
	 //road-road cross map
	map<pair<int, int>, int> road_road_cross_map;
	for (int c = 0; c <= cross_size - 1; c++)
	{
		generate_road_road_cross_map(cross_set[c], road_road_cross_map);
	}
	// build all cars
	// ans_map
	const int car_size = car_tab.size();

	vector<int> garage;
	Car *car_set = new Car[car_size]; // in dump, need delete at last
	for (int k = 0; k <= car_size - 1; k++) // construct car set
	{
		garage.push_back(car_tab[k][0]);
		car_set[k].set(car_tab[k], graph.Length, graph.Speed, cross_size);
		//car_set[k].Dijkstra(graph.Length, cross_size, cross_cross_road_map); // path algorithm
		car_set[k].Dijkstra_t(car_set[k].time_weight, cross_size, cross_cross_road_map, cross_tab, cross_index_map, index_cross_map);
		//car_set[k].two_path_merge(car_set[k].time_weight, cross_size, cross_cross_road_map);
		for (int i = 0; i <= cross_size; i++)
		{
			delete[]car_set[k].time_weight[i];
		}
		delete[]car_set[k].time_weight;
	}
	// car map
	map<int, int> car_set_map; // car_id, index of car_set
	for (int k = 0; k <= car_size - 1; k++)
	{
		car_set_map.insert(pair<int, int>(car_set[k].car_id, k));
	}
	 //car point to road and cross(to cross, from cross)
	for (int k = 0; k <= car_size - 1; k++) // construct cross set
	{
		car_set[k].point_road(road_set_map, road_set);
		car_set[k].point_cross();
	}

	// dijik_t, if find same route change, set original length 5
	vector<vector<int>> store_short_route;

	for (int k = 0; k <= car_size - 1; k++)
	{
		if (car_set[k].plan_route.size() <= 5)
		{
			int vec_num = count(store_short_route.begin(), store_short_route.end(), car_set[k].plan_route); // check if store before
			if (vec_num == 0)
			{
				store_short_route.push_back(car_set[k].plan_route);
			}
		}
	}

	// change long route
	//change_long_route(store_short_route, car_set, car_size, road_road_cross_map, cross_set_map, cross_set, cross_size, cross_cross_road_map);
	

	// if above route occur in some long route, change the long route


	vector<vector<int>> ans_tab;
	for (int k = 0; k <= car_size - 1; k++) // construct cross set
	{
		vector<int> tmp_vec;
		tmp_vec = car_set[k].plan_route;

		tmp_vec.insert(tmp_vec.begin(), car_tab[k][4]);
		tmp_vec.insert(tmp_vec.begin(), car_tab[k][0]);
		
		for (int i = 0; i <= tmp_vec.size() - 1; i++)
		{
			cout << tmp_vec[i] << " ";
		}
		cout << endl;
		ans_tab.push_back(tmp_vec);
	}

	////// test
	int time = 1;
	int time_unit_car_num = 70;
	int time_interval = 50;
	while(time < 2500)
	{
		int count = 0;
		vector<int> temp_index_vec;
		for (int i = 0; i <= ans_tab.size() - 1; i++)
		{

			if (ans_tab[i][1] == time)
			{
				count++;
				temp_index_vec.push_back(i);
			}
			
		}
		cout << "t = " << time << " count " << count << endl;
		time++;
		
		if (temp_index_vec.size() > time_unit_car_num)
		{
			//random_shuffle(temp_index_vec.begin(), temp_index_vec.end());
			int more = count - time_unit_car_num;
			for (int i = 0; i <= more - 1; i++) // front more
			{
				ans_tab[temp_index_vec[i]][1] += time_interval; // mid time
			}
		}
	}


	t.write_txt(answerPath, ans_tab);
	endTime = clock();
	cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	process_2(ans_tab, car_set, car_size, car_tab, car_set_map, road_set, road_size, road_set_map, cross_set, cross_size);







	////////////////// TODO:write output file

	return 0;

}

void test_func(Cross &cross)
{
	cross.cross_id = 88;
	return;
}
void test_func_2(Car **road)
{
	road[0][0].car_id = 3;
}
