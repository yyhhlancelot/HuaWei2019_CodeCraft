#pragma once
#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <iostream>

using namespace std;

#define INF 100000
const int DefaultVertices = 30;
template<class T, class E>
class Graph
{
public:
	//int numVertices;
	static const T maxWeight = INF;
	Graph(int size = DefaultVertices){};
	~Graph(){};
	int NumberOfVertices(){return numVertices;}
	int NumberOfEdges(){return numEdges;}

	virtual bool insertEdge(int id, int length, int speed, int channel, int from, int to, bool isDuplex, map<int, int> cross_index_map) = 0;


protected:
	int maxVertices;
	int numEdges;
	int numVertices;
};

template<class T, class E>
class Graphmtx : public Graph<T, E>
{
public : 
	E **Length;
	E **IsDuplex;
	E **Speed;

	Graphmtx(E sz); //= DefaultVertices);
	~Graphmtx()
	{
		delete []Length;
		delete []IsDuplex;
		delete[]Speed;
	}

	E getWeight(int from, int to)
	{
		if(from != -1 && to != -1)
			return Length[from][to];
		else
			return 0;
	}
	
	bool insertEdge(int id, int length, int speed, int channel, int from, int to, bool isDuplex, map<int, int> cross_index_map);
	void generate_adjacency_mat(vector<vector<int>> tab, map<int, int> cross_index_map);
	void outputGraph();

	
/*private:
	T *VerticesList;
	//E **Edge;	
	E **Length;
	E **Id;
	E **Speed;
	E **Channel;
	E **IsDuplex; */
};

template<class T, class E>
void Graphmtx<T, E>::generate_adjacency_mat(vector<vector<int>> tab, map<int, int> cross_index_map)
{
	for (int i = 0; i <= tab.size() - 1; i++)
	{
		insertEdge(tab[i][0], tab[i][1], tab[i][2], 
		tab[i][3], tab[i][4], tab[i][5], tab[i][6], cross_index_map);
	}
}

template<class T, class E>
Graphmtx<T, E>::Graphmtx(E cross_size) // constructor
{

	this->maxVertices = cross_size + 1;
	this->numVertices = 0;
	this->numEdges = 0;
	int i, j;

	Length = new E*[this->maxVertices];
	IsDuplex = new E*[this->maxVertices];
	Speed = new E*[this->maxVertices];

	for (i = 0; i <= this->maxVertices - 1; i++)
	{
		Length[i] = new E[this->maxVertices];
		IsDuplex[i] = new E[this->maxVertices];
		Speed[i] = new E[this->maxVertices];
	}
	for (i = 0; i <= this->maxVertices - 1; i++)
	{
		for (j = 0; j <= this->maxVertices - 1; j++)
		{
			Length[i][j] = (i == j) ? 0 : this->maxWeight;
			IsDuplex[i][j] = (i == j) ? 0 : this->maxWeight;
			Speed[i][j] = (i == j) ? 0 : this->maxWeight;
		}
	}
}

template<class T, class E>
bool Graphmtx<T, E>::insertEdge(int id, int length, int speed, int channel, int from, int to, bool isDuplex, map<int, int> cross_index_map)
{
	if (from >= 0 && to >= 0)
	{
		if (Length[cross_index_map[from]][cross_index_map[to]] == this->maxWeight)
		{
			Speed[cross_index_map[from]][cross_index_map[to]] = speed;
			Speed[cross_index_map[to]][cross_index_map[from]] = speed;
			IsDuplex[cross_index_map[from]][cross_index_map[to]] = isDuplex;
			if (IsDuplex[cross_index_map[from]][cross_index_map[to]])
			{
				Length[cross_index_map[from]][cross_index_map[to]] = length;
				Length[cross_index_map[to]][cross_index_map[from]] = length;
			}
			else
			{
				Length[cross_index_map[from]][cross_index_map[to]] = length;
			}
			
		}
		else
		{
			cout << "already exists and fail" << endl;
			return false;
		}
	}
	else
	{
		return false;
	}

}

template<class T, class E>
void Graphmtx<T, E>::outputGraph()
{	
	cout << "the isduplex adjacency mat:" << endl;
	for (int i = 0; i <= this->maxVertices - 1; i++)
	{
		for (int j = 0; j <= this->maxVertices - 1; j++)
		{
			cout << IsDuplex[i][j] << " ";
		}
		cout << endl;
	}
	cout << "the length adjacency mat:" << endl;
	for (int i = 0; i <= this->maxVertices - 1; i++)
	{
		for (int j = 0; j <= this->maxVertices - 1; j++)
		{
			cout << Length[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}

#endif
