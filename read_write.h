#pragma once
#ifndef READ_WRITE_H
#define READ_WRITE_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include "method.h"
#include <assert.h>
using namespace std;


class read_write_txt
{
public:
	int test2(int a, int b);
	vector<vector<int>> read_txt(string file, const char* txt_class);
	void write_txt(string file, vector<vector<int>> &ans_tab);
	ostream& writemap(ostream& os, vector<vector<int>> &ans_tab);
	//void write_txt(string file);

};

int read_write_txt::test2(int a, int b)
{
	int c = a + b;
	return c;
}

vector<vector<int>> read_write_txt::read_txt(string file, const char* txt_class)
{
	ifstream infile;
	
	infile.open(file.data()); //connect the file stream object with the file
	
	assert(infile.is_open()); //if false, output the error message and stop

	string line;
	
	vector<vector<int>> mat;
	vector<int> vec;
	
	int n_row = 0;
	int n_col; 
	
	// set the col num
	if (!strcmp(txt_class, "road"))
	{
		n_col = 7;
	}

	else if(!strcmp(txt_class, "car"))
	{
		n_col = 5;
	}
	else if(!strcmp(txt_class, "cross")) 
	{
		n_col = 5;
	}
	
	// compute the row num
	while(getline(infile, line))
	{
		if (line[0] == '#') {continue;}

		n_row++;
	}
	cout << "num of row is " << n_row << endl;
	
	infile.close();

	infile.open(file.data());

	
	// extract data
	int i = 0;

	while(getline(infile, line))
	{
		int j = 0;

		if (line[0] == '#') {continue;}

		line.erase(0, 1);

		line.erase(line.size() - 1, 1);

		char* line_ = (char *)line.c_str();
		
		string pat = ",";

		const char *pattern = pat.c_str();

		char* tmp_str = strtok(line_, pattern);;

		while(tmp_str)
		{
			vec.push_back(atoi(tmp_str));
			
			j++;

			tmp_str = strtok(NULL, pattern);
		}

		mat.push_back(vec);
		
		vec.clear();

		printf("\n");
		
		i++;

	 }   

	infile.close();
	if ((!strcmp(txt_class, "road"))||(!strcmp(txt_class, "car"))||(!strcmp(txt_class, "cross")))
	{
		for (int i = 0; i < n_row; i++)
		{
			for (int j = 0; j < n_col; j++)
			{
				cout << mat[i][j] << " ";
			}
			cout << endl;
		}
	}
	else // answer
	{
		for (int i = 0; i < n_row; i++)
		{
			vector<int> tmp_vec = mat[i];
			for (int j = 0; j <= tmp_vec.size() - 1; j++)
			{
				cout << tmp_vec[j] << " ";
			}
			cout << endl;
		}
	}


	return mat;

}
ostream& read_write_txt::writemap(ostream& os, vector<vector<int>> &ans_tab)
{
	for (int i = 0; i <= ans_tab.size() - 1; ++i)
	{
		os << "(";
		for (int j = 0; j <= ans_tab[i].size() - 1; ++j)
		{
			if (j != ans_tab[i].size() - 1)
			{
				os << ans_tab[i][j] << ", ";
			}
			else
			{
				os << ans_tab[i][j] << ")";
			}
			
		}
		os << "\n";
	}
	return os;
}
void read_write_txt::write_txt(string file, vector<vector<int>> &ans_tab)
{
	fstream of(file, ios::out);
	if (of.is_open())
	{
		writemap(of, ans_tab);
		//writemap(cout, ans_tab);
		of.close();
	}
}
#endif
