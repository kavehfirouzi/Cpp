
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int read_file(float* data, string filename)
{
	string line;
	ifstream myfile (filename);
	int counter=0;
	if (myfile.is_open())
	{
		
		while ( getline (myfile,line,',') )
		{
		  data[counter++]=::atof(line.c_str());
		}
		myfile.close();
	}
	return counter;
}