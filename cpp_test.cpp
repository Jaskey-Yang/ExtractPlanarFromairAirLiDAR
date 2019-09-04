#include <stdio.h>
#include <stdlib.h>
#include "MeanShift.h"
#include<string>
#include"Readlas.h"
using namespace std;

vector<vector<double> > load_points(const char * filename)
{
    vector<vector<double> > points;
    FILE *fp = fopen(filename, "r");
    char line[50];
    while (fgets(line, sizeof(line), fp) != NULL) {
        double x, y;
        char *x_str = line;
        char *y_str = line;
        while (*y_str != '\0') {
            if (*y_str == ',') {
                *y_str++ = 0;
                x = atof(x_str);
                y = atof(y_str);
                vector<double> point;
                point.push_back(x);
                point.push_back(y);
                points.push_back(point);
                break;
            }
            ++y_str;
        }
    }
    fclose(fp);
    return points;
}

void print_points(vector<vector<double> > points){
    for(int i=0; i<points.size(); i++){
        for(int dim = 0; dim<points[i].size(); dim++) {
            printf("%f ", points[i][dim]);
        }
        printf("\n");
    }
}

int main2(int argc, char **argv)
{
	DataIO dataIO;
    MeanShift *msp = new MeanShift();
    double kernel_bandwidth = 3;
	std::string filepath = "D:/testCloud.las";
	dataIO.SetFilePath(filepath);
	std::vector<pcl::PointXYZI> srcPointCloud;
	dataIO.ReadlasXYZI(srcPointCloud);
	std::vector<Cluster3D> cluster3D = msp->cluster3D(srcPointCloud, kernel_bandwidth);

	//const char* filePath = "D:\\C++\ExtractBuilding\\ExtractBuilding\\ExtractBuilding\\test.csv";
	//std::vector<std::vector<double> > points = load_points(filePath);
	//print_points(points);
	//vector<Cluster> clusters = msp->cluster(points, kernel_bandwidth);

    FILE *fp = fopen("result.csv", "w");
    if(!fp){
        perror("Couldn't write result.csv");
        exit(0);
    }

    printf("\n====================\n");
    printf("Found %lu clusters\n", cluster3D.size());
    printf("====================\n\n");
    for(int cluster = 0; cluster < cluster3D.size(); cluster++) {
      printf("Cluster %i:\n", cluster);
      for(int point = 0; point < cluster3D[cluster].original_points.size(); point++)
	  {
		  printf("%f ", cluster3D[cluster].original_points.at(point).x);
		  fprintf(fp, 0 ? ",%f" : "%f", cluster3D[cluster].original_points.at(point).x);
		  printf("%f ", cluster3D[cluster].original_points.at(point).y);
		  fprintf(fp, 1 ? ",%f" : "%f", cluster3D[cluster].original_points.at(point).y);
		  printf("%f ", cluster3D[cluster].original_points.at(point).z);
		  fprintf(fp, 2 ? ",%f" : "%f", cluster3D[cluster].original_points.at(point).z);
       /* for(int dim = 0; dim < cluster3D[cluster].original_points[point].size(); dim++)
		{
          printf("%f ", cluster3D[cluster].original_points[point][dim]);
          fprintf(fp, dim?",%f":"%f", cluster3D[cluster].original_points[point][dim]);
        }*/
        printf(" -> ");
        for(int dim = 0; dim < cluster3D[cluster].shifted_points.size(); dim++)
		{
          printf("%f ", cluster3D[cluster].shifted_points.at(dim).x);
		  printf("%f ", cluster3D[cluster].shifted_points.at(dim).y);
		  printf("%f ", cluster3D[cluster].shifted_points.at(dim).z);
        }
        printf("\n");
        fprintf(fp, "\n");
      }
      printf("\n");
    }
    fclose(fp);
	system("pause");
    return 0;
}
