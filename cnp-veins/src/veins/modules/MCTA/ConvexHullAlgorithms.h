#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include "veins/base/utils/Coord.h"

#include <math.h>

using namespace std;


class convexhull: public Coord{
    public:
        convexhull();
        ~convexhull();
        vector<Coord> returnConvexHull(Coord *points, int npts);
        vector<Coord> getContourConvex(std::vector<Coord> ContPos);
        vector<Coord> returnConvexesIntersection(std::vector<Coord> Conv1, std::vector<Coord> Conv2);
        double returnConvexArea(vector<Coord> ConvexVertices);

};

