#include <veins/modules/MCTA/ConvexHullAlgorithms.h>


bool sortPoints(const Coord &lhs, const Coord &rhs){
    return (lhs.x < rhs.x) || (lhs.x==rhs.x && lhs.y < rhs.y);
}

// Check if three points make a right turn using cross product
bool right_turn(const Coord &P1, const Coord &P2, const Coord &P3)
{
    return ((P3.x-P1.x)*(P2.y-P1.y) - (P3.y-P1.y)*(P2.x-P1.x)) > 0;
}
convexhull::convexhull(){

}
convexhull::~convexhull(){

}
vector<Coord> convexhull::getContourConvex(std::vector<Coord> ContPos){
    int n = ContPos.size();
    Coord Points[n];
    int k=0;
    for (auto i:ContPos){
        Points[k] =i;
        k++;
    }
    vector<Coord> ConvHul = returnConvexHull(Points,n);
    std::cout<<std::endl<<"CONVEX HULL VERTICES : " <<std::endl;
    for(auto j:ConvHul){
        std::cout <<"("<<j.x <<"; "<<j.y <<")"<<std::endl;
    }
    std::cout <<std::endl<<std::endl;
//    double Area = returnConvexArea(ConvHul);

    return ConvHul;
}
vector<Coord> convexhull::returnConvexesIntersection(std::vector<Coord> Conv1, std::vector<Coord> Conv2){
    vector<Coord> interpos;
    for (auto i:Conv1){
        bool flag1 = false, flag2 = false, flag3 = false, flag4 = false, check1 = false, check2 = false;
        for (auto j:Conv2){
            if (flag1 == true && flag2 == true){
                check1 = true;
            }
            else if (j.x < i.x){
                flag1 = true;
            }
            else if (j.y < j.y){
                flag2 = true;
            }
            if (flag3 == true && flag4 == true){
                check2 = true;
            }
            else if (j.x > i.x){
                flag3 = true;
            }
            else if (j.y > j.y){
                flag4 = true;
            }
        }
        if (check1 == true && check2 == true){
            interpos.push_back(i);
        }
    }
    return interpos;
}

vector<Coord> convexhull::returnConvexHull(Coord *points, int n_points){
    vector<Coord> c_hull;//Convex vertices
    vector<Coord> lowerCH;
    vector<Coord> upperCH;
    //Sorting points
    sort(points, points + n_points, sortPoints);
  /*  cout << "Sorted Points\n";
    for(int i = 0; i != n_points; ++i){
        cout << "(" << points[i].x << " , " << points[i].y << ")" << endl;
    }*/

    //Computing upper convex hull
    upperCH.push_back(points[0]);
    upperCH.push_back(points[1]);

    for(int i=2; i<n_points; i++)
    {
        while(upperCH.size() > 1 and (!right_turn(upperCH[upperCH.size()-2],upperCH[upperCH.size()-1], points[i])))
            upperCH.pop_back();
        upperCH.push_back(points[i]);
    }
/*
    cout << "Upper Covex Hull" << endl;
    for(int i=0; i < upperCH.size(); i++)
        cout << "(" << upperCH[i].x << " , " << upperCH[i].y << ")" << endl;
*/

    //Computing lower convex hull
    lowerCH.push_back(points[n_points-1]);
    lowerCH.push_back(points[n_points-2]);

    for(int i=2; i< n_points; i++)
    {
        while(lowerCH.size() > 1 and (!right_turn(lowerCH[lowerCH.size()-2],lowerCH[lowerCH.size()-1], points[n_points-i-1])))
            lowerCH.pop_back();
        lowerCH.push_back(points[n_points-i-1]);
    }
/*    cout << "Lower Covex Hull" << endl;
    for(int i=0; i < lowerCH.size(); i++)
        cout << "(" << lowerCH[i].x << " , " << lowerCH[i].y << ")" << endl;*/

    c_hull = upperCH;
    c_hull.insert(c_hull.end(), lowerCH.begin(), lowerCH.end());
    return c_hull;
}
double convexhull::returnConvexArea(vector<Coord> ConvexVertices){
    int n = ConvexVertices.size();
    double area;
    for(int j, i = 0; i < n; i++) {
        j = (i + 1);
        if (i < n-1){
            area += ConvexVertices[i].x*ConvexVertices[j].y;
            area -= ConvexVertices[i].y*ConvexVertices[j].x;
        }
        else{
            area += ConvexVertices[i].x * ConvexVertices[0].y;
            area-= ConvexVertices[i].y*ConvexVertices[0].x;
        }
    }
    return fabs(area);
}
