#include<iostream>
using namespace std ; 
using std::vector;
#include <cmath>


class joint_movement {

    private : 
    int degrees_arr[6];
    double pi = 3.14159;
    vector <double> rad_arr ; 

    public : 
    int * get_deg_from_rad(vector <double> joint_group_positions){
        cout<<joint_group_positions.size()<<endl;
        for(unsigned int i = 0; i < joint_group_positions.size(); i++){
            double element = joint_group_positions [i];
            element = element * (180 / pi); // convert joint from radians to degrees
            element = round (element) ; 
            //std::cout << "joint No " << i +1 << " _________________"<< element << std::endl;
            degrees_arr[i] = element ;
            };
        return degrees_arr ;  
        };

    // this method get vector and return vector as JointGroupPositions in MoveGroup is a vector 
    vector <double>  get_rad_from_deg(vector <double> joints_in_deg){
        cout<<joints_in_deg.size()<<endl;
        for(unsigned int i = 0; i < joints_in_deg.size(); i++){
            double element = joints_in_deg[i];
            element = element * (pi / 180); // convert joint from degrees to radian
            //std::cout << "joint No   in radians  " << i +1 << " _________________"<< element << std::endl;
            rad_arr.push_back(element) ;
            };
        return rad_arr ;  
        };

    };
