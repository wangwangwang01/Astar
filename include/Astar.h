#include <iostream>
#include<vector>
#include<unistd.h>
#include<stdlib.h>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
int nx_;
cv::Mat depth;
struct pose{
    int x;
    int y;
};
class queue{
    public:
        queue(pose pose_in,float cost_){
            pose_ = pose_in;
            cost = cost_;
        }
        queue(int x_p,int y_p,float cost_){
            pose_.x = x_p;
            pose_.y = y_p;
            cost = cost_;
        }
        pose pose_;
        float cost = 100000;
};
struct greater1 {
        bool operator()(const queue& a, const queue& b) const {
            return a.cost > b.cost;
        }
};
float calculatePotential(float* potential, int n){
        // get min of neighbors
        float min_h = std::min( potential[n - 1], potential[n + 1] ),
              min_v = std::min( potential[n - nx_], potential[n + nx_]);
        int prev_potential = std::min(min_h, min_v);
    return prev_potential + 1;
}
float getPoseCost(int x,int y,float *potential){
    //std::cout<<y*nx_ + x<<std::endl;
    return potential[y*nx_ + x];
}
bool getPath(pose start_p, pose end_p,cv::Mat &cos,float *potential){
    int c = 0;
    int x,y,next_x,next_y;
    x = end_p.x;
    y = end_p.y;

    float cost = 0;
    while (c++<4000000) {
        float minCost = 10000000;
        for(int x_axis = -1;x_axis < 2;x_axis++)
            for(int y_axis = -1;y_axis < 2;y_axis++){
                //std::cout<<"x:"<<x+x_axis<<"   y:"<<y+y_axis<<std::endl;
                if(x_axis == 0&&y_axis == 0)
                    continue;
                if((cost = getPoseCost(x+x_axis,y+y_axis,potential))<minCost){
                    minCost = cost;
                    next_x = x+x_axis;
                    next_y = y+y_axis;
                }
            }
        //std::cout<<minCost<<std::endl;
        cos.ptr(next_y)[next_x] = 100;

        y = next_y;
        x = next_x;
        if(start_p.x  == x&&start_p.y == y)
            return true;
        //cv::waitKey(1000);
    }
}
