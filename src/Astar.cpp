
#include "Astar.h"
#define POT_HIGH 1000000
using namespace std;
vector<queue> queue_;
void add(const cv::Mat &cos,float* potential,float prev_potential,int next_x,int next_y, int end_x,
         int end_y);
bool Astar(const cv::Mat &, const pose &, const pose &,int,float*);
int main(){
    depth = cv::imread( "./data/a.pgm",0);
    //cout<<depth.rows<<"  111  "<<depth.cols;

    cv::Mat_<uchar>::iterator it;
    cv::MatConstIterator_<uchar> it_in=depth.begin<uchar>();
    cv::MatConstIterator_<uchar> itend_in=depth.end<uchar>();
    while(it_in != itend_in)
    {
        if(*it_in.ptr > 100){
            *it_in.ptr = 254;

        }
        else
            *it_in.ptr = 0;
        it_in++;
    }
    pose start_pose{66,120};
    pose end_pose{178,101};
    float *potential;
    potential = new float[depth.rows*depth.cols];
    if(Astar(depth,start_pose,end_pose,400000,potential) == true){
        if(getPath(start_pose,end_pose,depth,potential)==true)
            cout<<"OK"<<endl;
    }
    cv::imshow( "keypoints", depth );
    cv::imwrite( "./data/b.pgm", depth );
    cv::waitKey(0); //暂停等待一个按键
    delete potential;

    return 0;
}
bool Astar(const cv::Mat &cost_map,const pose &start_pose,const pose &end_pose,int cycles,float *potential){
    queue_.clear();
    if(cost_map.ptr(start_pose.y)[start_pose.x]<100||cost_map.ptr(end_pose.y)[end_pose.x]<100)
        return false;
    nx_ = cost_map.cols;
    std::fill(potential, potential + cycles, POT_HIGH);
    int cycle = 0;

    queue_.push_back(queue(start_pose,0));
    potential[start_pose.y*nx_+start_pose.x] = 0;
    while (queue_.size() > 0 && cycle < cycles*2) {
        queue top = queue_[0];
        //cout<<top.pose_.x<<"  "<<top.pose_.y<<" "<<top.cost<<endl;

        depth.ptr(top.pose_.y)[top.pose_.x] = 220;
        static int x = 0;
        if(x == 0){
            cv::imshow( "keypoints", depth );
            cv::waitKey(1); //暂停等待一个按键
            x = 1;
        }else x=0;
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = (top.pose_.y) * nx_ + top.pose_.x;

        if(top.pose_.x == end_pose.x&&top.pose_.y == end_pose.y){
            cout<<"goal"<<endl;
            return true;
        }
        add(cost_map, potential, potential[i], top.pose_.x + 1,top.pose_.y, end_pose.x, end_pose.y);
        add(cost_map, potential, potential[i], top.pose_.x - 1,top.pose_.y, end_pose.x, end_pose.y);
        add(cost_map, potential, potential[i], top.pose_.x ,top.pose_.y + 1, end_pose.x, end_pose.y);
        add(cost_map, potential, potential[i], top.pose_.x ,top.pose_.y - 1, end_pose.x, end_pose.y);
        //cout<<queue_.size()<<endl;
    }
}

void add(const cv::Mat &cos, float* potential, float prev_potential, int next_x, int next_y, int end_x,
         int end_y)
{
    int next_i = (next_y) * nx_ + next_x;
    if (potential[next_i] < POT_HIGH)
        return;
    if(cos.ptr(next_y)[next_x]<100)
        return;


    float distance = abs(end_x - next_x) + abs(end_y - next_y);

    potential[next_i] = calculatePotential(potential,next_i);

    queue_.push_back(queue(next_x,next_y, potential[next_i] + distance * 1));
    std::push_heap(queue_.begin(), queue_.end(), greater1());


}
