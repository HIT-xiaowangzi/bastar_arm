#include "ros/ros.h"
#include <task3/GetPoint.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

boost::shared_ptr<sensor_msgs::PointCloud2 const> batEdge;    
pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
cv_bridge::CvImageConstPtr cvImgPtr;
// service回调函数,输入参数req,输出参数res
//const float baseHeight=0.88;//机器人坐标原点距离地面的距离(gazebo中测定，回实验室需重新验证)
////const float tableHeight=0.80;//桌面高度（比赛时需重新测定）
//const float blockHeight[7]={0.06,0.03,0.06,0.03,0.06,0.015,0.03};//不同类别对应的木块高度
const double R[3][3]={{-0.07036515,-0.75194792,0.65545638},{-0.99678815,0.07819133,-0.01730602},{-0.03823778,-0.65456889,-0.75503473}};//旋转矩阵 由标定得到
const double T[3]={0.158440293708, 0.0201937781102, 0.519211377701};//平移矩阵 由标定得到
const float fx=603.149536;//相机内参
const float fy=602.093201;//相机内参
const float x_0=325.593140;//相机内参
const float y_0=237.004852;//相机内参

geometry_msgs::Pose Callback01(uint16_t x,uint16_t y)
{
   
    ROS_INFO("Enter into PointCloud!");
    geometry_msgs::Pose ans;
    batEdge = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points/",ros::Duration(3));
    if(batEdge != NULL)
    {
        pcl::fromROSMsg(*batEdge, *cloud);
        std::cout<<"cloud width="<<cloud->width<<std::endl;
        std::cout<<"cloud height="<<cloud->height<<std::endl;
    }
    else
        std::cout<<"no topic found!!!"<<std::endl;
        uint32_t index=cloud->width*y+cloud->height;
        double camera_x=(double)(cloud->points[index].x);
        double camera_y=(double)(cloud->points[index].y);
        double camera_z=(double)(cloud->points[index].z);
    if(camera_x>2||camera_y>2||camera_z>2)
    {
        ROS_ERROR("Invalid PointCloud Data from task1_server!");
    }
    double world_x=R[0][0]*camera_x+R[0][1]*camera_y+R[0][2]*camera_z+T[0];
    double world_y=R[1][0]*camera_x+R[1][1]*camera_y+R[1][2]*camera_z+T[1];
    double world_z=R[2][0]*camera_x+R[2][1]*camera_y+R[2][2]*camera_z+T[2];
    ans.position.x=world_x;
    ans.position.y=world_y;
    ans.position.z=world_z;
    ROS_INFO("world_x:[%lf]",world_x);
    ROS_INFO("world_y:[%lf]",world_y);
    ROS_INFO("world_z:[%lf]",world_z);
    ans.orientation.w=0;//方位信息由末端补偿来确定
    ans.orientation.x=0;
    ans.orientation.y=0;
    ans.orientation.z=0;
    return ans;
}
void Callback02(const sensor_msgs::ImageConstPtr imgmsg)
{
    cvImgPtr =cv_bridge::toCvCopy(imgmsg, sensor_msgs::image_encodings::TYPE_16UC1);
//   std::cout << "image data: " << depth_frame.at<uint16_t>(240, 320) << std::endl;//表示获取图像坐标为240,320的深度值,单位是毫米
//   cv::imwrite("testdepth.jpg",depth_frame);
}

geometry_msgs::Pose Callback03(uint16_t x,uint16_t y)
{
    geometry_msgs::Pose ans;
    while(cvImgPtr==NULL)
    {
        ROS_INFO("Wait");
    }
    cv::Mat depth_frame=cvImgPtr->image;
    double x_div_z=(x-x_0)/fx;
    double y_div_z=(y-y_0)/fy;
    double camera_z=(double)depth_frame.at<uint16_t>(y,x)/1000;
    if(camera_z==0)
    {
        int i,j;
        int count=0;
        for(i=-2;i<=2;i++)
        {
            for(j=-2;j<=2;j++)
            {
                if(depth_frame.at<uint16_t>(y+i,x+j)!=0)
                {
                    camera_z+=(double)depth_frame.at<uint16_t>(y+i,x+j)/1000;
                    count++;
                }
            }
        }
        camera_z=camera_z/count;
        if(camera_z==0)
        {
            ROS_ERROR("z=0 Point!");
        }
    }
    double camera_x=camera_z*x_div_z;
    double camera_y=camera_z*y_div_z;
    ROS_INFO("camera_x:[%lf]",camera_x);
    ROS_INFO("camera_y:[%lf]",camera_y);
    ROS_INFO("camera_z:[%lf]",camera_z);
    double world_x=R[0][0]*camera_x+R[0][1]*camera_y+R[0][2]*camera_z+T[0];
    double world_y=R[1][0]*camera_x+R[1][1]*camera_y+R[1][2]*camera_z+T[1];
    double world_z=R[2][0]*camera_x+R[2][1]*camera_y+R[2][2]*camera_z+T[2];
    ans.position.x=world_x;
    ans.position.y=world_y;
    ans.position.z=world_z;
    ROS_INFO("world_x:[%lf]",world_x);
    ROS_INFO("world_y:[%lf]",world_y);
    ROS_INFO("world_z:[%lf]",world_z);
    ans.orientation.w=0;//方位信息由末端补偿来确定
    ans.orientation.x=0;
    ans.orientation.y=0;
    ans.orientation.z=0;
    return ans;
}
bool PointTransform(task3::GetPoint::Request &req,task3::GetPoint::Response &res)
{
    res.Poses.poses.clear();
    int size=req.result.MyLists.size();
    
    if(size>3)
    {
        ROS_INFO("size:[%d]",size);
        ROS_INFO("The service input is too many!");
    }
    else
    {
        if(size<3)
        {
            ROS_INFO("size:[%d]",size);
            ROS_INFO("The service input is too little!");
        }
        for(int i=0;i<size;i++)
        {
            ROS_INFO("Enter For!");
            std::cout<<req.result.MyLists.at(i).x<<std::endl;
            std::cout<<req.result.MyLists.at(i).y<<std::endl;
            geometry_msgs::Pose ans=Callback03(req.result.MyLists[i].x,req.result.MyLists[i].y);
            res.Poses.poses.push_back(ans);
        }
    }
    
    return true;
}
int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "task3_server");
    // 创建节点句柄
    ros::NodeHandle n;
    // 创建一个名为add_two_ints的server,注册回调函数add()
    ros::ServiceServer service = n.advertiseService("task3_service", PointTransform);
    // 循环等待回调函数
    ROS_INFO("task3_Server is ready!");
    //ros::init(argc, argv, "task1_depthframe");
    ros::NodeHandle n1;
    ros::Subscriber listener=n1.subscribe("/camera/aligned_depth_to_color/image_raw",2,Callback02);
    ros::spin();
    return 0;
}
