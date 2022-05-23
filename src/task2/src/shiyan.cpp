#include "ros/ros.h"
#include "baxtercontroller.h"
#include <thread>
#include <future>
#include <math.h>
#include <Eigen/Dense>
#define L_ARM "left"
#define R_ARM "right"
const double x_angle=2;//先绕OX轴转x_angle
const double y_angle=2;//再绕OV’轴转y_angle
const double z_angle=-2;//再绕OW''轴转z_angle
const double PI=3.1415926535897;
//旋转矩阵：Rot(x,x_angle)*Rot(y,y_angle)*Rot(z,z_angle)
class Shiyan
{    
private:
    ros::NodeHandle nh;
    BaxterController* baxter;
public:
    Shiyan(ros::NodeHandle nh)
    {
        this->nh=nh;
        baxter=new BaxterController(nh);//指向机器人的指针，方便调函数
    }
    ~Shiyan()
    {
        delete baxter;
    }
    double** Rotx(double alpha)
    {
        double** ans=new double*[3];
        for(int i=0;i<3;i++)
        {
            ans[i]=new double[3];
        }
        ans[0][0]=1;ans[0][1]=0;ans[0][2]=0;
        ans[1][0]=0;ans[1][1]=cos(alpha);ans[1][2]=-sin(alpha);
        ans[2][0]=0;ans[2][1]=sin(alpha);ans[2][2]=cos(alpha);
        return ans;
    }

    double** Roty(double beta)
    {
        double** ans=new double*[3];
        for(int i=0;i<3;i++)
        {
            ans[i]=new double[3];
        }
        ans[0][0]=cos(beta);ans[0][1]=0;ans[0][2]=sin(beta);
        ans[1][0]=0;ans[1][1]=1;ans[1][2]=0;
        ans[2][0]=-sin(beta);ans[2][1]=0;ans[2][2]=cos(beta);
        return ans;
    }

    double** Rotz(double gamma)
    {
        double** ans=new double*[3];
        for(int i=0;i<3;i++)
        {
            ans[i]=new double[3];
        }
        ans[0][0]=cos(gamma);ans[0][1]=-sin(gamma);ans[0][2]=0;
        ans[1][0]=sin(gamma);ans[1][1]=cos(gamma);ans[1][2]=0;
        ans[2][0]=0;ans[2][1]=0;ans[2][2]=1;
        return ans;
    }
    double** MulMatrix(double** Matrix1,double** Matrix2)
    {
        double** Matrix=new double*[3];
        int i=0,j=0;
        for(i=0;i<3;i++)
        {
            Matrix[i]=new double [3];
        }
        for(i=0;i<3;i++)
        {
            for(j=0;j<3;j++)
            {
                Matrix[i][j]=0;
            }
        }
        for(i=0;i<3;i++)
        {
            for(j=0;j<3;j++)
            {
                for(int k=0;k<3;k++)
                {
                    Matrix[i][j]+=Matrix1[i][k]*Matrix2[k][j];
                }
            }
        }
        return Matrix;
    }
    std::vector<double> getCurPos(std::string armID)
    {
        std::vector<double>currJnt=baxter->getJntPos(armID);//getJntPos:读取当前7个关节的坐标值
        std::vector<double> Cart;
        Cart.resize(6);
        Cart.clear();
        baxter->joint2Cart(currJnt,Cart,armID);//机器人正运动学 Cart返回末端位置
        return Cart;
    }
    double** GetRotMatrix(double alpha,double beta,double gamma)
    {
        double** Rx=Rotx(alpha);
        double** Ry=Roty(beta);
        double** Rz=Rotz(gamma);
        //double temp[3][3]={{0,0,0},{0,0,0},{0,0,0}};
        //double result[3][3]={{0,0,0},{0,0,0},{0,0,0}};
        double** result=MulMatrix(MulMatrix(Rz,Ry),Rx);
        // for(int i=0;i<3;i++)
        // {
        //     for(int j=0;j<3;j++)
        //     {
        //         for(int k=0;k<3;k++)
        //         {
        //             result[i][j]+=temp[i][k]*Rotx[k][j];
        //         }
        //     }
        // }
        // for(int i=0;i<3;i++)
        // {
        //     for(int j=0;j<3;j++)
        //     {
        //         std::cout<<result[i][j]<<" ";
        //     }
        //     std::cout<<std::endl;
        // }
        // std::vector<double> ans;
        // ans.clear();
        // ans.push_back(result[0][2]);
        // ans.push_back(result[1][2]);
        // ans.push_back(result[2][2]);
        // return ans;
        return result;
    }
    void test()
    {
        std::vector<std::vector<double>> path;
        path.clear();//path:路径点的集合 往里面添加要经过的路径点
        std::vector<double> init=getCurPos(R_ARM);//getCurPos:读取当前的末端位置
        path.push_back(init);//加入初始位置
        //std::vector<double> objPos={0.66894,-0.516567,0.259578,3.0775,0.381637,1.49508};
        // std::vector<double> zPos=GetCosine(init[3],init[4],init[5]);//根据RPY欧拉角计算出z轴的方向余弦
        double** RotMatrix=GetRotMatrix(init[3],init[4],init[5]);//3-alpha-x,4-beta-y,5-
        double length=-0.03;//一次移动10cm,可调
        std::vector<double> nextPos=init;//设出目标终点
        nextPos[0]+=length*RotMatrix[2][0];//x方向的步进值
        nextPos[1]+=length*RotMatrix[2][1];//y
        nextPos[2]+=length*RotMatrix[2][2];//z
        //nextPos即为目标终点
        path.push_back(nextPos);//把终点加入集合
        baxter->moveTo(path,0.02,0.01,0.10,0.05,R_ARM);//从起点到终点的路径规划
        //path:路径 0.02,0.01,0.10,0.05:与速度有关 R_ARM:右臂
    }
    void rotation(double x_angle,double y_angle,double z_angle)
    {
        std::vector<std::vector<double>> path;
        path.clear();
        std::vector<double> init=getCurPos(R_ARM);//读取初始的位置
        path.push_back(init);
        ros::Duration(2.0).sleep();
        double** RotMatrix=GetRotMatrix(init[3],init[4],init[5]);//获得当前的旋转矩阵
        double** Final=MulMatrix(MulMatrix(MulMatrix(RotMatrix,Rotx(x_angle*PI/180.0)),Roty(y_angle*PI/180.0)),Rotz(z_angle*PI/180.0));
        //连续右乘获得目标旋转矩阵
        std::cout<<"旋转之前："<<std::endl;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                std::cout<<RotMatrix[i][j]<<" ";//输出转之前的旋转矩阵
            }
            std::cout<<std::endl;
        }
        std::vector<double> temp1=getCurPos(R_ARM);
        std::cout<<"末端坐标："<<std::endl;
        for(auto it=temp1.begin();it!=temp1.end();it++)
        {
            std::cout<<*it<<" ";
        }
        std::cout<<std::endl;
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix<<Final[0][0],Final[0][1],Final[0][2],
        Final[1][0],Final[1][1],Final[1][2],
        Final[2][0],Final[2][1],Final[2][2];//赋值
        Eigen::Vector3d eulerAngle=rotation_matrix.eulerAngles(2,1,0);//由目标旋转矩阵反解出欧拉角，baxter需要欧拉角作为输入参数
        
        std::vector<double>end=init;
        end[3]=eulerAngle[2];//由于一个左乘一个右乘，顺序相反
        end[4]=eulerAngle[1];
        end[5]=eulerAngle[0];
        std::vector<double> TarJnt;
        baxter->cart2Joints(baxter->getJntPos(R_ARM),end,TarJnt,R_ARM);
        baxter->rightMoveOnce(TarJnt);
        // path.push_back(end);
        // baxter->moveTo(path,0.02,0.01,0.07,0.04,R_ARM);//输入新位置
        // ros::Duration(5.0).sleep();
        init=getCurPos(R_ARM);
        double** RotMatrix2=GetRotMatrix(init[3],init[4],init[5]);
        std::cout<<"旋转之后："<<std::endl;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                std::cout<<RotMatrix2[i][j]<<" ";//输出旋转之后的姿态角
            }
            std::cout<<std::endl;
        }
        std::vector<double> temp2=getCurPos(R_ARM);
        std::cout<<"末端坐标："<<std::endl;
        for(auto it=temp2.begin();it!=temp2.end();it++)
        {
            std::cout<<*it<<" ";
        }
        std::cout<<std::endl;
    }
    void duqu()
    {
        std::vector<double> Cart=getCurPos(R_ARM);
        std::cout<<"实际："<<std::endl;
        for(auto it=Cart.begin();it!=Cart.end();it++)
        {
            std::cout<<*it<<" ";
        }
    }
};
int main(int argc,char** argv)
{
    ros::init(argc,argv,"shiyan");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    Shiyan sh(nh);
    // sh.rotation(x_angle,0,0);
    // ros::Duration(2.0).sleep();
    // //sh.rotation(-x_angle,0,0);
    // //ros::Duration(5.0).sleep();
    // sh.rotation(0,y_angle,0);
    // ros::Duration(2.0).sleep();
    // //sh.rotation(0,-y_angle,0);
    // //ros::Duration(5.0).sleep();
    // sh.rotation(0,0,z_angle);
    // ros::Duration(2.0).sleep();
    // //sh.rotation(0,0,-z_angle);
    // //ros::Duration(5.0).sleep();
    sh.test();
    //sh.duqu();
    spinner.stop();
    return 0;
}
