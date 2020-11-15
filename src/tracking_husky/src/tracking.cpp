#include "FOAP.h"
#include "IOEigen.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include "tf/transform_datatypes.h"
#include <algorithm>
#include <vector>
#include <string>
#include <random>
#include <chrono>


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/unsupported/Eigen/KroneckerProduct>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include "gazebo_msgs2/GetNextPoint.h"

using namespace IOeigen;
using namespace Foap;
using namespace std;

void positionCallBack(const nav_msgs::Odometry::ConstPtr& msg);
void distanceCallBack(const sensor_msgs::LaserScan::ConstPtr& msg2);

//Functions
Eigen::MatrixXd vector2matrix(Eigen::VectorXd v, int dim);
Eigen::VectorXd matrix2vector(Eigen::MatrixXd const M);
Eigen::MatrixXd transpose(Eigen::MatrixXd const M);
Eigen::VectorXd getSingleAgent(Eigen::VectorXd const e_q, int n_agents, int current);
double TBGgain(double t0, double tb, double t);



Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
std::string sep = "\n----------------------------------------\n";


// path and gazebo environment input names
std::string path = "/home/bernardo/catkin_hbp_task/src/tracking_husky/data/";
bool status_file;
std::string robot_name = "husky"; // same as mav_name in launchfile
std::string obstacle_name = "obstacle";
std::string relativeEntityName = "world" ;
std::string simulation_filename = path+"sim.dat";
std::string ht_filename = path+"ht.dat";

std::string q_test_filename = path+"q_test.dat";
std::string q_alfa_filename = path+"q_alfa.dat";
std::string vel_filename = path+"vel.dat";
std::string etrack_filename = path+"error_tracking.dat";
std::string yaw_filename = path+"yaw.dat";


// pub & suv variables name
std::string slash("/");
std::string publisher_name = "/husky/cmd_vel";
std::string subscriber_name = "/husky/odom";


// ros msg's 
std::vector<nav_msgs::Odometry> pos_msg;
sensor_msgs::LaserScan dis_msg;
Eigen::MatrixXd orien;

float l2a = 0.1;

// system dimentions
int n_robots = 1, dim = 2, n_models = 2, n_obstacles = 0;


double X,Y,Z;
float Yaw=0, Pitch=0, Roll=0,minDis,direction;

int main(int argc,char** argv){
    std::map<std::string, Eigen::MatrixXd> params;
    Eigen::MatrixXd delta_alfa,alfa;

    
	ros::init(argc,argv,"tracking");
	ros::NodeHandle nh;


	Eigen::MatrixXd q0,qp,qobs,q_ev;
    Eigen::VectorXd qorien,alfa_dot;
    float nearest_dist,nearest_ori;

    q0.resize(dim, n_robots);
    qp.resize(dim,n_robots);
    q_ev.resize(dim,n_robots);
    alfa.resize(dim, n_robots);
    delta_alfa.resize(dim,n_robots);
    orien.resize(n_robots,3);
    qorien.resize(n_robots);
    

    gazebo_msgs::GetModelState getModelState;
    geometry_msgs::Point pp;
    geometry_msgs::Quaternion qq;

    ros::ServiceClient gms_c = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
    ros::ServiceClient gwp_c = nh.serviceClient<gazebo_msgs::GetWorldProperties>("gazebo/get_world_properties");
    ros::ServiceClient npoint = nh.serviceClient<gazebo_msgs2::GetNextPoint>("/get_point");


    gazebo_msgs::GetWorldProperties gwp_s;


    if (gwp_c.call(gwp_s)){
        std::vector<std::string> models = gwp_s.response.model_names;
        n_models = (int)models.size();
        ROS_INFO("Number of models: %i", n_models);
        if(n_models==1){ // only and empty world has been loaded
            ROS_ERROR("An empty world as been loaded");
            return 1;
        }
        // compare if the Adjancency matrix size is equal to the number of robots
        if(n_robots > (n_models-1)){
            ROS_ERROR("Number of robots don't match with the Adjancency matrix");
            return 1;
        }

        // resize qobs by the number of obstacles obtained
        n_obstacles = n_models-n_robots-1;
        if(n_obstacles>0){
            qobs.resize(dim, n_obstacles);
        }


        
        

        //sort the name models to find the robots & obstacles initial positions:
        // models[0] = ground_plane, models[1, n_robots+1] = robots, models[n_robots+2, end] = obstacles
        std::sort(models.begin(), models.end());
        Eigen::VectorXd current_pose(dim); // assume a 2D pose, change dim to add more dimension: e.x: [x, y, z, yaw]
        
        // get robots position
        double roll3, pitch3, yaw3;
        for(int i=0; i<n_robots; i++){
            getModelState.request.model_name = models[n_obstacles+i+1];
            getModelState.request.relative_entity_name = relativeEntityName ;
            gms_c.call(getModelState);
            pp = getModelState.response.pose.position;
            qq = getModelState.response.pose.orientation;
            
            tf::Quaternion qa(qq.x, qq.y, qq.z, qq.w);
			tf::Matrix3x3 mat2(qa);
			mat2.getEulerYPR(yaw3, pitch3, roll3);

			current_pose << l2a*cos(yaw3), l2a*sin(yaw3);
			qorien(i) = yaw3;			
			delta_alfa.col(i)=current_pose;

            current_pose << pp.x, pp.y;
            q0.col(i) = current_pose;

        }
        // get obstacles positions
        if(n_obstacles>0){

            for(int i=0; i<n_obstacles; i++){
                getModelState.request.model_name = models[i];
                getModelState.request.relative_entity_name = relativeEntityName ;
                gms_c.call(getModelState);
                pp = getModelState.response.pose.position;
                
                current_pose << pp.x, pp.y;
                qobs.col(i)=current_pose;
                   
            }
        }

       }
        

        ROS_INFO("Number of robots: %i", n_robots);
        

        std::cout << "Init robot positions: \n" << q0.format(OctaveFmt) << sep;
        std::cout << "Init obstacles positions: \n" << qobs << endl;

        

    /***************************** Pub's & Sub's *********************/
    pos_msg.resize(n_robots);

    int frecuencia=40;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(publisher_name,1);
    ros::Subscriber pos_sub= nh.subscribe<nav_msgs::Odometry>(subscriber_name,1,positionCallBack);
    ros::Subscriber dis_sub= nh.subscribe<sensor_msgs::LaserScan>("/husky/laser/scan",1,distanceCallBack);

	ros::Rate rate(frecuencia);

	Eigen::MatrixXd robot_new_poses;
    Eigen::VectorXd q_test, p_ia, q_c;
	
    Eigen::VectorXd qori = Eigen::VectorXd::Zero(n_robots);
	Eigen::VectorXd error = Eigen::VectorXd::Zero(n_robots*dim);
	Eigen::VectorXd convex_error = Eigen::VectorXd::Zero(n_robots*dim);
    Eigen::MatrixXd I_dim = Eigen::MatrixXd::Identity(dim,dim);
    Eigen::VectorXd sec_error = Eigen::VectorXd::Zero(dim);
    Eigen::MatrixXd q_odom(dim, n_robots);
    double sec_dist=1.5,min_dis_cob;

	alfa=q0 + delta_alfa;
	qori = qorien;
    q_test = matrix2vector(q0);

	Eigen::MatrixXd matA(2,2);
    Eigen::MatrixXd matAt(2,2);

    /************************ Read data parameters *************************/
    Eigen::MatrixXd tmp; // tmp matrix to read parameters

    int updated = 0, steps = 0, count = 0,tray_flag,ev_flag;
    double dt, t = 0, tf, d, k1,k2; 
    Eigen::VectorXd ratios(2);
    Eigen::VectorXd system_params;
    IOEigen::readMatrix(simulation_filename, tmp);
    system_params = matrix2vector(tmp);
    if(system_params.size() < 8){
        std::cout << "Default system simulations params inicialization" << std::endl;
        dt = d = 0.01; tf = 100; k1 = 0.75;
        k2 = 0.75; tray_flag=2; ev_flag=2; ratios << 1.5, 1.5;
    }
    else{
        dt = d = system_params(0); tf = system_params(1); k1 = system_params(2);
        k2 = system_params(3); tray_flag=system_params(4); ev_flag=system_params(5);
        ratios << system_params(6), system_params(7);
        
    }

    std::cout << "Simulation inicialization" << sep;
    double om=120;
    tf=1*om;
    steps = tf*frecuencia;
    dt=d=1.0/frecuencia; 
     
	
	
	double normae=0,norma;
	
	double det,v,w;   
    double *m = new double[2];
    double *m_dot = new double[2];
    Eigen::MatrixXd q_test_data(steps+1,dim*n_robots);
    Eigen::MatrixXd q_alfa_data(steps+1,dim*n_robots);
    Eigen::VectorXd velit(2*n_robots+1);
    Eigen::MatrixXd vel_data(steps+1,dim*n_robots+1);
    Eigen::MatrixXd e_tracking_data(steps+1,2*dim+1);
    Eigen::VectorXd e_tracking(2*dim+1);
    Eigen::MatrixXd cons_data(steps+1,dim*n_robots);
    Eigen::MatrixXd yaw_angle_data(steps+1,n_robots);
    double begin_time,current_time,nk=0,t0_tbg=0;
    double k1t=0.1,k2t=0.1;


    /*************************** Start simulation ***********************/
    Eigen::MatrixXd velocities=vector2matrix(Eigen::VectorXd::Zero(n_robots*dim),dim);
    Eigen::VectorXd ht_params;
    IOEigen::readMatrix(ht_filename, tmp);
    ht_params = FOAP::matrix2vector(tmp);
    TTF ht;
    ht.init_ht(n_robots, dim, ht_params);

    std::cout << "Transition function inicialization" << sep;
    
    gazebo_msgs2::GetNextPoint gnp;
    bool av=true;
    double tf_tbg=30;

	while(ros::ok()){
        
        if(count==0){
            begin_time=ros::Time::now().toSec();
            cout<<"Tiempo inicial: "<<begin_time<<endl;
        }
        
        current_time=ros::Time::now().toSec()-begin_time;
		
        //end simulation condition
        if(current_time>tf)
		{
			geometry_msgs::Twist msg;	
            msg.linear.x=0;
            msg.linear.y=0;
            msg.angular.z=0;
            vel_pub.publish(msg);
            break;
		}

        // hold for updates
        if(updated < 2){
            rate.sleep();
            updated+=1;
            continue;
        }
        
					
		ros::spinOnce();
        for(int i=0;i<n_robots;i++)
        {

            q_test(i)=X;
            q_test(i+n_robots)=Y;
            delta_alfa(0,i)=l2a*cos(orien(i,2));
            delta_alfa(1,i)=l2a*sin(orien(i,2));
            qori(i)=orien(i,2);
            q_ev(i)=minDis*cos(direction)*cos(qori(i)-M_PI/2)-minDis*sin(direction)*sin(qori(i)-M_PI/2)+X;
            q_ev(i+n_robots)=minDis*sin(direction)*sin(qori(i)-M_PI/2)+minDis*cos(direction)*cos(qori(i)-M_PI/2)+Y;
            nearest_ori=direction;
            nearest_dist=minDis;

                
        }
        alfa=vector2matrix(q_test,dim) + delta_alfa;
            
        

                  

		//get next point
        if(av||current_time-t0_tbg>tf_tbg){
            if(av==false)
            {
                cout<<"Point reached: "<<X<<" "<<Y<<endl;
            }
            do{
                npoint.call(gnp);
                min_dis_cob=1e10;
                for(int i=0;i<n_obstacles;i++)
                {
                    sec_error(0)=gnp.response.next_point.x-qobs(0,i);
                    sec_error(1)=gnp.response.next_point.y-qobs(1,i);
                    if(sec_error.norm()<min_dis_cob)
                        min_dis_cob=sec_error.norm();

                }
            }while(min_dis_cob<sec_dist);
            
            m[0]=gnp.response.next_point.x;
            m[1]=gnp.response.next_point.y;
            m_dot[0]=0;
            m_dot[1]=0; 
            cout<<"Next_goal: "<<m[0]<<" "<<m[1]<<endl;
            t0_tbg=ros::Time::now().toSec()-begin_time;
            
            av=false;    
        }
     
		
        
                               
        // get j-robot
        for (int j = 0; j < n_robots; j++){
            
            // get j-decoupling matrix
            matA(0,0)=cos(qori(j));
            matA(0,1)=-l2a*sin(qori(j));
            matA(1,0)=sin(qori(j));
            matA(1,1)=l2a*cos(qori(j));

            
            //position error
            e_tracking<<current_time,(alfa(0,0)-m[0]),(alfa(1,0)-m[1]),m[0],m[1];


            error(j)=((alfa(0,0)-m[0]));
            error(j+n_robots)=((alfa(1,0)-m[1]));
            norma=sqrt(error(j)*error(j)+error(j+n_robots)*error(j+n_robots));
            normae+=sqrt(error(j)*error(j)+error(j+n_robots)*error(j+n_robots));
               
            k1t=k2t=TBGgain(t0_tbg,t0_tbg+tf_tbg,current_time); 
            error(j)=(-k1t*(error(j)))+m_dot[0];
            error(j+n_robots)=(-k2t*(error(j+n_robots)))+m_dot[1];


            p_ia = getSingleAgent(error,n_robots,j);
            
            // obstacle avoidance
            convex_error = FOAP::convexCombination(alfa, qobs, ratios, p_ia, I_dim, ht, current_time, d, dt, j);
            //convex_error = FOAP::obstacleAvoidance(alfa, q_ev, ratios, p_ia, I_dim, ht, current_time, d, dt, j,nearest_dist);
            
            


            det=matA(0,0)*matA(1,1)-matA(0,1)*matA(1,0);
           
            w=-1*matA(1,0)*1/det*convex_error(0)+matA(0,0)*1/det*convex_error(1);
           
            v=matA(1,1)*1/det*convex_error(0)-1*matA(0,1)*1/det*convex_error(1);
            
           
           // update control law for each robot
            velocities(0,j)=v;
            velocities(1,j)=w;

            
            
            
            

        }
        
        //saving data per iteration
        q_test_data.row(count) = q_test;
        q_alfa_data.row(count) = matrix2vector(alfa);
        yaw_angle_data.row(count)= qori;
        velit<<current_time,velocities(0,0),velocities(1,0);
        e_tracking_data.row(count)=e_tracking;
        vel_data.row(count) = velit;
        q_test_data.row(count) = matrix2vector(q_test);
        q_alfa_data.row(count) = matrix2vector(alfa);
            
        // set MultiDOF msg
        for(int i=0; i<n_robots; i++){
            geometry_msgs::Twist msg;
            msg.linear.x=velocities(0,i);
            msg.linear.y=0;
            msg.linear.z=0;
            msg.angular.x=0;
            msg.angular.y=0;
            msg.angular.z=velocities(1,i);


            if(norma<=0.5){
                cout<<"Goal reached!"<<endl;
                msg.linear.x=0;
                msg.linear.y=0;
                msg.angular.z=0;
                av=true;
            }
                
            vel_pub.publish(msg);
           
            
        }
            
        
           

		//verify if it's over
		count++;
        
		rate.sleep(); 
			
	}	
	
    cout<<"ECM: "<<normae/steps<<endl;
	IOEigen::writeMatrix(q_test_filename, q_test_data);
    IOEigen::writeMatrix(q_alfa_filename, q_alfa_data);
    IOEigen::writeMatrix(vel_filename, vel_data);
    IOEigen::writeMatrix(etrack_filename, e_tracking_data);
    IOEigen::writeMatrix(yaw_filename,yaw_angle_data);
    delete [] m;
    delete [] m_dot; 
    

return 0;
}


void positionCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
	X = msg->pose.pose.position.x;
    Y = msg->pose.pose.position.y;
    Z = msg->pose.pose.position.z;


        tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        //creating rotation matrix from quaternion
        tf::Matrix3x3 mat(q);
        //obtaining euler angles
        double roll, pitch, yaw;
        mat.getEulerYPR(yaw, pitch, roll);
        //saving the data obtained
        orien(0) = (float) roll; orien(1) = (float) pitch; orien(2) = (float)yaw; 

}

void distanceCallBack(const sensor_msgs::LaserScan::ConstPtr& msg2)
{ 
    
    int minIdx=std::min_element(msg2->ranges.begin(),msg2->ranges.end()) - msg2->ranges.begin();
    
    minDis = *min_element(msg2->ranges.begin(),msg2->ranges.end());
    
    direction = float(M_PI*float(minIdx)/360);    
    
    
    

}


double TBGgain(double t0, double tb, double t)
{
    double d=0.01,Xi,Xip;
    if (t < t0){
        Xi = 0;
        Xip = 0;}
    else if ((t <= tb) && (t >= t0)){
        Xi = 10*( (pow(t - t0,3))/(pow(tb - t0,3)) ) - 15*( (pow(t - t0,4))/(pow(tb - t0,4)) ) + 6*( (pow(t - t0,5))/(pow(tb - t0,5)) );    
        Xip = 30*( (pow(t - t0,2))/(pow(tb - t0,3)) ) - 60*( (pow(t - t0,3))/(pow(tb - t0,4)) ) + 30*( (pow(t - t0,4))/(pow(tb - t0,5)) );}
    else if (t > tb){
        Xi = 1;   
        Xip = 0;}


    return (Xip/(1-Xi+d));
}

Eigen::VectorXd getSingleAgent(Eigen::VectorXd const e_q, int n_agents, int current){
    int dim = e_q.size()/n_agents;
    Eigen::VectorXd e(dim);
    for(int i=0; i<dim; i++){
        e(i) = e_q(i*n_agents + current);
    }

    return e;
}


Eigen::MatrixXd transpose(Eigen::MatrixXd const M){
    // M is a scalar
    if(M.size() == 1)
        return M;

    return M.transpose().eval();    
}


/*
Parser for matrix to vector
M = 
1 2
3 4
v = [1, 2, 3, 4]
*/
Eigen::VectorXd matrix2vector(Eigen::MatrixXd const M){
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> M2(M);
    Eigen::Map<Eigen::VectorXd> v(M2.data(), M2.size());

    return v;
}

/*
Parser for vector to matrix
v = [1, 2, 3, 4]
M =
1 2
3 4
*/
Eigen::MatrixXd vector2matrix(Eigen::VectorXd v, int dim){
    int n_robots = v.size()/dim;
    Eigen::Map<Eigen::MatrixXd> MT(v.data(),n_robots, dim);
    Eigen::MatrixXd M =  MT.transpose().eval();

    return M;
}



