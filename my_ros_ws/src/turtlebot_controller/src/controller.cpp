#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"
#include "my_srv/Velocity.h"

//This is the position x, y, theta of the robot
double my_x, my_y, my_theta;


		
//This callback function takes the position of the robot from the simulator
void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{

	my_x = msg->x;
	my_y = msg->y;
	my_theta = msg -> theta+0.0001;	
//	ROS_INFO("Pos@[%f, %f, %f]", my_x, my_y, my_theta);
}

//This function is used to get a random velocity
void get_random_vel(ros::NodeHandle nh)
{
	ros::ServiceClient client4 = nh.serviceClient<my_srv::Velocity>("/service_node/velocity");
	my_srv::Velocity rec_vel;
	rec_vel.request.min=0.0;
	rec_vel.request.max=1.0;
	client4.call(rec_vel);
	ROS_INFO("Random [%f, %f]", rec_vel.response.x, rec_vel.response.z);
}

//This function is used to refresh the screen
void reset_screen(ros::NodeHandle nh)
{
	ros::ServiceClient client0 = nh.serviceClient<std_srvs::Empty>("/reset");
	std_srvs::Empty srv0;
	client0.call(srv0);
}

//This function is used to teleport the robot somewhere in the screen
void init_robot(ros::NodeHandle nh, double x, double y, double theta)
{
	ros::ServiceClient client1 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
	turtlesim::TeleportAbsolute srv1;
	srv1.request.x =x;
	srv1.request.y = y;
	srv1.request.theta = theta + 0.000001;
	client1.call(srv1);
}

//This function is used to clear the trail
void clear_screen(ros::NodeHandle nh)
{
	ros::ServiceClient client0 = nh.serviceClient<std_srvs::Empty>("/clear");
	std_srvs::Empty srv0;
	client0.call(srv0);
}

void control_method_1(geometry_msgs::Twist &my_vel)
{

}



void control_method_2(geometry_msgs::Twist &my_vel, double a, double b, double c)
{
	double K1 = 1.0;
	double K2 = 15.0;
	double d = (a*my_x + b*my_y + c)/sqrt(a*a+b*b);
	double u = 1.0;
	double r= -K1*u*d*sin(my_theta)/my_theta - K2*fabs(u)*my_theta;
//	ROS_INFO("Distance@[%f %f]", d, my_theta);
	my_vel.linear.x = u;
	my_vel.angular.z = r;
}
	

void control_method_3(geometry_msgs::Twist &my_vel, double goal_x, double goal_y, double ox, double oy)
{
	double K1 = 5.0;
	double K2 = 5.0;
	
	//Compute a vector heading to the goal
	double gx = goal_x - my_x;
	double gy = goal_y - my_y;
	//Compute the norm of this vector
	double dg = sqrt(gx*gx+gy*gy);
	//Compute the attractive force to the goal
	double fgx = gx/dg;
	double fgy = gy/dg;
	
	//Chose the maximum distance for detecting obstacles
	double rho_0 = 4;
	//Compute a vector heading from the obstacle to the robot
	double dox = my_x - ox;
	double doy = my_y - oy;
	//Compute the norm of this vector
	double rho = sqrt(dox*dox+doy*doy);
	//Compute the notm of this vector
	double ux = dox / rho;
	double uy = doy / rho;
	double fox;
	double foy;
	
	//Compute the repulsive force exerted by the obstacle
	
	if (rho >= rho_0)
	{
		fox=0;
		foy=0;
	}
	else
	{
		// This is computed using the APF formula
		fox = K1*(1/(rho) - 1/(rho_0))*ux/(rho*rho);
		foy = K1*(1/(rho) - 1/(rho_0))*uy/(rho*rho);		
	}
	//ROS_INFO("Obstacle Distance@[%f %f %f]", rho, fox, fox);
	//Compute the velocity vector heading to the goal
	double vx = fgx + fox;
	double vy = fgy + foy;
	
	
	/*
	//Compute the velocity vector heading to the goal
	double vx = fgx;
	double vy = fgy;
	*/
	
	//Compute the direction of the vector
	double vdir = atan2(vy, vx);
	//Compute the angular error compared with the current heading
	double angledif = vdir - my_theta;
	//Keep the value within the range -pi + pi
	if (angledif < -M_PI) angledif+=2*M_PI;
	if (angledif > M_PI) angledif-=2*M_PI;

	//If the robot has not reached the goal, compute u and r 
	double u, r;
	if (dg < 1.0)
	{
		u=0;
		r=0;
	}
	else
	{
		u=1.0;
		r=K2*angledif;
	}

	my_vel.linear.x = u;
	my_vel.angular.z = r;
}	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlebot_controller");
	ros::NodeHandle nh;
	get_random_vel(nh);
	reset_screen(nh);
	init_robot(nh, 1, 1, 0);
	clear_screen(nh);
	

	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1, turtleCallback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	
	ros::Rate loop_rate(10);
	
	while(ros::ok())
	{
		ros::spinOnce();
		
		geometry_msgs::Twist my_vel;
		
		//Straigth line that describes the path through 
		//the implicit equation ax+by+c=0
		/*double a = -1;
		double b = 1;
		double c = 0;
		control_method_2(my_vel, a,b,c);*/
		double gx = 9.0;
		double gy = 9.0;
		control_method_3(my_vel, gx, gy, 6, 5);
		pub.publish(my_vel);
		loop_rate.sleep();
	}
	return 0;
}
