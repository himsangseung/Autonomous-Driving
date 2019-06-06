#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
using namespace ros;

//Subscriber lidar_sub;
Subscriber nav_sub;
Publisher cmd_pub;

void nav_callback(const nav_msgs::Odometry::ConstPtr& msg){
	float x, y, z;
    float disp;
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	z = msg->pose.pose.position.z;

	//ROS_INFO("Position: x = %f, y = %f, z = %f",x,y,z);
    disp = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    ROS_INFO("Distance =%f", disp);
	geometry_msgs::Twist twist;

	//twist.linear.x = 0.1;
	//twist.angular.z = 0;
    //cmd_pub.publish(twist);   
    

    if (disp > 0.2){
	    cmd_pub.publish(twist);
        ROS_INFO("The distance from the origin exceeded 0.2 m");
    }


}
/*
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg ){
    float scan_measure;
    scan_measure = msg->ranges[0];
    ROS_INFO ("New Scan Received, distance= %f", scan_measure);


    
    if (scan_measure < 0.5){
        geometry_msgs::Twist twist;
        twist.linear.x =0;
        twist.angular.z =0; 
        cmd_pub.publish(twist);
        
    }
    
    
    
    else{
        geometry_msgs::Twist twist;
        twist.linear.x =0.1;
        twist.angular.z =0;
        cmd_pub.publish(twist);
    }
    


}
*/
int main(int argc, char **argv){ 
    init(argc,argv, "nav_nodes");
    NodeHandle n;
    Rate rate (1);

    cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    // Subscribe to a given topic, in this case "chatter".
	//chatterCallback: is the name of the callback function that will be executed each time a message is received.
    //lidar_sub = n.subscribe("scan", 1000, lidar_callback);
    nav_sub = n.subscribe("/odom",10, nav_callback );
    // Enter a loop, pumping callback

    rate.sleep();
    ros::spin();
    
    return 0;
}



