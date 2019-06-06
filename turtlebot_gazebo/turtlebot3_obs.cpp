#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
using namespace ros;

Subscriber lidar_sub;
Publisher cmd_pub;

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

int main(int argc, char **argv){ 
    init(argc,argv, "lidar_obs");
    NodeHandle n;
    Rate rate (1);

    cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    // Subscribe to a given topic, in this case "chatter".
	//chatterCallback: is the name of the callback function that will be executed each time a message is received.
    lidar_sub = n.subscribe("scan", 1000, lidar_callback);
    
    // Enter a loop, pumping callback

    rate.sleep();
    ros::spin();
    
    return 0;
}



