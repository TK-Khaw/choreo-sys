#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <consys.h>
#include <sstream>

int main(int argc, char **argv){
	if(argc != 2){
		ROS_ERROR("Arguments Input ERROR.");
		return 1;
	}

	ros::init(argc, argv, "simCart");
	ros::NodeHandle n;

	//AsyncSpinner creation. 
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	//Initialize IK class.	
	sbotIK sbotik(n);

	//Initialize conSys class.
	conSys cS(n, argv[1], &sbotik); //Let default the initial state first.  	
	
	ros::waitForShutdown();
	return 0;
}
