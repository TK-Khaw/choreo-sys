#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>

#include <math.h>
#include <sensor_msgs/JointState.h>

//Class for implementation of inverse kinematic of sbot. 
class sbotIK 
{
public:
	sbotIK(ros::NodeHandle n);
	~sbotIK();

	bool computeAndPublish();
	float fPhi; //Parametrized angle value. 	
	
private:
	//IK computation and publication methods. 
	bool compute();
	bool publish();
	void cartesianCompute(float& fxp, float& fyp);
	void ikCompute(const float& fxp, const float& fyp);
	float constraint(float x);

	ros::Publisher m_pubComJ1; //Joint Position Command Publisher for Joint 1.
	ros::Publisher m_pubComJ2; //Joint Position Command Publisher for Joint 2.
	
	float m_fJ1;    //Angle for Joint 1.
	float m_fJ2;    //Anlge for Joint 2.
	float m_fL1;    //Length of Link1
	float m_fL2;    //Length of Link2
	float m_fR;     //Radius of disk.
	float m_fa;     //Degree of inclination in Radian. Rosparam stores as degree. Constructor convert to radian. 
	float m_fCa2;   //Some repeatedly used constant.
	float m_fSa;    //Some repeatedly used constant.
	float m_fl1ml2; //Some repeatedly used constant.
	float m_fx0;    //x offset from the center of the disk.
	float m_fy0;    //y offset from the cetner of the disk.		
	
};
