#include <sbotik.h>
#include <sstream>

sbotIK::sbotIK(ros::NodeHandle n):
	fPhi(0),
	m_fJ1(1.047198),
	m_fJ2(-0.523599)
{
	//Advertise services. 
	m_pubComJ1 = n.advertise<std_msgs::Float64>("sbot/controller/position/Joint1/command", 1000);
	m_pubComJ2 = n.advertise<std_msgs::Float64>("sbot/controller/position/Joint2/command", 1000);
	
	bool bErrFlag = true;

	//Get common parameter from ros parameter server.
	bErrFlag &= n.getParam("/disk_radius", m_fR);
	bErrFlag &= n.getParam("/inclination_angle", m_fa);
	bErrFlag &= n.getParam("/x_offset", m_fx0);
	bErrFlag &= n.getParam("/y_offset", m_fy0);
	
	//Get link length from robot_description. 
	urdf::Model sbotModel;
	urdf::LinkSharedPtr bufLink;
	
	if(!sbotModel.initParamWithNodeHandle("robot_description", n)){ ROS_ERROR("CAN'T LOAD ROBOT MODEL IN SBOTIK CLASS"); }
	
	//No error handling at this phase. Assumed robot_description is always correct. Might be TODO later.
	//Assumed the link is box. If changed assumption, need to change the cast here. 
	sbotModel.getLink("Link1", bufLink);
	m_fL1 = urdf::static_pointer_cast<urdf::Box>(bufLink->visual->geometry)->dim.x;
	sbotModel.getLink("Link2", bufLink);
	m_fL2 = urdf::static_pointer_cast<urdf::Box>(bufLink->visual->geometry)->dim.x;
	
	m_fa = 0.01745329*m_fa; //R2D CONVERSION.
	m_fCa2 = pow(cos(m_fa), 2);
	m_fSa =  sin(m_fa);
	m_fl1ml2 = m_fL1*m_fL1 - m_fL2*m_fL2; //Some frequently used constant. 

	
	ROS_INFO("VALUE FOR LINK1 %f", m_fL1);
	ROS_INFO("VALUE FOR LINK2 %f", m_fL2);
    ROS_INFO("disk_radius %f", m_fR);	
    ROS_INFO("l1ml2 %f", m_fl1ml2);	
    ROS_INFO("ca2 %f", m_fCa2);	
    ROS_INFO("sa  %f", m_fSa);	

}

sbotIK::~sbotIK(){
}

//Reserved bool return value for error reporting in future. TODO implement error halting mechanism.
bool sbotIK::computeAndPublish(){
	compute();
	publish();

	return true;
}

//Essence of computation over here. 
bool sbotIK::compute(){
	//First compute the desired cartesian. 
	float fxp, fyp;

	cartesianCompute(fxp, fyp);
	ikCompute(fxp, fyp);

	return true;
}

void sbotIK::cartesianCompute(float& fxp, float& fyp){
	fxp = m_fx0 - m_fR*sqrt(pow(cos(fPhi),2) + pow(sin(fPhi),2)*m_fCa2);
	fyp = m_fy0 - m_fR*m_fSa*sin(fPhi);
}

void sbotIK::ikCompute(const float& fxp, const float& fyp){
	float ssxy = fxp*fxp + fyp*fyp;

	m_fJ1 = atan2(fyp, fxp) + acos(constraint((ssxy + m_fl1ml2)/(2*m_fL1*sqrt(ssxy))));  //this is more than enough no need to consider other value specifically for this problem. acos always returns positive. That is what we want. 
	
	m_fJ2 = atan2(( fyp - m_fL1*sin(m_fJ1) ),( fxp - m_fL1*cos(m_fJ1) )) - m_fJ1;		
}

//for constraining domain to be inserted into arccosine. 
float sbotIK::constraint(float x){
	if(x > 1){ x = 1;}
	if(x < -1){ x = -1;}

	return x;
}

bool sbotIK::publish(){
	std_msgs::Float64 msg;
	
	//Publish to Joint1.	
	msg.data = m_fJ1;
	m_pubComJ1.publish(msg);

	//Publish to Joint2.
	msg.data = m_fJ2;
	m_pubComJ2.publish(msg);
	
	return true;
}

