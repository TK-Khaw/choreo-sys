#include <ros/ros.h>
#include <math.h>
#include <sbotik.h>
#include <vector>
#include <std_msgs/Float64.h>

#define LOOP_RATE 50
#define CON_GAIN 10
#define TWOPI 6.283185307
#define PI 3.141592654

class conSys
{
public:
	conSys(ros::NodeHandle n,const std::string& name, sbotIK* bot); //TODO: Figure a way to allow initialization of states.
	~conSys();

	enum ConMode{
		SINGLE,	//Single integrator node dynamics. 
		DOUBLE, //Double integrator node dynamics.
		MODSINGLE //Modified single integrator control law. NOT IMPLEMENTED DO NOT USE.
	};
	

private:	
	void stateUpdateCallback(const std_msgs::Float64::ConstPtr& msg, const std::string& topicName); 
	void update(const ros::TimerEvent& e);	
	void errorGravitate(float& fbuf);
	float modFactor(const float& ferr);
	
	sbotIK* m_pBot; //Pointer to the IK controller of the robot. need for update.  

	float m_fCurState[2]; //Without offset. Offset at publish time.
	float m_fOff; // offset of the robot. in radian.  
	std::vector<float*> m_vecfNeighborStates; //state vector from neighboring agent, in virtual space. 
	
	std_msgs::Float64 m_msgBuf;	
	ros::Timer m_timerUpdate;
	ros::Publisher m_pubState;
	std::vector<ros::Subscriber> m_vecSubNeighbors;

	std::vector<float> 	m_vecfConGains; //Consensus Control Law Gains. Pointer to array storing the configured values.  
	int 	m_iLoopRate; //Loop Rate for the consensus loop.

	ConMode m_conMode;
};
