#include <consys.h>

conSys::conSys(ros::NodeHandle n, const std::string& name, sbotIK* bot):
	m_fCurState({0,0}),
	m_iLoopRate(50),
	m_pBot(bot)
{
	ROS_ERROR("LOADED %s conSys!!", name.c_str());

	//First, obtain all necessary parameters from parameter server.
	std::vector<std::string> vecNeighborNames;
	if(!n.getParam("/network/"+name+"/neighbors", vecNeighborNames)){
		ROS_ERROR("Failed to retrieve communication topology. ");
	}
	if(!n.getParam("/network/"+name+"/offset", m_fOff)){
		ROS_ERROR("Failed to obtain offset value.");
	}
	
	bool bErrFlag = true;
	int iConMode = 0;
	bErrFlag &= n.getParam("consensus/con_mode", iConMode);
	if( iConMode >=0 && iConMode <=2){
		m_conMode = static_cast<ConMode>(iConMode);
		
		//initialization decision.
		switch(m_conMode){
			case SINGLE:
				m_vecfConGains.resize(1,0);
				break;
			case DOUBLE:
				m_vecfConGains.resize(2,0);
				break;
			case MODSINGLE:
				m_vecfConGains.resize(1,0); //Placeholder. Dunno yet.
				break;
		}

		bErrFlag &= n.getParam("consensus/gains", m_vecfConGains);

			
	}else{
		bErrFlag = false;
	}

	bErrFlag &= n.getParam("consensus/loop_hz", m_iLoopRate);
		
	if(!bErrFlag){ ROS_ERROR("Error procuring loop rate and gain for consensus control."); }

	
	//Convert Degree to Radian. Config file is in degree.
	m_fOff *= 0.01745329252;
	
	//Clear and resize vector storing neighbor state. 	
	m_vecfNeighborStates.clear();
	m_vecfNeighborStates.resize(vecNeighborNames.size(), 0);
	for(int i=0; i < m_vecfNeighborStates.size(); i++){
		m_vecfNeighborStates[i] = new float[2]; //init array into the pointer.
		m_vecfNeighborStates[i][0] = 0;
		m_vecfNeighborStates[i][1] = 0; //initialize with 0.
	}

	//Initialize subscribers to neighbor state. 
	m_vecSubNeighbors.clear();
	
	std::string strBuf;
	for(int i = 0; i< vecNeighborNames.size(); i++){
		strBuf = "/"+vecNeighborNames[i]+"/phi_state";
		m_vecSubNeighbors.push_back( n.subscribe<std_msgs::Float64>(strBuf,1, boost::bind(&conSys::stateUpdateCallback,this, _1, strBuf)));
	}
		
	//Initialize and publish initial state of this node. Note resource name is relative for publisher invocation. 	
	m_pubState = n.advertise<std_msgs::Float64>("phi_state", 100);
	m_msgBuf.data = 0; //think about additional stuff later. 
	m_pubState.publish(m_msgBuf);

	//Create Timer for to run concensus model. 	
	m_timerUpdate = n.createTimer(ros::Duration(1.0/m_iLoopRate), &conSys::update, this);

}

conSys::~conSys()
{
	for(int i=0; i < m_vecfNeighborStates.size(); i++){
		if(!m_vecfNeighborStates[i]){
			delete m_vecfNeighborStates[i];
		}
	}
}

//Update callback for setting neighboring state value upon update. 
void conSys::stateUpdateCallback(const std_msgs::Float64::ConstPtr& msg, const std::string& topicName)
{
	//Handle update to new value for neighbor state.
	for(int i=0; i<m_vecSubNeighbors.size(); i++){
		if(topicName == m_vecSubNeighbors[i].getTopic()){
			m_vecfNeighborStates[i][1] = m_vecfNeighborStates[i][0]; //Shift the storage back on the buffer of two.
			m_vecfNeighborStates[i][0] = msg->data; //fmod(msg->data, TWOPI); //precautionary measure. data mod 2pi.
			break;
		}
	}

}


//Update callback for state per iteration. 
void conSys::update(const ros::TimerEvent& e)
{


	//Basically doing the control law computation. 	

	switch(m_conMode){
		case SINGLE:
		{
			float fErr = 0;
			float fbuf; //buffer for computation. 

			for(int i=0; i<m_vecfNeighborStates.size(); i++){
				fbuf = m_vecfNeighborStates[i][0] - m_fCurState[0];
				errorGravitate(fbuf);
				fErr += fbuf;
			}
		
			m_fCurState[0] += (m_vecfConGains[0]/(1.0f * m_iLoopRate))*fErr;
			errorGravitate(m_fCurState[0]); //m_fCurState[0] = fmod(m_fCurState[0] - PI , PI) + PI; //To preserve in ring space.
			break;
		}
		case DOUBLE:
		{
			float fErr = 0;
			float fNext = m_fCurState[0] - m_fCurState[1];
			float fbuf;

			float fDelayGain = m_vecfConGains[1]/(1.0f * m_iLoopRate);
			float fOnGain = 1/(1.0f * m_iLoopRate * m_iLoopRate) + fDelayGain;

			//errorGravitate(fNext);
			fNext += m_fCurState[0];

			for(int i=0; i<m_vecfNeighborStates.size(); i++){
				fbuf = m_vecfNeighborStates[i][0] - m_fCurState[0];
				//errorGravitate(fbuf);
				fErr += fOnGain*fbuf;

				fbuf = m_vecfNeighborStates[i][1] - m_fCurState[1];
				//errorGravitate(fbuf); 
				fErr += fDelayGain*fbuf;
			}

			fNext += m_vecfConGains[0]*fErr; 

			//Shift register after all are done. 
			m_fCurState[1] = m_fCurState[0];
			m_fCurState[0] = fNext;
			break;
		}
		case MODSINGLE:
		{
			float fErr = 0;
			float fbuf; //buffer for computation. 

			for(int i=0; i<m_vecfNeighborStates.size(); i++){
				fbuf = m_vecfNeighborStates[i][0] - m_fCurState[0];
				errorGravitate(fbuf);
				fErr += fbuf;
			}
		
			m_fCurState[0] += (m_vecfConGains[0]/(1.0f * m_iLoopRate))*modFactor(fErr)*fErr;
			errorGravitate(m_fCurState[0]);//m_fCurState[0] = fmod(m_fCurState[0], TWOPI); //To preserve in ring space.fmod dont neglect sign.
			break;
		}
	}
 
	//Publishing, similar for all methods. 	
	//Update it to IK controller. 
	m_pBot->fPhi = m_fCurState[0] + m_fOff;
	m_pBot->computeAndPublish();
	
	//And publish it.
	m_msgBuf.data = m_fCurState[0];
	m_pubState.publish(m_msgBuf);

}

//Make sure the error between neighboring states is always within range of -pi to pi.
void conSys::errorGravitate(float& fbuf){
	if(fbuf < -PI){
		fbuf += TWOPI;
	}
	if(fbuf > PI){
		fbuf -= TWOPI;
	}
}


//Logistic distribution inspired modification. 
float conSys::modFactor(const float& ferr){
	float fExpo = exp((m_vecfConGains[2]*log(99) - fabs(ferr))/m_vecfConGains[2]);
	return 1 + m_vecfConGains[1]*fExpo/(m_vecfConGains[2]*(pow((1+fExpo), 2))); //corrected equation. 
}