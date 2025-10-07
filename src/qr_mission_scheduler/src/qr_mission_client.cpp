#include "qr_mission_client.h"



void  load_conveyor_location(std::vector<std::string> &conveyor_location)
	{
		std::ifstream input("conveyor_data.txt");
		ROS_INFO("Reading data");
	if (input.is_open())
	{
		std::string data;
		while(std::getline(input, data))
			{ 
				conveyor_location.push_back(data);
			}
	  //std::cout<<data_vector[1]<<"\n";
	}
	
	input.close();
	
	}


void load_data(std::vector<std::string> &data_vector)
	{
	
	std::ifstream input("/home/nvidia/catkin_ws/rack_data.txt");
	ROS_INFO("Reading data");
	if (input.is_open())
	{
		std::string data;
		while(std::getline(input, data))
			{ 
				data_vector.push_back(data);
			}
	  //std::cout<<data_vector[1]<<"\n";
	}
	input.close();
	}
	
	
void spinThread()
{
  ros::spin();
}


void split_data(std::vector<std::string> &data_vector,std::vector<std::vector<std::string>> &pose_data_)
	{
		
		char space_delimiter = ' ';
		std::vector<std::string> split_data{};
		//pose_data.clear();
		
		for (auto& it : data_vector)
		{
			split_data.clear();
			std::string word;
			std::stringstream sstream(it);
			std::cout<<it<<std::endl;
			while (std::getline(sstream, word, space_delimiter))
			{
			word.erase(std::remove_if(word.begin(), word.end(), ispunct), word.end());
				split_data.push_back(word);
			}
			pose_data_.push_back(split_data);	
		}
		
	
	}

void call_server(Client &client,const std::string &goal_type,int &source_,int &destination)
	{
		//source = source_;
		ROS_ERROR("Source --> %d", source);
	    ros::Rate rate(5);
     if(source!=destination)
	 {
	    std_msgs::Bool locationChange;
        locationChange.data=1;
	    robotLocationChangePub.publish(locationChange);
                std_msgs::Bool break_data;     
                break_data.data = false;
                enableBreakPub.publish(break_data);


	    qr_mission_scheduler::mission_schedulerGoal	 goal;

	    goal.goalType.goalType =goal_type;
	    goal.goalType.source = source;
	    goal.goalType.destination = destination;
	    client.sendGoal(goal);
	    client.waitForResult(ros::Duration(5.0));
	if (source == destination)
	{
	                ROS_INFO("Same --> destination and source %d , destination %d", source,destination);

		std_msgs::Bool locationChange;
		locationChange.data=0;
		robotLocationChangePub.publish(locationChange);
		source = destination;

		m_reached_goal = true;
         std_msgs::Bool break_data;     
         break_data.data = true;
         enableBreakPub.publish(break_data);

	}


	while (client.getState() !=actionlib::SimpleClientGoalState::SUCCEEDED)
	{	
		ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
		if(client.getState().toString() == "ABORTED" || client.getState().toString() == "PREEMPTED" )
			break;

		if(!m_is_charging_requested && m_is_charge_initiated)
		{
			ROS_WARN_STREAM("Cancelling charge operation that was previously initiated");
			client.cancelAllGoals();
			m_is_charge_initiated = false;
		}

		if(!ros::ok()) break;

		rate.sleep();
	}
		if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			printf("Success --> reached destination");
			ROS_INFO("Success reached destination,source %d , destination %d", source,destination);
			source = destination;
			m_reached_goal = true;
            		std_msgs::Bool break_data;     
            		break_data.data = true;
            		enableBreakPub.publish(break_data);


			if(m_is_charging_requested)
			{
				m_is_charge_initiated = false;
				m_is_charging_requested = false;
			}
		}
        }
	if (source == destination)
        {
		ROS_INFO("Same  destination and source %d , destination %d", source,destination);

		std_msgs::Bool locationChange;
                locationChange.data=0;
		robotLocationChangePub.publish(locationChange);
                source = destination;
                std_msgs::Bool break_data;
                break_data.data = true; 
                enableBreakPub.publish(break_data);
                m_reached_goal = true;

        }

}

void go_for_charging(Client &client)

{
	ROS_ERROR_STREAM("Going for charging with source and destination ID "<<source <<" -- "<<m_charge_destination);
	m_lift_action_data.reached = false;
        lift_action_pub.publish(m_lift_action_data);
	
	call_server(client, "CHARGING", source, m_charge_destination);
	//m_is_charge_initiated = false;
	m_is_charging_requested = false ; 
}

bool charging_request_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	if(m_picked_count != 0 && req.data)
	{
		ROS_ERROR_STREAM("Charging request rejected as pick count is "<<m_picked_count);
		m_is_charging_requested = false;

		res.success = false;
		res.message = "Charging request rejected as pick count is not zero";

		return true;
	}

	m_is_charging_requested = req.data;
	ROS_INFO_STREAM("Charging request state: "<<std::boolalpha<<m_is_charging_requested);

	res.success = true;
	res.message = "Charging request accepted";

	return true;
}




void batteryCBData(const sensor_msgs::BatteryState& msg)
{

    m_battery_percentage = msg.percentage * 100;
    usleep(1000);
    if(m_battery_percentage < 44 && m_battery_percentage > 0 && m_picked_count ==0) 
        {
            ROS_INFO("Battery low calling for charging ");
           // m_is_charging_requested = true;
            m_is_battery_low = true;
        
        }
}




void wms_drop_data_split(const wms_data::WMSTasks& msg)
{
		//ROS_INFO("dropping wms_data_msg %ld ",msg.tasks.size());
        //m_picked_count = msg.tasks.size();	
		for(int i{0};i<msg.tasks.size();i++)
			{
				if(dropping_location_data.size() == 0 && msg.tasks[i].unique_id != dropping_last_task_unique_id)
				{
					for(int k{0};k<msg.tasks.size();k++)
					{
						dropping_location_data.push_back(msg.tasks[k]);

					}
				}
				
				
				else 
				{
			/*		int m_match_count{0};
					for (int j{0}; j<dropping_location_data.size();j++)
					{
						//if(msg.tasks[i].unique_id != dropping_location_data[j].unique_id && msg.tasks[i].unique_id != dropping_last_task_unique_id)
						if(msg.tasks[i].unique_id == dropping_location_data[j].unique_id)
						{
							///ROS_INFO("Matching id found");
							break;
				
						}
						if(msg.tasks[i].unique_id == dropping_last_task_unique_id)
						{
							bream_maximum_pick_size;
						}
						else 
						{
							//break;
							m_match_count +=1;
						//	ROS_INFO(" No Matching id found %d, picking_location_data %ld ",m_match_count,picking_location_data.size());

						}
					}	
					
				if (m_match_count == dropping_location_data.size())
					{
						dropping_location_data.push_back(msg.tasks[i]);
					}
					*/
	//		}
			
			
			ROS_INFO("size of dropping location data %ld ",dropping_location_data.size());
			}
		}
	
		//m_bin_number = msg.tasks[0].bin;
		//m_rack_number = msg.tasks[0].rack;
	
}

void lifter_feedback_reset()
{
        //               m_lifter_confirmation = false;
           //               m_lifter_id = "";
            //            m_failstatus = false; //if fail status is true, the picking / dropping is failed, dont count that

}



void wms_pick_data_split(const wms_data::WMSTasks& msg)
	{
		ROS_INFO("wms_data_msg %ld ",msg.tasks.size());
        if(m_is_battery_low == false && msg.tasks.size()>0)
        {	
		for(int i{0};i<msg.tasks.size();i++)
			{
				if(picking_location_data.size() == 0 && msg.tasks[i].unique_id != last_task_unique_id)
				{
				     	lifter_feedback_reset(); 
					for(int k{0};k<msg.tasks.size();k++)
					{
						picking_location_data.push_back(msg.tasks[k]);

					}
				}
			//	else 
				//	{
				//		ROS_INFO("Already %ld data available for pick ",picking_location_data.size());
				//	}
				
			}
            
			/*	
				else 
				{
					int m_match_count{0};
					for (int j{0}; j<picking_location_data.size();j++)
					{
						//if(msg.tasks[i].unique_id != picking_location_data[j].unique_id && msg.tasks[i].unique_id != last_task_unique_id)
						if(msg.tasks[i].unique_id == picking_location_data[j].unique_id)
						{
							ROS_INFO("Matching id found");
							break;
				
						}
					else 
						{
							//break;
							m_match_count +=1;
							ROS_INFO(" No Matching id found %d, picking_location_data %ld ",m_match_count,picking_location_data.size());

						}
					}	
					
				if (m_match_count == picking_location_data.size())
					{
						picking_location_data.push_back(msg.tasks[i]);
					}
			}
			 */
			
			ROS_INFO("size of picking location data %ld ",picking_location_data.size());

	//	}
	
		//m_bin_number = msg.tasks[0].bin;
		//m_rack_number = msg.tasks[0].rack;
	}
}
void lifter_feedback(const lift_action::LiftFeedback& msg) 
	{
			m_lifter_confirmation = msg.status;
			m_lifter_id = msg.unique_id;
			m_failstatus = msg.failStatus; //if fail status is true, the picking / dropping is failed, dont count that 
	
			ROS_INFO("m_lifter_confirmation %d : ,m_lifter_id : %s  ",m_lifter_confirmation,m_lifter_id.c_str());
	}
/*
void lifter_feedback_reset()
{
	//		 m_lifter_confirmation = false;
         		  m_lifter_id = "";
            //            m_failstatus = false; //if fail status is true, the picking / dropping is failed, dont count that 

}
*/
void rack_to_conveyor(Client &client,bool action)
{
					ROS_INFO("INSIDE RACK TO CONVEYOR");	
					m_bin_number = int (picking_location_data[0].bin)-100  ;
					//if(m_rack_number>1000)
					//{
						m_rack_number = picking_location_data[0].rack - 1000;
						m_rack_number = int(m_rack_number/10);
					//}
					if(m_rack_number < 0)
					{
						ROS_ERROR("rack number is wrong %d ",picking_location_data[0].rack , m_rack_number);
						
					}
					m_id = picking_location_data[0].unique_id;
					ROS_WARN("Id--> %s, M_rack_num %d, M_bin_number %d",m_id.c_str(),m_rack_number,m_bin_number);
					destination = std::stof(pose_data[m_rack_number][m_bin_number]);
					std::cout <<source<<" : " <<destination<<" : "<<pose_data[0][2] <<std::endl;
					m_lift_action_data.unique_id = m_id;
					m_lift_action_data.reached = m_reached_goal;
					m_lift_action_data.shelf =picking_location_data[0].shelf;
					if(action ==true)
					{
					m_lift_action_data.action = true;
					}
					if (action==false)
					{
					 m_lift_action_data.action = false;

					}
					m_lift_action_data.rack = picking_location_data[0].rack;
					m_lift_action_data.acr_shelf = picking_location_data[0].acr_shelf;
					m_lift_action_data.whole_bin_no= picking_location_data[0].whole_bin_no;
					m_lift_action_data.putaway_barcode = picking_location_data[0].putaway_code;
					//m_lift_action_data.level = picking_location_data[0].level;
					//m_lift_action_data.year = picking_location_data[0].year;
					//m_lift_action_data.date_data = picking_location_data[0].date_data;
					//m_lift_action_data.serial_no = picking_location_data[0].serial_no;
					ROS_INFO("1 --> lift action reached goal status %d ",  m_lift_action_data.reached);
					lift_action_pub.publish(m_lift_action_data);
					call_server(client,goal_type,source,destination);
					//lifter_feedback_reset();
					 ROS_INFO("2 --> lift action reached goal status %d ",  m_lift_action_data.reached);

					lift_action_pub.publish(m_lift_action_data);
					while(m_lifter_confirmation != true && m_id == m_lifter_id)
					
					{
					 m_lift_action_data.reached = m_reached_goal;
					lift_action_pub.publish(m_lift_action_data);
						ROS_WARN("Liftaction in process");
						ros::Duration(1.0).sleep();
					}					
					ROS_INFO("2 --> m_lifter_confirmation %d : ,m_lifter_id : %s  ",m_lifter_confirmation,m_lifter_id.c_str());
					
					ROS_INFO("m_lifter_confirmation %d : ,m_id : %s  ",m_lifter_confirmation,m_id.c_str());
					if(m_lifter_confirmation == true && m_id == m_lifter_id)
						{
							ROS_INFO("Lifting done, confirmation recived, removing the picking location %s --> %s  ",m_id,m_lifter_id);
							try 
							{
								if(picking_location_data.size()>0)
									{
									last_task_unique_id =m_id;
									picking_location_data.erase(picking_location_data.begin());
									ROS_INFO("picking_location_data_size after erasing %ld",picking_location_data.size());
									}
								throw 505;
							}
							catch (...) 
							{
								ROS_INFO(" something wrong with picking location data");

							}
							m_reached_goal = false;
							 //here we are checking 2 main checks, 
							 //1 we are checking whether the pick was successufl if not we are skipping 
							if(action==true && m_failstatus ==false)
							{
								ROS_WARN("increasing m_picked count %d ",m_picked_count);
								m_picked_count +=1;
								ROS_WARN("Done : %d",m_picked_count);

							}
							// here we are checkign the drop was successful, else we skip the count
							if(action==false && reset_flag_==true && m_failstatus ==false)
							{
								m_picked_count -=1;
								reset_flag_=false;

							}
						}
					else
							{
								reset_flag_=true;
							}
					ROS_WARN("Picked_count-->  %d ",m_picked_count);	
					ROS_WARN("DONE RACK TO CONVEYOR");
}

void drop_in_conveyor(Client &client,bool action)
	{

		ROS_INFO("INSIDE   DROP IN CONVEYOR");

            //for(int i { 0 }; i < conveyor_pose_data[0].size(); i++)
			//for(int i { 0 }; i < dropping_location_data.size(); i++) 

		//	.{
				if(dropping_location_data.size()>0)
					{
					m_bin_number =dropping_location_data[0].bin -100;
					m_rack_number = dropping_location_data[0].rack-1000;
					m_rack_number = int(m_rack_number/10);
					if(m_rack_number<0)
					{
						ROS_ERROR("rack number is wrong %d ",dropping_location_data[0].rack , m_rack_number);
						
					}
					m_id = dropping_location_data[0].unique_id;
					
					ROS_INFO("Id--> %s, M_rack_num %d, M_bin_number %d",m_id.c_str(),m_rack_number,m_bin_number);
					//destination = std::stof(conveyor_pose_data[0][m_bin_number]);
					destination = std::stof(pose_data[m_rack_number][m_bin_number]);
					if(destination != source)
					{
						m_reached_goal = false;
					}
					std::cout <<source<<" : " <<destination<<" : "<<pose_data[0][2] <<std::endl;
					m_lift_action_data.unique_id = m_id;
					m_lift_action_data.reached = m_reached_goal;
					m_lift_action_data.shelf =dropping_location_data[0].shelf;
					if (action ==true)
						{
					m_lift_action_data.action = false;
						}
					if(action ==false)
					{
					  m_lift_action_data.action = false;

					}
					
					m_lift_action_data.rack = dropping_location_data[0].rack;
					m_lift_action_data.acr_shelf = dropping_location_data[0].acr_shelf;
                                        m_lift_action_data.whole_bin_no= dropping_location_data[0].whole_bin_no;
                                        m_lift_action_data.putaway_barcode = dropping_location_data[0].putaway_code;
					//m_lift_action_data.bin= dropping_location_data[0].bin;
					//m_lift_action_data.core = dropping_location_data[0].core;
					//m_lift_action_data.level = dropping_location_data[0].level;
					//m_lift_action_data.year = dropping_location_data[0].year;
					//m_lift_action_data.date_data = dropping_location_data[0].date_data;
					//m_lift_action_data.serial_no = dropping_location_data[0].serial_no;
					//lifter_feedback_reset();
					 ROS_INFO("3 --> lift action reached goal status %d ",  m_lift_action_data.reached);

					lift_action_pub.publish(m_lift_action_data);
					call_server(client,goal_type,source,destination);
				
				
				/*
				m_id = m_dropping_id + std::to_string(i);
				m_lift_action_data.unique_id = m_id;
				m_lift_action_data.reached = m_reached_goal;
				m_lift_action_data.shelf = i;
				m_lift_action_data.action = action;
				m_lift_action_data.rack = i;
*/
				lift_action_pub.publish(m_lift_action_data);
/*				//destination = droping_poses[m_dropping_pos_count];
				destination = std::stof(conveyor_pose_data[0][m_dropping_pos_count]);
				
				call_server(client, goal_type, source, destination);
				lift_action_pub.publish(m_lift_action_data);
				 */
				ROS_INFO("Dropping id %s , recived_droppping_id : %s ", m_id.c_str(), m_dropping_id.c_str());
				
				while(m_reached_goal == true) 
				{

					 m_lift_action_data.reached  = true;
//					 lifter_feedback_reset();
					 //ROS_INFO("4 --> lift action reached goal status %d ",  m_lift_action_data.reached);

					 lift_action_pub.publish(m_lift_action_data);

					ROS_DEBUG_ONCE("lifer_confirmation %d , m_id: %s , m_dropping_id : %s ", m_lifter_confirmation, m_id.c_str(), m_lifter_id.c_str());
					if(m_lifter_confirmation == true && m_id == m_lifter_id) 
					{
						try 
							{
								if(dropping_location_data.size()>0)
									{
									dropping_last_task_unique_id =m_id;
									dropping_location_data.erase(dropping_location_data.begin());
									ROS_INFO("dropping_location_data_size after erasing %ld",dropping_location_data.size());
									}
								throw 505;
							}
							catch (...) 
							{
								ROS_INFO(" something wrong with dropping location data");
							}
						
						
						ROS_INFO("dropped %d ", m_dropping_pos_count);
						m_dropping_pos_count += 1;
						if(action == true && m_failstatus == false)
						{
						m_picked_count -= 1;
                          		//	m_picked_count = dropping_location_data.size();
						
						}
						m_reached_goal = false;
						if(action==false && m_failstatus==false)  
						{
							ROS_WARN("Inside Drop conveyor picked_count increase %d",m_picked_count);
						//	m_picked_count += -3;
							ROS_WARN("Picked_count, inside Drop in conveyor %d ",m_picked_count);

						}
						
					}
				}
	        //	ROS_INFO("Lifting done, confirmation recived, removing the picking location");
            }
			else
				{
					ROS_INFO("size of dropping location --> %ld",dropping_location_data.size());
				}
			ROS_INFO("DONE DROP IN CONVEYOR");
	}
	
           // m_dropping_pos_count = 0;
		//	m_picked_count = 0;

     	//}



int main (int argc, char** argv)
{

	ros::init(argc, argv, "qr_mission_client");
	ros::NodeHandle nh;

	wms_pick_data_sub = nh.subscribe("/wms_pick_tasks", 1,wms_pick_data_split);
	wms_drop_data_sub = nh.subscribe("/wms_drop_tasks", 1,wms_drop_data_split);
     battery_sub = nh.subscribe("/battery_data",1,batteryCBData);

	lift_feedback_sub = nh.subscribe("lifterFeedback",1,lifter_feedback);
	lift_action_pub = nh.advertise<lift_action::LiftAction>("lift_action", 1);
	robotLocationChangePub=nh.advertise<std_msgs::Bool>("/robot_location_change",1);
	charging_srvr = nh.advertiseService("initiate_charging", &charging_request_cb);


    enableBreakPub  = nh.advertise<std_msgs::Bool>("/brake_activator",1);
	nh.getParam("/mission_client/max_bins",m_maximum_pick_size);
	nh.getParam("/mission_client/starting_vertx",source);
	nh.getParam("/mission_client/picked_count",m_picked_count);
	nh.getParam("/mission_client/charge_destination", m_charge_destination);

	Client client("mission_server",true);
	boost::thread spin_thread(&spinThread);

	load_data(data_vector);
	
	ROS_INFO("done reading data");

	split_data(data_vector,pose_data);
	
	ROS_INFO("done spliting data");
	load_conveyor_location(conveyor_data);
	split_data(conveyor_data,conveyor_pose_data);

	client.waitForServer();
	ros::Rate rate(1);
	
	while (ros::ok())
		{
	
				ROS_INFO("remaining picking location %ld",picking_location_data.size());
				ROS_ERROR("source %d",source);	
				if(picking_location_data.size()>0)
					{
						std::cout<<"first picking data :"<<picking_location_data[0].unique_id<<std::endl;
					}
				ROS_WARN("Picked_count %d ",m_picked_count);
				if(m_picked_count ==0)
				{
					dropped_flag =true;
				}
				if(picking_location_data.size() != 0 && m_picked_count < m_maximum_pick_size && dropped_flag == true )
				{
					if(picking_location_data[0].action == false)
					{
						m_dropping_id ="Picking"; // Here we are picking the boxes from conveyor 
						drop_in_conveyor(client,false);
						m_picked_from_conveyor = true;
						ROS_WARN("Picked_count %d ",m_picked_count);
						
					}
					 
					if(picking_location_data[0].action == true)
					{
						m_dropping_id = "Dropping";
						rack_to_conveyor(client,true);
						m_picked_from_conveyor = false;
					}
					 
				}
				
				else 
					{
						if(m_picked_count > 0 && m_picked_from_conveyor == false)
						{
							m_dropping_id="Dropping";
							drop_in_conveyor(client,true);
							dropped_flag = false;
						}
						 
						if(m_picked_count > 0 && m_picked_from_conveyor == true )
						{
							m_dropping_id = "Dropping";
							rack_to_conveyor(client,false);
						}
					}

				if(m_is_charging_requested)
				{
					m_is_charge_initiated = true;
					go_for_charging(client);
				}

				rate.sleep();

		
		}

	return 0;
}
