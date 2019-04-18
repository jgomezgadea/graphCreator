/** \file robotnik_fms_graph_node.cpp
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2016
 *
 * \brief This node manages the graph route of the graph robot
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <robotnik_fms_routes/Graph.h>
#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <robotnik_msgs/State.h>
#include <robotnik_fms_msgs/State.h>
#include <robotnik_fms_msgs/GetRoute.h>
#include <robotnik_fms_msgs/ReloadGraph.h>
#include <robotnik_fms_msgs/GetNodeInfo.h>
#include <robotnik_fms_msgs/BlockNode.h>
#include <robotnik_fms_msgs/GetBlockedNode.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include "robotnik_fms_msgs/RobotStatus.h"
#include "robotnik_fms_msgs/NodesInfo.h"
#include "robotnik_fms_msgs/NodeInfo.h"
#include "robotnik_fms_msgs/NodesID.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <robotnik_msgs/alarmmonitor.h>
#include <robotnik_msgs/alarmsmonitor.h>

#include <tf/transform_datatypes.h>

#include <std_msgs/Float32.h>

#include <robot_local_control_msgs/RobotStatus.h>

#include <tf/transform_listener.h>

#include <fstream>

//! Size of string for logging
#define DEFAULT_THREAD_DESIRED_HZ 40.0

using namespace std;

//! Class Rcomponent
class GraphNode
{
  protected:
    //! Controls if has been initialized succesfully
    bool initialized, ros_initialized;
    //! Controls the execution of the GraphNode's thread
    bool running;

    //! State of the GraphNode
    int state;
    //! State before
    int previous_state;
    //!	Saves the name of the component
    string component_name;
    //! ROS node handle
    ros::NodeHandle nh_;
    //! Private ROS node handle
    ros::NodeHandle pnh_;
    //! Desired loop frequency
    double desired_freq_, real_freq;
    //! Path of the xml graph
    string graph_file_;

    //! Publish the component state
    ros::Publisher state_publisher_;

    ros::Publisher graph_scale_publisher_;

    ros::ServiceServer route_service_server_;         // service server
    ros::ServiceServer reload_graph_service_server_;  // service server
    ros::ServiceServer get_node_info_service_server_; // service server

    ros::ServiceServer block_node_service_server_;         // service server
    ros::ServiceServer reserve_node_service_server_;       // service server
    ros::ServiceServer check_blocked_node_service_server_; // service server

    //! General status diagnostic updater
    diagnostic_updater::Updater *diagnostic_;

    ros::Timer timerPublish;

    Graph *graph_route;
    double dGraphFreq_;

    //! Mutex to exclude concurrent access to alarm vectors
    pthread_mutex_t mutexGraph;

  public:
    //! Public constructor
    GraphNode(ros::NodeHandle h);
    //! Public destructor
    ~GraphNode();

    //! Starts the control loop of the component and its subcomponents
    //! @return OK
    //! @return ERROR starting the thread
    //! @return RUNNING if it's already running
    //! @return NOT_INITIALIZED if it's not initialized
    virtual int start();
    //! Stops the main control loop of the component and its subcomponents
    //! @return OK
    //! @return ERROR if any error has been produced
    //! @return NOT_RUNNING if the main thread isn't running
    virtual int stop();
    //! Returns the general state of the GraphNode
    int getState();
    //! Returns the general state of the GraphNode as string
    char *getStateString();
    //! Returns the general state as string
    char *getStateString(int state);
    //! Method to get current update rate of the thread
    //! @return pthread_hz
    double getUpdateRate();

  protected:
    //! Configures and initializes the component
    //! @return OK
    //! @return INITIALIZED if the component is already intialized
    //! @return ERROR
    int setup();
    //! Closes and frees the reserved resources
    //! @return OK
    //! @return ERROR if fails when closes the devices
    //! @return RUNNING if the component is running
    //! @return NOT_INITIALIZED if the component is not initialized
    int shutdown();
    //! All core component functionality is contained in this thread.
    //!	All of the GraphNode component state machine code can be found here.
    void controlLoop();
    //! Actions performed on initial state
    void initState();
    //! Actions performed on standby state
    void standbyState();
    //! Actions performed on ready state
    void readyState();
    //! Actions performed on the emergency state
    void emergencyState();
    //! Actions performed on Failure state
    void failureState();
    //! Actions performed on Shudown state
    void shutdownState();
    //! Actions performed in all states
    void allState();
    //! Switches between states
    void switchToState(int new_state);
    //! Setups all the ROS' stuff
    int rosSetup();
    //! Shutdowns all the ROS' stuff
    int rosShutdown();
    //! Reads data a publish several info into different topics
    void rosPublish();
    //! Reads params from params server
    void rosReadParams();

    //! Get Nodes Used
    bool getNodesUsed(std::vector<Node *> *route);

    //! True if it is posible to block/free node idNode by Robot iRobot
    bool checkNode(bool bBlock, int idNode, int iRobot, string *msg);

    //! Idem CheckNode but blocking/unblocking node
    bool blockNode(bool bBlock, int idNode, int iRobot, string *msg);

    //! Idem CheckNode but blocking/unblocking node
    bool reserveNode(bool bReserve, int idNode, int iRobot, string *msg);

    void copyNodeToMessage(Node *Or, robotnik_fms_msgs::NodeInfo *Dest);
    void copyMessageToNode(robotnik_fms_msgs::NodeInfo *Or, Node *Dest);

    // Examples
    // void topicCallback(const std_msgs::StringConstPtr& message); // Callback for a subscriptor
    bool routeServiceServerCb(robotnik_fms_msgs::GetRoute::Request &request, robotnik_fms_msgs::GetRoute::Response &response);                      // Callback for a service server
    bool reloadGraphServiceServerCb(robotnik_fms_msgs::ReloadGraph::Request &request, robotnik_fms_msgs::ReloadGraph::Response &response);          // Callback for reloading the Graph
    bool getNodeInfoServiceServerCb(robotnik_fms_msgs::GetNodeInfo::Request &request, robotnik_fms_msgs::GetNodeInfo::Response &response);          // Callback for getting the node info
    bool blockNodeServiceServerCb(robotnik_fms_msgs::BlockNode::Request &request, robotnik_fms_msgs::BlockNode::Response &response);                // Callback for getting the node info
    bool getBlockedNodeServiceServerCb(robotnik_fms_msgs::GetBlockedNode::Request &request, robotnik_fms_msgs::GetBlockedNode::Response &response); // Callback for getting the node info

    void status_subscriberCB(robotnik_fms_msgs::RobotStatus msg);

    void timerPublishCallback(const ros::TimerEvent &);

    // Status Publisher
    ros::Publisher NodeUsedPublisher;
    ros::Publisher NodeIDUsedPublisher;
    ros::Publisher GraphPublisher;

    std::vector<ros::Subscriber> vRobotStatusSubs;
    std::vector<robotnik_fms_msgs::RobotStatus> vRobotStatus;
    std::vector<std::string> vRobotStatusTopicName;

    int iMaxRobots;

    ros::Publisher markers_pub_nodes_used;
    ros::Publisher markers_pub_graph;
    ros::Publisher markers_pub_robots;

    //! Diagnostic updater callback
    void diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void push_al(robotnik_msgs::alarmsmonitor *Alarms, std::string type, std::string group, int line, const char *calledby, std::string text, bool bPrint, bool bHMI);
    void pop_old_alarms(robotnik_msgs::alarmsmonitor *Alarms);
    ros::Timer timerAlarms;
    void timerPublishAlarms(const ros::TimerEvent &event);
    ros::Publisher alarm_pub_;
    robotnik_msgs::alarmsmonitor Alarms;
    int iDisplay_Alarm_Monitor_;
    bool bDemo_;
    std::vector<int> vLastNodeDemoMode;

    double fGraph_markers_scale;

    std::string graph_frame = "";
};

#define PUSH_ALARM(text) push_al(&Alarms, "Alarm", component_name, __LINE__, __FUNCTION__, text, true, false)
#define PUSH_EVENT(text) push_al(&Alarms, "Event", component_name, __LINE__, __FUNCTION__, text, true, false)
#define PUSH_ERROR(text) push_al(&Alarms, "Error", component_name, __LINE__, __FUNCTION__, text, true, false)

#define PUSH_ALARM_NOPRINT(text) push_al(&Alarms, "Alarm", component_name, __LINE__, __FUNCTION__, text, false, false)
#define PUSH_EVENT_NOPRINT(text) push_al(&Alarms, "Event", component_name, __LINE__, __FUNCTION__, text, false, false)
#define PUSH_ERROR_NOPRINT(text) push_al(&Alarms, "Error", component_name, __LINE__, __FUNCTION__, text, false, false)

#define PUSH_ALARM_HMI(text) push_al(&Alarms, "Alarm", component_name, __LINE__, "", text, true, true)
#define PUSH_EVENT_HMI(text) push_al(&Alarms, "Event", component_name, __LINE__, "", text, true, true)
#define PUSH_ERROR_HMI(text) push_al(&Alarms, "Error", component_name, __LINE__, "", text, true, true)

void GraphNode::push_al(robotnik_msgs::alarmsmonitor *Alarms, std::string type, std::string group, int line, const char *calledby, std::string text, bool bPrint, bool bHMI)
{
    bool bNotFound = true;
    //int iFound=0;
    std::string txt;
    std::string grp = group;

    if (strlen(calledby) > 0)
        grp = grp + "::" + std::string(calledby);
    if (line > 0)
        grp = grp + "::" + std::to_string(line);

    if (bHMI)
    {
        txt = /*component_name+"::"+*/ text;
    }
    else
    {
        txt = component_name + "::" + std::string(calledby) + "::" + std::to_string(line) + "::" + text;
    }
    for (int i = 0; i < Alarms->alarms.size(); i++)
    {
        robotnik_msgs::alarmmonitor alarm = Alarms->alarms[i];
        if (alarm.group == grp)
        {
            if (alarm.type == type)
            {
                if (alarm.text == txt)
                {
                    if (alarm.hmi == bHMI)
                    {
                        alarm.text = txt;
                        //Using seconds active calculate time without refresh
                        //If seconds active > seconds_to_delete -> delete alarm
                        alarm.seconds_active = 0;
                        bNotFound = false;
                    }
                }
            }
        }
    }
    if (bNotFound)
    {
        robotnik_msgs::alarmmonitor alarm;
        alarm.group = grp;
        alarm.type = type;
        alarm.text = txt;
        alarm.hmi = bHMI;
        alarm.component = component_name;
        alarm.display = iDisplay_Alarm_Monitor_;

        if (bPrint)
        {
            if (type == "Alarm")
            {
                if (alarm.hmi)
                {
                    alarm.display = 10;
                    ROS_WARN("%s", txt.c_str());
                    //ROS_INFO("HMI::TEXT:%s,GROUP:%s,TYPE:%s, Display:%d",txt.c_str(),grp.c_str(),type.c_str(),alarm.display);
                }
                else
                {
                    ROS_WARN("%s", txt.c_str());
                }
            }
            if (type == "Event")
            {
                if (alarm.hmi)
                {
                    alarm.display = 12;
                    ROS_INFO("%s", txt.c_str());
                    //ROS_INFO("HMI::TEXT:%s,GROUP:%s,TYPE:%s, Display:%d",txt.c_str(),grp.c_str(),type.c_str(),alarm.display);
                }
                else
                {
                    ROS_INFO("%s", txt.c_str());
                }
            }
            if (type == "Error")
            {
                if (alarm.hmi)
                {
                    alarm.display = 11;
                    ROS_ERROR("%s", txt.c_str());
                    //ROS_INFO("HMI::TEXT:%s,GROUP:%s,TYPE:%s, Display:%d",txt.c_str(),grp.c_str(),type.c_str(),alarm.display);
                }
                else
                {
                    ROS_ERROR("%s", txt.c_str());
                }
            }
        }
        Alarms->alarms.push_back(alarm);
    }
    return;
}

void GraphNode::pop_old_alarms(robotnik_msgs::alarmsmonitor *Alarms)
{
    for (int i = 0; i < Alarms->alarms.size(); i++)
    {
        Alarms->alarms[i].seconds_active = Alarms->alarms[i].seconds_active + 1;
        //alarm.text=alarm.text+"1";
        if (Alarms->alarms[i].seconds_active > 10)
        {
            // Deleting old alarm;
            std::vector<robotnik_msgs::alarmmonitor>::iterator it = Alarms->alarms.begin();
            std::advance(it, i);
            Alarms->alarms.erase(it);
        }
    }
    return;
}

void GraphNode::timerPublishAlarms(const ros::TimerEvent &event)
{
    pop_old_alarms(&Alarms);
    alarm_pub_.publish(Alarms);
}

void GraphNode::timerPublishCallback(const ros::TimerEvent &event)
{

    rosPublish();
}

/*! \fn GraphNode::GraphNode()
 *  \brief Constructor by default
 *	\param hz as double, sets the desired frequency of the controlthread
 *	\param h as ros::NodeHandle, ROS node handle
*/
GraphNode::GraphNode(ros::NodeHandle h) : nh_(h), pnh_("~")
{
    // Set main flags to false
    ros_initialized = initialized = running = false;
    // reads params from server
    rosReadParams();

    if (desired_freq_ <= 0.0)
        desired_freq_ = DEFAULT_THREAD_DESIRED_HZ;

    state = robotnik_msgs::State::INIT_STATE;
    // Realizar para cada una de las clases derivadas
    component_name.assign("GraphNode");

    vRobotStatusSubs.resize(iMaxRobots);
    vRobotStatus.resize(iMaxRobots);

    for (int i = 0; i < iMaxRobots; i++)
    {
        vRobotStatus[i].id = -1;
    }

    vRobotStatusTopicName.resize(iMaxRobots);
    vLastNodeDemoMode.resize(iMaxRobots);

    pthread_mutex_init(&mutexGraph, NULL);
}

/*! \fn GraphNode::~GraphNode()
 * Destructor by default
*/
GraphNode::~GraphNode()
{
}

/*! \fn int GraphNode::setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
int GraphNode::setup()
{
    // Checks if has been initialized
    if (initialized)
    {
        ROS_INFO("%s::Setup: Already initialized", component_name.c_str());

        return INITIALIZED;
    }

    ifstream f(graph_file_);
    if (!f.good())
    {
        PUSH_ERROR_HMI("Error Reading File:" + graph_file_);
        //ROS_ERROR("--------------------------------------------------------------------------------");
        //return ERROR;
    }

    //
    ///////////////////////////////////////////////////
    // Setups the component or another subcomponents if it's necessary //
    ///////////////////////////////////////////////////
    graph_route = new Graph(graph_file_.c_str());

    std::string sError;

    if (graph_route->setup(&sError) != OK)
    {
        PUSH_ERROR_HMI("Error Reading Graph:" + sError);
        return ERROR;
    }
    initialized = true;

    return OK;
}

/*! \fn int GraphNode::shutDown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return RUNNING if the component is running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int GraphNode::shutdown()
{

    if (running)
    {
        ROS_INFO("%s::Shutdown: Impossible while thread running, first must be stopped", component_name.c_str());
        return THREAD_RUNNING;
    }
    initialized = false;

    return OK;
}

/*! \fn int GraphNode::start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int GraphNode::start()
{
    // Performs ROS setup
    rosSetup();
    if (running)
    {
        ROS_INFO("%s::start: the component's thread is already running", component_name.c_str());
        return THREAD_RUNNING;
    }
    ROS_INFO("%s started", component_name.c_str());
    running = true;
    // Executes the control loop
    controlLoop();
    PUSH_EVENT("STARTING");
    return OK;
}

/*! \fn int GraphNode::stop()
 * Stops the control thread of the Motors
 * \return OK
 * \return ERROR if it can't be stopped
 * \return THREAD_NOT_RUNNING if the thread is not running
*/
int GraphNode::stop()
{
    if (!running)
    {
        ROS_INFO("%s::stop: Thread not running", component_name.c_str());

        return THREAD_NOT_RUNNING;
    }
    //
    ///////////////////////////////////////////////////
    // Stops another subcomponents, if it's necessary //
    ///////////////////////////////////////////////////
    //
    ROS_INFO("%s::Stop: Stopping the component", component_name.c_str());
    running = false;
    usleep(100000);
    return OK;
}

/*!	\fn void GraphNode::controlLoop()
 *	\brief All core component functionality is contained in this thread.
*/
void GraphNode::controlLoop()
{
    ROS_INFO("%s::controlLoop(): Init", component_name.c_str());
    ros::Rate r(desired_freq_);
    ros::Time t1, t2;
    while (running && ros::ok())
    {
        t1 = ros::Time::now();
        switch (state)
        {
        case robotnik_msgs::State::INIT_STATE:
            initState();
            break;
        case robotnik_msgs::State::STANDBY_STATE:
            standbyState();
            break;
        case robotnik_msgs::State::READY_STATE:
            readyState();
            break;
        case robotnik_msgs::State::SHUTDOWN_STATE:
            shutdownState();
            break;
        case robotnik_msgs::State::EMERGENCY_STATE:
            emergencyState();
            break;
        case robotnik_msgs::State::FAILURE_STATE:
            failureState();
            break;
        }
        allState();
        ros::spinOnce();
        r.sleep();
        t2 = ros::Time::now();
        real_freq = 1.0 / (t2 - t1).toSec();
    }
    shutdownState();
    // Performs ROS Shutdown
    rosShutdown();
    ROS_INFO("%s::controlLoop(): End", component_name.c_str());
}

/*!	\fn void GraphNode::initState()
 *	\brief Actions performed on initial
 * 	Setups the component
*/
void GraphNode::initState()
{
    // If component setup is successful goes to STANDBY (or READY) state
    if (setup() != ERROR)
    {
        switchToState(robotnik_msgs::State::STANDBY_STATE);
    } //else switchToState(robotnik_msgs::State::SHUTDOWN_STATE);
}

/*!	\fn void GraphNode::shutdownState()
 *	\brief Actions performed on Shutdown state
*/
void GraphNode::shutdownState()
{
    stop();
    if (shutdown() == OK)
    {
        //switchToState(robotnik_msgs::State::INIT_STATE);
    }
}

/*!	\fn void GraphNode::standbyState()
 *	\brief Actions performed on Standby state
*/
void GraphNode::standbyState()
{
    switchToState(robotnik_msgs::State::READY_STATE);
}

/*!	\fn void GraphNode::readyState()
 *	\brief Actions performed on ready state
*/
void GraphNode::readyState()
{
}

/*!	\fn void GraphNode::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void GraphNode::emergencyState()
{
}

/*!	\fn void GraphNode::FailureState()
 *	\brief Actions performed on failure state
*/
void GraphNode::failureState()
{
}

/*!	\fn void GraphNode::AllState()
 *	\brief Actions performed on all states
*/
void GraphNode::allState()
{
    diagnostic_->update();
    //rosPublish();
}

/*!	\fn double GraphNode::getUpdateRate()
 * 	\brief Gets current update rate of the thread
 * 	\return real frequency of the thread
*/
double GraphNode::getUpdateRate()
{
    return desired_freq_;
}

/*!	\fn int GraphNode::getState()
 * 	\brief returns the state of the component
*/
int GraphNode::getState()
{
    return state;
}

/*!	\fn char *GraphNode::getStateString()
 *	\brief Gets the state of the component as string
*/
char *GraphNode::getStateString()
{
    return getStateString(state);
}

/*!	\fn char *GraphNode::getStateString(int state)
 *	\brief Gets the state as a string
*/
char *GraphNode::getStateString(int state)
{
    switch (state)
    {
    case robotnik_msgs::State::INIT_STATE:
        return (char *)"INIT";
        break;
    case robotnik_msgs::State::STANDBY_STATE:
        return (char *)"STANDBY";
        break;
    case robotnik_msgs::State::READY_STATE:
        return (char *)"READY";
        break;
    case robotnik_msgs::State::EMERGENCY_STATE:
        return (char *)"EMERGENCY";
        break;
    case robotnik_msgs::State::FAILURE_STATE:
        return (char *)"FAILURE";
        break;
    case robotnik_msgs::State::SHUTDOWN_STATE:
        return (char *)"SHUTDOWN";
        break;
    default:
        return (char *)"UNKNOWN";
        break;
    }
}

/*!	\fn void GraphNode::switchToState(int new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void GraphNode::switchToState(int new_state)
{

    if (new_state == state)
        return;

    // saves the previous state
    previous_state = state;
    ROS_INFO("%s::SwitchToState: %s -> %s", component_name.c_str(), getStateString(state), getStateString(new_state));
    state = new_state;
}

/*!	\fn void GraphNode::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int GraphNode::rosSetup()
{

    // Checks if has been initialized
    if (ros_initialized)
    {
        ROS_INFO("%s::rosSetup: Already initialized", component_name.c_str());
        return INITIALIZED;
    }

    // Publishers
    state_publisher_ = pnh_.advertise<robotnik_fms_msgs::State>("state", 1);
    NodeUsedPublisher = pnh_.advertise<robotnik_fms_msgs::NodesInfo>("nodes_used", 1);
    NodeIDUsedPublisher = pnh_.advertise<robotnik_fms_msgs::NodesID>("nodes_id_used", 1);
    GraphPublisher = pnh_.advertise<robotnik_fms_msgs::NodesInfo>("graph", 1);
    graph_scale_publisher_ = pnh_.advertise<std_msgs::Float32>("graph_marker_scale", 1);

    ros_initialized = true;

    // Services server
    route_service_server_ = pnh_.advertiseService("get_route", &GraphNode::routeServiceServerCb, this);
    reload_graph_service_server_ = pnh_.advertiseService("reload_graph", &GraphNode::reloadGraphServiceServerCb, this);
    get_node_info_service_server_ = pnh_.advertiseService("get_node_info", &GraphNode::getNodeInfoServiceServerCb, this);
    block_node_service_server_ = pnh_.advertiseService("block_node", &GraphNode::blockNodeServiceServerCb, this);
    check_blocked_node_service_server_ = pnh_.advertiseService("get_blocked_node", &GraphNode::getBlockedNodeServiceServerCb, this);

    timerPublish = pnh_.createTimer(ros::Duration(dGraphFreq_), &GraphNode::timerPublishCallback, this);

    markers_pub_nodes_used = pnh_.advertise<visualization_msgs::MarkerArray>("graph_nodes_used_marker_array", 10);
    markers_pub_graph = pnh_.advertise<visualization_msgs::MarkerArray>("graph_marker_array", 10);
    markers_pub_robots = pnh_.advertise<visualization_msgs::MarkerArray>("graph_marker_robots", 10);

    // Sets up the diagnostic updater
    diagnostic_ = new diagnostic_updater::Updater();

    diagnostic_->setHardwareID("GraphNode");
    diagnostic_->add("State", this, &GraphNode::diagnosticUpdate);
    diagnostic_->broadcast(0, "Doing important initialization stuff.");

    alarm_pub_ = pnh_.advertise<robotnik_msgs::alarmsmonitor>("/monitoring_alarms/alarms", 1);
    timerAlarms = pnh_.createTimer(ros::Duration(0.2), &GraphNode::timerPublishAlarms, this);

    for (int i = 0; i < iMaxRobots; i++)
    {
        std::string msg = " Suscribing to" + vRobotStatusTopicName[i];
        PUSH_EVENT(msg);
        vRobotStatusSubs[i] = pnh_.subscribe<robotnik_fms_msgs::RobotStatus>(vRobotStatusTopicName[i].c_str(), 1, &GraphNode::status_subscriberCB, this);
    }

    return OK;
}

void GraphNode::status_subscriberCB(robotnik_fms_msgs::RobotStatus msg)
{
    //ROS_INFO("Received %d",msg.id);
    if ((msg.id >= 0) && (msg.id < iMaxRobots))
    {
        vRobotStatus[msg.id] = msg;
        graph_route->reserveNode(msg.id, msg.node);
        //ROS_INFO("id:%d node:%d topic:%s",msg.id,msg.node,);
    }
    else
    {
        PUSH_ERROR("Received msg from Robot:" + std::to_string(msg.id) + " Check robot configuration launcher");
    }
}

/*!	\fn void GraphNode::rosReadParams
 * 	\brief Reads the params set in ros param server
*/
void GraphNode::rosReadParams()
{
    pnh_.param("desired_freq", desired_freq_, 10.0);
    pnh_.param<string>("graph_file", graph_file_, "route.xml");
    pnh_.param("display_alarm_monitor", iDisplay_Alarm_Monitor_, 7);
    ROS_INFO("GraphNode::rosReadParams: graph file = %s", graph_file_.c_str());

    pnh_.getParam("RobotStatusTopicName", vRobotStatusTopicName);
    pnh_.param("max_robots", iMaxRobots, 1);
    pnh_.param("max_robots", iMaxRobots, 1);

    pnh_.param("graph_public_freq", dGraphFreq_, 2.0);

    if (dGraphFreq_ <= 0)
        dGraphFreq_ = 1.0;

    pnh_.param("demo", bDemo_, false);
    if (bDemo_)
    {
        std::string msg = "---Demo-- Mode:" + std::to_string(bDemo_) + " Graph File:" + graph_file_;
        PUSH_EVENT(msg);
    }
    else
    {
        std::string msg = " Graph File:" + graph_file_;
        PUSH_EVENT(msg);
    }
    pnh_.param("graph_markers_scale", fGraph_markers_scale, 1.0);
}

/*!	\fn int GraphNode::rosShutdown()
 * 	\brief Closes all ros stuff
*/
int GraphNode::rosShutdown()
{
    if (running)
    {
        ROS_INFO("%s::rosShutdown: Impossible while thread running, first must be stopped", component_name.c_str());
        return THREAD_RUNNING;
    }
    if (!ros_initialized)
    {
        ROS_INFO("%s::rosShutdown: Impossible because of it's not initialized", component_name.c_str());
        return NOT_INITIALIZED;
    }
    ros_initialized = false;
    return OK;
}

/*!	\fn void GraphNode::rosPublish()
 * 	\brief Reads data a publish several info into different topics
*/
void GraphNode::rosPublish()
{
    if (!initialized)
        return;

    visualization_msgs::MarkerArray marker_array_robots_msg;

    for (int i = 0; i < vRobotStatus.size(); i++)
    {

        robotnik_fms_msgs::RobotStatus RS = vRobotStatus[i];

        //marker_array_robots_msg.markers.resize(iMaxRobots);
        //marker_array_robots_msg.markers.clear();

        visualization_msgs::Marker MRobot;

        if (RS.id >= 0)
        {
            MRobot.header.stamp = ros::Time::now();
            MRobot.header.frame_id = RS.map;
            MRobot.ns = "basic_shapes";
            MRobot.id = i;
            MRobot.type = visualization_msgs::Marker::ARROW;
            MRobot.color.r = 0.0f;
            MRobot.color.g = 1.0f;
            MRobot.color.b = 1.0f;
            MRobot.color.a = 1.0f;

            MRobot.action = visualization_msgs::Marker::ADD;
            MRobot.pose.position.x = RS.position.position.x / fGraph_markers_scale;
            MRobot.pose.position.y = RS.position.position.y / fGraph_markers_scale;
            MRobot.pose.position.z = RS.position.position.z;

            MRobot.pose.orientation = tf::createQuaternionMsgFromYaw(RS.theta);
            //MRobot.pose.orientation.x = 0.0;
            //MRobot.pose.orientation.y = 0.0;
            //MRobot.pose.orientation.z = 0.0;
            //MRobot.pose.orientation.w = 1.0;

            MRobot.scale.x = 0.4;
            MRobot.scale.y = 0.4;
            MRobot.scale.z = 0.4;

            MRobot.lifetime = ros::Duration((1.0 / dGraphFreq_) * 10.0);

            stringstream streamx;
            streamx << fixed << setprecision(2) << RS.position.position.x;
            string x = streamx.str();

            stringstream streamy;
            streamy << fixed << setprecision(2) << RS.position.position.y;
            string y = streamy.str();

            stringstream streamt;
            streamt << fixed << setprecision(2) << RS.theta;
            string T = streamt.str();

            stringstream streamb;
            streamb << fixed << setprecision(2) << RS.battery;
            string B = streamb.str();

            std::string txt = "R:" + std::to_string(i) + "X" + x +
                              "Y" + y +
                              "T" + T +
                              "N" + std::to_string(RS.node) +
                              "B" + B;

            if (!RS.connected)
                txt = txt + "-NC-";

            //" N:" + std::to_string(graph_route->dijkstraGraph->GetNearestNodeID(RS.position.position.x,RS.position.position.y,RS.map));

            //ROS_INFO("----------%s",txt.c_str());

            visualization_msgs::Marker MText;
            MText.header.stamp = ros::Time::now();
            MText.header.frame_id = RS.map;
            MText.ns = "basic_shapes";
            MText.id = i + 1000;
            MText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            MText.action = visualization_msgs::Marker::ADD;
            MText.text = txt;
            MText.pose.position.x = RS.position.position.x / fGraph_markers_scale;
            MText.pose.position.y = RS.position.position.y / fGraph_markers_scale;
            MText.pose.position.z = RS.position.position.z + 1.5 + RS.id * 0.5;

            MText.pose.orientation.x = 0.0;
            MText.pose.orientation.y = 0.0;
            MText.pose.orientation.z = 0.0;
            MText.pose.orientation.w = 1.0;
            MText.scale.x = 0.3;
            MText.scale.y = 0.3;
            MText.scale.z = 0.3;
            MText.color.r = 1.0f;
            MText.color.g = 0.0f;
            MText.color.b = 0.5f;
            MText.color.a = 1.0f;
            MText.lifetime = ros::Duration((1.0 / dGraphFreq_) * 10.0);

            marker_array_robots_msg.markers.push_back(MRobot);
            marker_array_robots_msg.markers.push_back(MText);
        }
    }

    markers_pub_robots.publish(marker_array_robots_msg);

    robotnik_fms_msgs::State msg;

    // STATE

    msg.state.state = this->state;
    msg.state.desired_freq = this->desired_freq_;
    msg.state.real_freq = this->real_freq;
    msg.state.state_description = getStateString();
    pthread_mutex_lock(&mutexGraph);
    msg.nodes = graph_route->getNodes();
    pthread_mutex_unlock(&mutexGraph);
    state_publisher_.publish(msg);

    std_msgs::Float32 msgscale;
    msgscale.data = fGraph_markers_scale;

    //ROS_INFO("scale %f",fGraph_markers_scale);

    graph_scale_publisher_.publish(msgscale);

    std::vector<Node *> route;
    robotnik_fms_msgs::NodesInfo Nodes;
    robotnik_fms_msgs::NodesID NodesId;

    graph_route->getNodesUsed(&route);

    //ROS_INFO(" ROUTE ------------ Size:%d",route.size());

    std::string text_nodes_used;

    if (route.size() > 0)
    {
        for (int i = 0; i < route.size(); i++)
        {
            robotnik_fms_msgs::NodeInfo Node_inf;
            Node *nod;
            nod = route[i];

            copyNodeToMessage(nod, &Node_inf);

            graph_frame = nod->sFrame_id;

            Nodes.Nodes.push_back(Node_inf);
            std::string sID = "ID:" + std::to_string(nod->iNode);

            if (nod->iRobot >= 0)
            {
                sID = sID + "/R:" + std::to_string(nod->iRobot);
                text_nodes_used = text_nodes_used + sID;
            }
            if (nod->iResRobot >= 0)
            {
                sID = sID + "/Res:" + std::to_string(nod->iResRobot);
                text_nodes_used = text_nodes_used + sID;
            }
            NodesId.id.push_back(sID);
        }

        visualization_msgs::MarkerArray marker_array_msg;
        uint32_t shape = visualization_msgs::Marker::CUBE;

        marker_array_msg.markers.resize(route.size());
        for (int i = 0; i < route.size(); i++)
        {
            robotnik_fms_msgs::NodeInfo Node_inf;
            Node *nod;
            nod = route[i];
            marker_array_msg.markers[i].header.stamp = ros::Time::now();
            marker_array_msg.markers[i].header.frame_id = nod->sFrame_id;
            marker_array_msg.markers[i].ns = "basic_shapes";
            marker_array_msg.markers[i].id = i + 2000;
            marker_array_msg.markers[i].type = shape;
            marker_array_msg.markers[i].color.r = 0.0f + nod->iNode;
            marker_array_msg.markers[i].color.g = 1.0f + nod->iRobot;
            marker_array_msg.markers[i].color.b = 0.0f + nod->iNode;
            marker_array_msg.markers[i].color.a = 1.0f + nod->iNode;

            marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
            marker_array_msg.markers[i].pose.position.x = nod->dX / fGraph_markers_scale;
            marker_array_msg.markers[i].pose.position.y = nod->dY / fGraph_markers_scale;
            marker_array_msg.markers[i].pose.position.z = nod->dZ;
            marker_array_msg.markers[i].pose.orientation.x = 0.0;
            marker_array_msg.markers[i].pose.orientation.y = 0.0;
            marker_array_msg.markers[i].pose.orientation.z = 0.0;
            marker_array_msg.markers[i].pose.orientation.w = 1.0;

            marker_array_msg.markers[i].scale.x = 0.2;
            marker_array_msg.markers[i].scale.y = 0.2;
            marker_array_msg.markers[i].scale.z = 0.2;

            marker_array_msg.markers[i].lifetime = ros::Duration((1.0 / dGraphFreq_) * 10.0);

            if (nod->iRobot >= 0)
            {
                std::string txt = "";

                visualization_msgs::Marker MText;
                MText.header.stamp = ros::Time::now();
                MText.header.frame_id = nod->sFrame_id;
                MText.ns = "basic_shapes";
                MText.id = i + 1000;
                MText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                MText.action = visualization_msgs::Marker::ADD;
                MText.text = txt + std::to_string(nod->iRobot);
                MText.pose.position.x = nod->dX / fGraph_markers_scale;
                MText.pose.position.y = nod->dY / fGraph_markers_scale;
                MText.pose.position.z = nod->dZ + .3;

                MText.pose.orientation.x = 0.0;
                MText.pose.orientation.y = 0.0;
                MText.pose.orientation.z = 0.0;
                MText.pose.orientation.w = 1.0;
                MText.scale.x = 0.3;
                MText.scale.y = 0.3;
                MText.scale.z = 0.3;
                MText.color.r = 1.0f;
                MText.color.g = 0.0f;
                MText.color.b = 1.0f;
                MText.color.a = 1.0f;
                MText.lifetime = ros::Duration((1.0 / dGraphFreq_) * 10.0);
                marker_array_msg.markers.push_back(MText);
            }
        }
        markers_pub_nodes_used.publish(marker_array_msg);

        NodeUsedPublisher.publish(Nodes);
        NodeIDUsedPublisher.publish(NodesId);
    }
    else
    {
        NodesId.id.push_back("-");
        NodeIDUsedPublisher.publish(NodesId);
    }
    visualization_msgs::MarkerArray marker_array_msg;
    //visualization_msgs::MarkerArray marker_array_arrow_msg;
    int iID = 0;
    float scale = 0.2 * fGraph_markers_scale;

    robotnik_fms_msgs::NodesInfo Graph;

    if (graph_route->dijkstraGraph->vNodes.size() > 0)
    {

        for (int i = 0; i < graph_route->dijkstraGraph->vNodes.size(); i++)
        {
            robotnik_fms_msgs::NodeInfo Node_inf;
            Node nod = graph_route->dijkstraGraph->vNodes[i];

            std::string sZones = "Z";
            for (int j = 0; j < nod.viZones.size(); j++)
            {
                Node_inf.zones.push_back(nod.viZones[j]);
                sZones = sZones + std::to_string(nod.viZones[j]);
            }

            copyNodeToMessage(&nod, &Node_inf);

            Graph.Nodes.push_back(Node_inf);

            //ROS_INFO("Node:%d Pos:%f,%f arcs:%d",nod.iNode,nod.dX,nod.dY,nod.vAdjacent.size());

            visualization_msgs::Marker M;
            M.header.stamp = ros::Time::now();
            M.header.frame_id = nod.sFrame_id;
            M.ns = "basic_shapes";
            M.id = iID + 10000;
            if (isnan(nod.dTheta))
            {
                M.type = visualization_msgs::Marker::SPHERE;
                M.pose.orientation.x = 0.0;
                M.pose.orientation.y = 0.0;
                M.pose.orientation.z = 0.0;
                M.pose.orientation.w = 1.0;
            }
            else
            {
                M.type = visualization_msgs::Marker::ARROW;
                M.pose.orientation = tf::createQuaternionMsgFromYaw(nod.dTheta);
            }
            M.action = visualization_msgs::Marker::ADD;
            M.pose.position.x = nod.dX / fGraph_markers_scale;
            M.pose.position.y = nod.dY / fGraph_markers_scale;
            M.pose.position.z = nod.dZ;

            float fR = 1.0;
            float fG = 1.0;
            float fB = 1.0;
            float fA = 1.0;

            /* TODO Change color depending on action

            if (nod.bLoad)
                fR = 0.5;
            if (nod.bUnload)
                fG = 0.5;
            if (nod.bCharge)
                fB = 0.5;
                */

            M.scale.x = scale;      //+0.1*nod.bStop;
            M.scale.y = scale;      //+0.1*nod.bStop;
            M.scale.z = scale * .2; //+0.1*nod.bStop;
            M.color.r = fR;
            M.color.g = fG;
            M.color.b = fB;
            M.color.a = fA;
            M.lifetime = ros::Duration((1.0 / dGraphFreq_) * 10.0);
            marker_array_msg.markers.push_back(M);
            iID++;

            std::string txt;
            if (sZones.length() > 1)
                txt = sZones + ":";
            else
                txt = "";

            if (txt != "")
                txt = txt + ":";
            visualization_msgs::Marker MText;
            MText.header.stamp = ros::Time::now();
            MText.header.frame_id = nod.sFrame_id;
            MText.ns = "basic_shapes";
            MText.id = iID;
            MText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            MText.action = visualization_msgs::Marker::ADD;
            txt = txt + std::string(nod.cName); //std::to_string(nod.iNode);
            if (nod.iResRobot >= 0)
                txt = txt + " Res:" + std::to_string(nod.iResRobot);
            if (nod.iRobot >= 0)
                txt = txt + " R:" + std::to_string(nod.iRobot);
            MText.text = txt;
            //ROS_INFO("%s NodesUsed:%s",txt.c_str(),text_nodes_used.c_str());
            MText.pose.position.x = nod.dX / fGraph_markers_scale;
            MText.pose.position.y = nod.dY / fGraph_markers_scale;
            MText.pose.position.z = nod.dZ + 0.1;
            MText.pose.orientation.x = 0.0;
            MText.pose.orientation.y = 0.0;
            MText.pose.orientation.z = 0.0;
            MText.pose.orientation.w = 1.0;
            MText.scale.x = scale; //*3.0;
            MText.scale.y = scale; //*3.0;
            MText.scale.z = scale; //*3.0;
            MText.color.r = 1.0f;
            MText.color.g = 0.0f;
            MText.color.b = 1.0f;
            MText.color.a = 1.0f;
            MText.lifetime = ros::Duration((1.0 / dGraphFreq_) * 10.0);
            marker_array_msg.markers.push_back(MText);
            iID++;

            //marker_array_arrow_msg.markers.resize(nod.vAdjacent.size());
            for (int j = 0; j < nod.vAdjacent.size(); j++)
            {

                int iNext = nod.vAdjacent[j].iNextNode;
                //Node nodDest=graph_route->dijkstraGraph->vNodes[iNext];
                Node *nodDest = graph_route->dijkstraGraph->getNodeFromID(iNext);

                if (nodDest != NULL)
                {
                    //ROS_INFO("Adding Arc:%d from Node:%d[%f,%f]F:%s name:%s to Node:%d[%f,%f]F:%s name:%s",j,nod.iNode,nod.dX,nod.dY,nod.sFrame_id.c_str(),std::string(nod.cName).c_str(),nodDest->iNode,nodDest->dX,nodDest->dY,nodDest->sFrame_id.c_str(),std::string(nodDest->cName).c_str());

                    if (nod.sFrame_id == nodDest->sFrame_id)
                    {

                        visualization_msgs::Marker M;

                        // tf::TransformListener listener;
                        //
                        // tf::StampedTransform transform;
                        // try{
                        //     listener.lookupTransform( nodDest->sFrame_id,nod.sFrame_id,ros::Time(0), transform);
                        // }
                        // catch (tf::TransformException ex){
                        //     ROS_ERROR("%s",ex.what());
                        //     ros::Duration(1.0).sleep();
                        //}

                        M.header.stamp = ros::Time::now();
                        M.header.frame_id = nod.sFrame_id;
                        M.ns = "basic_shapes";
                        M.id = iID;

                        M.type = visualization_msgs::Marker::ARROW;
                        M.action = visualization_msgs::Marker::ADD;

                        M.pose.position.x = 0.0; //nod.dX;
                        M.pose.position.y = 0.0; //nod.dY;
                        M.pose.position.z = 0.0;
                        M.pose.orientation.x = 0.0;
                        M.pose.orientation.y = 0.0;
                        M.pose.orientation.z = 0.0;
                        M.pose.orientation.w = 1.0;

                        M.scale.x = 0.02;
                        M.scale.y = 0.05;
                        //marker_array_msg.markers[i+j].scale.z =scale;

                        M.color.r = 1.0;
                        M.color.g = 1.0;
                        M.color.b = 1.0;
                        M.color.a = 1.0;

                        M.points.resize(2);

                        M.points[0].x = nod.dX / fGraph_markers_scale;
                        M.points[0].y = nod.dY / fGraph_markers_scale;
                        M.points[0].z = nod.dZ;

                        M.points[1].x = nodDest->dX / fGraph_markers_scale;
                        M.points[1].y = nodDest->dY / fGraph_markers_scale;
                        M.points[1].z = nodDest->dZ;

                        M.lifetime = ros::Duration((1.0 / dGraphFreq_) * 10.0);
                        marker_array_msg.markers.push_back(M);
                        iID++;
                    }
                    else
                    {
                        //ROS_ERROR(" nod:%d sFrame_id_or:%s nodDest:%d sFrame_id_dest:%s",nod.iNode,nod.sFrame_id.c_str(),nodDest->iNode,nodDest->sFrame_id.c_str());
                    }
                }
            }
        }
        //markers_pub_graph_arcs.publish(marker_array_arrow_msg);
        markers_pub_graph.publish(marker_array_msg);
        GraphPublisher.publish(Graph);
    }
}

/*! \fn void CopyNodeToMessage(Node *NodeOr,robotnik_fms_msgs::NodeInfo *MessageDest)
 *  \brief Copy Node from Class to message
 */
void GraphNode::copyNodeToMessage(Node *Or, robotnik_fms_msgs::NodeInfo *Dest)
{
    //
    //Any modification in this function must be replicated in CopyMessageToNode
    //

    Dest->id = Or->iNode;
    Dest->position.pose.x = Or->dX;
    Dest->position.pose.y = Or->dY;
    Dest->coord_z = Or->dZ;
    Dest->position.pose.theta = Or->dTheta;
    Dest->position.header.frame_id = Or->sFrame_id;
    Dest->robot = Or->iRobot;
    Dest->robot_reserved = Or->iResRobot;
    Dest->name = std::string(Or->cName);
}

/*! \fn void CopyMessageToNode()
 *  \brief Copy Node from Message To Class
 */
void GraphNode::copyMessageToNode(robotnik_fms_msgs::NodeInfo *Or, Node *Dest)
{
    //
    //Any modification in this function must be replicated in CopyNodeToMessage
    //

    Dest->iNode = Or->id;
    Dest->dX = Or->position.pose.x;
    Dest->dY = Or->position.pose.y;
    Dest->dZ = Or->coord_z;
    Dest->dTheta = Or->position.pose.theta;
    Dest->sFrame_id = Or->position.header.frame_id;
    Dest->iRobot = Or->robot;
    Dest->iResRobot = Or->robot_reserved;
    strcpy(Dest->cName, Or->name.c_str());
}

/*! \fn bool GetNodesUsed(std::vector<Node*>*route)
 *  \brief Get Nodes Used
 */
bool GraphNode::getNodesUsed(std::vector<Node *> *route)
{
    return graph_route->getNodesUsed(route);
}

/*!	\fn void GraphNode::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat)
 * 	\brief Callback to update the component diagnostic
*/
void GraphNode::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

    if (state == robotnik_msgs::State::READY_STATE || state == robotnik_msgs::State::INIT_STATE || state == robotnik_msgs::State::STANDBY_STATE)
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything OK!");
    else if (state == robotnik_msgs::State::EMERGENCY_STATE)
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Watch out!");
    else
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Error!");

    stat.add("State", getStateString());
}

// Callback handler for the service server
bool GraphNode::routeServiceServerCb(robotnik_fms_msgs::GetRoute::Request &request, robotnik_fms_msgs::GetRoute::Response &response)
{
    pthread_mutex_lock(&mutexGraph);
    ROS_WARN_STREAM("GetRouteRequest");
    ROS_INFO_STREAM(request);
    if (state != robotnik_msgs::State::READY_STATE)
    {
        ROS_ERROR("GraphNode::routeServiceServerCb: Component not ready");
        pthread_mutex_unlock(&mutexGraph);
        response.ret = false;
        response.msg = "Component Not READY";
        pthread_mutex_unlock(&mutexGraph);
        return true;
    }
    vector<Node> v_nodes;
    vector<double> v_velocities;
    std::string nodes = "Nodes:";

    if (request.from_node == request.to_node)
    {
        robotnik_fms_msgs::NodeInfo nod;
        Node *nod_temp = graph_route->getNode(request.from_node);

        copyNodeToMessage(nod_temp, &nod);

        nod.name = std::string(nod_temp->cName);

        nodes = nodes + std::to_string(nod_temp->iNode);

        response.nodes.push_back(nod);
    }
    else
    {
        if (graph_route->getRoute(request.from_node, request.to_node, &v_nodes, &v_velocities) != 0)
        {
            ROS_ERROR("GraphNode::routeServiceServerCb: Error getting route between %d and %d", request.from_node, request.to_node);
            pthread_mutex_unlock(&mutexGraph);
            response.ret = false;
            response.msg = "Route Not Found";
            return true;
        }
        geometry_msgs::Pose2D pose;
        for (int i = 0; i < (int)v_nodes.size(); i++)
        {
            robotnik_fms_msgs::NodeInfo nod;

            copyNodeToMessage(&v_nodes[i], &nod);

            nodes = nodes + std::to_string(v_nodes[i].iNode);
            if (nod.stop)
                nodes = nodes + "P,";
            else
                nodes = nodes + ",";
            response.nodes.push_back(nod);
            //response.nodes.insert(response.nodes.begin(),nod);
        }
        response.velocities = v_velocities;
    }
    pthread_mutex_unlock(&mutexGraph);

    response.ret = true;
    response.msg = "Route Found from " + std::to_string(request.from_node) + " to " + std::to_string(request.to_node) + " " + nodes;

    ROS_WARN_STREAM("GetRouteResponse");
    ROS_INFO_STREAM("Response nodes size " << response.nodes.size());
    ROS_INFO_STREAM("Response nodes " << response.msg);
    return true;
}

// Callback handler for the service server
bool GraphNode::reloadGraphServiceServerCb(robotnik_fms_msgs::ReloadGraph::Request &request, robotnik_fms_msgs::ReloadGraph::Response &response)
{
    pthread_mutex_lock(&mutexGraph);
    string new_file;

    if (request.file.size() > 0)
    {
        new_file = request.file;
    }
    else
    {
        // If no file -> new_file = last file
        // This will be used to reset graph
        new_file = graph_file_;
    }

    PUSH_EVENT_HMI(" Reading new File:" + new_file);

    Graph *graph_route_aux = new Graph(new_file.c_str());

    std::string msg;
    if (graph_route_aux->setup(&msg) != OK)
    {
        pthread_mutex_unlock(&mutexGraph);
        PUSH_ERROR_HMI(" Error in Graph Setup:" + msg);
        return false;
    }

    // free memory of current graph
    delete graph_route;

    graph_file_ = new_file;
    graph_route = graph_route_aux;
    ROS_INFO("GraphNode::reloadGraphServiceServerCb: New graph loaded from file %s", graph_file_.c_str());

    response.ret = true;
    pthread_mutex_unlock(&mutexGraph);
    return true;
}

//! True if it is posible to block/free node idNode by Robot iRobot
bool GraphNode::checkNode(bool bBlock, int idNode, int iRobot, string *msg)
{
    Node *node_info = NULL;
    node_info = graph_route->getNode(idNode);
    if (node_info == NULL)
    {
        *msg = "NOT FOUND Node:" + std::to_string(idNode);
        return false;
    }

    if (bBlock)
    {

        if (!graph_route->checkNodeFree(idNode, iRobot))
        {
            *msg = "Node:" + std::to_string(idNode) + " ALREADY used by other Robot:" + std::to_string(node_info->iRobot);
            return false;
        }

        //if (node_info->bBlocked){
        if (node_info->iRobot >= 0)
        {
            if (node_info->iRobot == iRobot)
            {
                // Already Blocked by this robot
                *msg = "Node:" + std::to_string(idNode) + " ALREADY Blocked by Robot:" + std::to_string(iRobot);
                return true;
            }
            else
            {
                // Already Blocked by other robot
                *msg = "Node:" + std::to_string(idNode) + " ALREADY Blocked by other Robot:" + std::to_string(node_info->iRobot);
                return false;
            }
        }
        else
        {
            // Block OK
            *msg = "Node:" + std::to_string(idNode) + " Block Possible by Robot:" + std::to_string(node_info->iRobot);
            return true;
        }
    }
    else
    {
        //if (node_info->bBlocked){
        if (node_info->iRobot >= 0)
        {
            // Unblock OK
            if (node_info->iRobot == iRobot)
            {
                *msg = "Node:" + std::to_string(idNode) + " Unblock Possible by Robot:%d " + std::to_string(node_info->iRobot);
                return true;
            }
            else
            {
                // Not Blocked by this robot
                *msg = "Trying to Unblock Node:" + std::to_string(idNode) + " by Robot " + std::to_string(iRobot) + " but BLOCKED by Robot:" + std::to_string(node_info->iRobot);
                return false;
            }
        }
        else
        {
            // Not Blocked
            *msg = "Trying to Unblock Node:" + std::to_string(idNode) + "by Robot " + std::to_string(iRobot) + "but Already Unblocked ";
            return true;
        }
    }
}

bool GraphNode::reserveNode(bool bReserve, int idNode, int iRobot, string *msg)
{
    Node *node_info = NULL;
    node_info = graph_route->getNode(idNode);
    if (node_info == NULL)
    {
        *msg = "NOT FOUND Node:" + std::to_string(idNode);
        return false;
    }
    if (bReserve)
    {

        if (!graph_route->checkNodeFree(idNode, iRobot))
        {
            *msg = "Not Possible to Reserve Node:" + std::to_string(idNode) + " ALREADY used by other Robot:" + std::to_string(node_info->iRobot);
            return false;
        }
        //if (node_info->bReserved){
        if (node_info->iResRobot >= 0)
        {
            if (node_info->iResRobot == iRobot)
            {
                // Already Reserved by this robot
                *msg = "Node:" + std::to_string(idNode) + " ALREADY Reserved by Robot:" + std::to_string(node_info->iRobot);
                return true;
            }
            else
            {
                // Already Reserved by other robot
                *msg = "Node:" + std::to_string(idNode) + " ALREADY Reserved by other Robot:" + std::to_string(node_info->iRobot);
                return false;
            }
        }
        else
        {
            // Reserved OK
            *msg = "Node:" + std::to_string(idNode) + " Reserved OK by Robot:" + std::to_string(iRobot);
            graph_route->reserveNode(iRobot, idNode);
            return true;
        }
    }
    else
    {
        //if (node_info->bReserved){
        if (node_info->iResRobot >= 0)
        {
            // Unblock OK
            if (node_info->iRobot == iRobot)
            {
                *msg = "Node:" + std::to_string(idNode) + " Unreserved OK by Robot:%d " + std::to_string(iRobot);
                graph_route->reserveNode(-1, idNode);
                //node_info->bReserved=false;
                return true;
            }
            else
            {
                // Not Reserved by this robot
                *msg = "Trying to Reserve Node:" + std::to_string(idNode) + " by Robot " + std::to_string(iRobot) + " but Reserved by Robot:" + std::to_string(node_info->iRobot);
                return false;
            }
        }
        else
        {
            // Not Reserved
            *msg = "Trying to Unreserve Node:" + std::to_string(idNode) + "by Robot " + std::to_string(iRobot) + "but Already Unreserved ";
            return true;
        }
    }
}

bool GraphNode::blockNode(bool bBlock, int idNode, int iRobot, string *msg)
{
    Node *node_info = NULL;

    /*
    if (idNode==-1) {
        graph_route->UnBlockAll(iRobot);
        return true;
    }*/

    node_info = graph_route->getNode(idNode);
    if (node_info == NULL)
    {
        *msg = "NOT FOUND Node:" + std::to_string(idNode);
        return false;
    }
    if (bBlock)
    {
        if (!graph_route->checkNodeFree(idNode, iRobot))
        {
            *msg = "Not Possible to Block Node:" + std::to_string(idNode) + " ALREADY used by other Robot:" + std::to_string(node_info->iRobot);
            return false;
        }
        if (node_info->iRobot >= 0)
        {
            //if (node_info->bBlocked){
            if (node_info->iRobot == iRobot)
            {
                // Already Blocked by this robot
                *msg = "Node:" + std::to_string(idNode) + " ALREADY Blocked by Robot:" + std::to_string(node_info->iRobot);
                return true;
            }
            else
            {
                // Already Blocked by other robot
                *msg = "Node:" + std::to_string(idNode) + " ALREADY Blocked by other Robot:" + std::to_string(node_info->iRobot);
                return false;
            }
        }
        else
        {
            // Block OK
            *msg = "Node:" + std::to_string(idNode) + " Blocked OK by Robot:" + std::to_string(iRobot);
            node_info->iRobot = iRobot;
            //    node_info->bBlocked=true;
            vLastNodeDemoMode[iRobot] = idNode;
            if (bDemo_)
                graph_route->reserveNode(iRobot, idNode);
            return true;
        }
    }
    else
    {
        //if (node_info->bBlocked){
        if (node_info->iRobot >= 0)
        {
            // Unblock OK
            if (node_info->iRobot == iRobot)
            {
                *msg = "Node:" + std::to_string(idNode) + " Unblocked OK by Robot:%d " + std::to_string(iRobot);
                node_info->iRobot = -1;
                //node_info->bBlocked=false;
                return true;
            }
            else
            {
                // Not Blocked by this robot
                *msg = "Trying to Unblock Node:" + std::to_string(idNode) + " by Robot " + std::to_string(iRobot) + " but BLOCKED by Robot:" + std::to_string(node_info->iRobot);
                return false;
            }
        }
        else
        {
            // Not Blocked
            *msg = "Trying to Unblock Node:" + std::to_string(idNode) + "by Robot " + std::to_string(iRobot) + "but Already Unblocked ";
            return true;
        }
    }
}

// Callback handler for the service server
bool GraphNode::blockNodeServiceServerCb(robotnik_fms_msgs::BlockNode::Request &request, robotnik_fms_msgs::BlockNode::Response &response)
{
    std::string msg = "BlockNodeService Robot:" + std::to_string(request.robot_id) +
                      " to block:" + std::to_string(request.block) /*+
            " to reserve:" + std::to_string(request.reserve)*/
                      + " Nodes:";
    std::string nodes = "";
    for (int i = 0; i < request.node_id.size(); i++)
        nodes = nodes + std::to_string(request.node_id[i]) + ",";
    msg = msg + nodes;

    pthread_mutex_lock(&mutexGraph);

    for (int i = 0; i < request.node_id.size(); i++)
    {
        std::string msg_local;
        bool bret = checkNode(request.block, request.node_id[i], request.robot_id, &msg_local);
        if (!bret)
        {
            response.ret = false;
            response.msg = msg + " Not Possible:" + msg_local;
            PUSH_ERROR(response.msg);

            pthread_mutex_unlock(&mutexGraph);
            return false;
        }
    }

    for (int i = 0; i < request.node_id.size(); i++)
    {
        std::string msg_local;
        bool bret = blockNode(request.block, request.node_id[i], request.robot_id, &msg_local);
        if (!bret)
        {
            response.ret = false;
            response.msg = msg + " Not Possible:" + msg_local;
            PUSH_ERROR(response.msg);
            pthread_mutex_unlock(&mutexGraph);
            return false;
        }
        else
        {
            msg = msg + " " + msg_local;
            //ROS_INFO("%s",msg.c_str());
        }
    }
    response.ret = true;
    response.msg = msg;
    //PUSH_EVENT(response.msg);
    pthread_mutex_unlock(&mutexGraph);
    return true;
}

// Callback handler for the service server
bool GraphNode::getBlockedNodeServiceServerCb(robotnik_fms_msgs::GetBlockedNode::Request &request, robotnik_fms_msgs::GetBlockedNode::Response &response)
{
    pthread_mutex_lock(&mutexGraph);
    response.blocked = !graph_route->checkNodesFree(request.node_id, request.robot_id);
    pthread_mutex_unlock(&mutexGraph);
    return true;
}

// Callback handler for the service server
bool GraphNode::getNodeInfoServiceServerCb(robotnik_fms_msgs::GetNodeInfo::Request &request, robotnik_fms_msgs::GetNodeInfo::Response &response)
{
    pthread_mutex_lock(&mutexGraph);
    Node *node_info = NULL;
    node_info = graph_route->getNode(request.node_id);

    if (node_info == NULL)
    {
        response.ret = false;
        response.msg = "Node NOT found";
        ROS_ERROR("GraphNode::getNodeInfoServiceServerCb: node %d doesn't exist", request.node_id);
        pthread_mutex_unlock(&mutexGraph);
        return false;
    }

    copyNodeToMessage(node_info, &response.node_info);

    response.ret = true;
    response.msg = "Node found";

    response.node_info.used = graph_route->checkNodeFree(request.node_id, -1);
    pthread_mutex_unlock(&mutexGraph);
    return true;
}

// MAIN
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotnik_fms_graph_node");
    ros::NodeHandle n;
    GraphNode controller(n);
    controller.start();
    return (0);
}
