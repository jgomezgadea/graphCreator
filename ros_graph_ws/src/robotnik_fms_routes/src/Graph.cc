/** \file Graph.cc
 * \author Robotnik Automation s.L.L.
 * \version 1.1
 * \date    2011
 * \brief Class for managing the nodes, magnets and routes of the system
 * (C) 2011 Robotnik Automation, SLL. All rights reserved
*/

#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <robotnik_fms_routes/Graph.h>

/*! \fn Graph::Graph(std::string graph_path)
 * 	\brief Constructor
*/
Graph::Graph(std::string graph_path)
{
    graph_path_ = graph_path;
    dijkstraGraph = new Dijkstra();
    bInitialized = false;
}

/*! \fn Graph::~Graph()
 * 	\brief Destructor
*/
Graph::~Graph()
{
    delete dijkstraGraph;
}

/*! \fn int Graph::setup()
 * 	\brief Configures and initializes the graph
 *  @return OK
 *  @return ERROR
 *  @return INITIALIZED
*/
std::string Graph::setup()
{
    ROS_INFO("Graph::start Setup");

    if (bInitialized)
    {
        return "Graph::setup: Already initialized";
    }
    std::string desmsg = deserialize();
    if (desmsg != "OK")
    {
        return "Graph::setup: Error deserializing the graph:" + desmsg;
    }
    desmsg = dijkstraGraph->finalizeEdition(dijkstraGraph->vNodes);
    if (desmsg != "OK")
    {
        return "Graph::setup: Error graph:" + desmsg;
    }

    ROS_INFO("Graph::end Setup OK");
    bInitialized = true;

    return "OK";
}

/*! \fn int Graph::ShutDown()
 * 	\brief Closes and frees the reserved resources
 *  @return OK
 *  @return NOT_INITIALIZED
*/
int Graph::shutDown()
{
    if (!bInitialized)
    {
        ROS_ERROR("Graph::shutDown: Impossible because it's not initialized");
        return NOT_INITIALIZED;
    }

    deleteAll();

    bInitialized = false;

    return OK;
}

/*! \fn void Graph::printNodes()
 * 	\brief Print nodes of current graph
*/
void Graph::printNodes()
{
    dijkstraGraph->printNodes();
}

/*! \fn void Graph::print()
 * 	\brief Print zones of current graph
*/
void Graph::printZones()
{
    dijkstraGraph->printZones();
}

/*! \fn int Graph::setGraph(graph_msgs::GraphNodeArray graph)
 * 	\brief Adds a new node to the graph
*/
int Graph::setGraph(graph_msgs::GraphNodeArray graph)
{
    delete dijkstraGraph;
    dijkstraGraph = new Dijkstra();

    for (int i = 0; i < graph.nodes.size(); i++)
    {
        dijkstraGraph->addNode(graph.nodes[i]);
    }

    return 0;
}

/*! \fn std::string Graph::addNode(graph_msgs::GraphNode node)
 * 	\brief Adds a new node to the graph
*/
std::string Graph::addNode(graph_msgs::GraphNode node)
{
    std::string res = dijkstraGraph->addNode(node);

    return res;
}

/*! \fn int Graph::addNode(int node, double x, double y, double z, double theta, std::string frame, char *name)
 * 	\brief Adds a new node to the graph
*/
graph_msgs::GraphNode Graph::addNode(std::string node, int zone, double x, double y, double z, double theta, std::string frame, std::string name)
{
    graph_msgs::GraphNode new_node;

    new_node.name = name;
    new_node.id = node;
    new_node.zone = zone;
    new_node.pose.x = x;
    new_node.pose.y = y;
    new_node.pose.z = z;
    new_node.pose.theta = theta;
    new_node.pose.frame_id = frame;

    dijkstraGraph->addNode(new_node);

    return new_node;
}

/*! \fn int Graph::addArc(std::string from_node_id, std::string to_node_id)
 * 	\brief Adds an arc from a node to another with constant weight
*/
int Graph::addArc(std::string from_node_id, std::string to_node_id)
{
    graph_msgs::GraphArc new_arc;
    new_arc.node_dest = to_node_id;
    new_arc.distance = 1;
    new_arc.max_speed = 1.5;

    dijkstraGraph->addArc(from_node_id, new_arc);

    return 0;
}

/*! \fn int Graph::addArc(int from_node_id, int to_node_id, float weight)
 * 	\brief Adds an arc from a node to another with weight
*/
int Graph::addArc(std::string from_node_id, std::string to_node_id, float weight)
{
    graph_msgs::GraphArc new_arc;
    new_arc.node_dest = to_node_id;
    new_arc.distance = weight;
    new_arc.max_speed = 1.5;

    dijkstraGraph->addArc(from_node_id, new_arc);

    return 0;
}

/*! \fn int Graph::addArc(int from_node_id, int to_node_id, float weight, float max_speed)
 * 	\brief Adds an arc from a node to another with weight and max_speed
*/
int Graph::addArc(std::string from_node_id, std::string to_node_id, float weight, float max_speed)
{
    graph_msgs::GraphArc new_arc;
    new_arc.node_dest = to_node_id;
    new_arc.distance = weight;
    new_arc.max_speed = max_speed;

    dijkstraGraph->addArc(from_node_id, new_arc);

    return 0;
}

/*! \fn int Graph::setArcPos(std::string from_id_old, std::string from_id, std::string to_id_old, std::string to_id)
 * 	\brief Modify the pos of an arc
*/
int Graph::setArcPos(std::string from_id_old, std::string from_id, std::string to_id_old, std::string to_id)
{
    return dijkstraGraph->setArcPos(from_id_old, from_id, to_id_old, to_id);
}

/*! \fn int Graph::setArc(string from_id, graph_msgs::GraphArc arc)
 * 	\brief Modify the pos of an arc
*/
int Graph::setArc(std::string from_id, graph_msgs::GraphArc arc)
{
    return dijkstraGraph->setArc(from_id, arc);
}

/*! \fn std::vector<graph_msgs::GraphNode> Graph::getNodes()
 * 	\brief Get msg with the graph nodes
*/
std::vector<graph_msgs::GraphNode> Graph::getNodes()
{
    std::vector<Node *> nodes = dijkstraGraph->getNodes();

    std::vector<graph_msgs::GraphNode> new_nodes;
    for (int i = 0; i < nodes.size(); i++)
    {
        new_nodes.push_back(nodes[i]->node);
    }

    return new_nodes;
}

/*! \fn graph_msgs::GraphNodeArray Graph::getGraphMsg()
 * 	\brief Get msg with the graph nodes
*/
graph_msgs::GraphNodeArray Graph::getNodesMsg()
{
    std::vector<graph_msgs::GraphNode> nodes_vector = getNodes();

    graph_msgs::GraphNodeArray nodes_msg;
    for (int i = 0; i < nodes_vector.size(); i++)
    {
        nodes_msg.nodes.push_back(nodes_vector[i]);
    }

    return nodes_msg;
}

/*! \fn std::vector<Node *> Graph::GetNodesUsed()
 * 	\brief Gets the list of nodes used
*/
std::vector<graph_msgs::GraphNode> Graph::getNodesUsed()
{
    return dijkstraGraph->getNodesUsed();
}

/*! \fn graph_msgs::GraphNodeArray Graph::GetNodesUsedMsg()
 * 	\brief Get msg with the used nodes
*/
graph_msgs::GraphNodeArray Graph::getNodesUsedMsg()
{
    std::vector<graph_msgs::GraphNode> nodes_vector = dijkstraGraph->getNodesUsed();

    graph_msgs::GraphNodeArray nodes_msg;
    for (int i = 0; i < nodes_vector.size(); i++)
    {
        nodes_msg.nodes.push_back(nodes_vector[i]);
    }

    return nodes_msg;
}

/*! \fn int Graph::ReserveNode(int iRobot, int iIDNode)
 * 	\brief Reserve Node
*/
bool Graph::reserveNode(int iRobot, std::string iIDNode)
{
    return dijkstraGraph->reserveNode(iRobot, iIDNode);
}

/*! \fn int Graph::UnBlockAll(int iRobot)
 * 	\brief UnBlock all nodes
*/
bool Graph::unBlockAll(int iRobot)
{
    return dijkstraGraph->unBlockAll(iRobot);
}

/*! \fn int Graph::getRoute(int from, int to, vector<std::string> *route)
 * 	\brief Gets the list of nodes from initial node "from" to the end node "to"
*/
int Graph::getRoute(std::string from, std::string to, vector<std::string> *route)
{
    return dijkstraGraph->getRoute(from, to, route);
}

/*! \fn int Graph::getRoute(int from, int to, vector<geometry_msgs::Pose2D> *nodes, vector<double> *max_speed){
 * 	\brief Obtiene las coordenadas de los nodos y de los imanes ordenadas para esa trayectoria, además de las velocidades entre dichos nodos
*/
int Graph::getRoute(std::string from, std::string to, vector<geometry_msgs::Pose2D> *nodes, vector<double> *max_speed)
{
    vector<std::string> route;
    int i = 0;
    geometry_msgs::Pose2D pos;

    if (dijkstraGraph->getRoute(from, to, &route) != 0) // Ruta incorrecta
        return -1;

    for (i = 0; i < (int)(route.size() - 1); i++)
    { // Recorremos los nodos de la ruta hasta el penultimo
        if (!getNodePosition(route[i], &pos))
        {
            nodes->push_back(pos);
            graph_msgs::GraphArc arc;
            if (getArcBetweenNodes(route[i], route[i + 1], arc) != 0)
            {
                ROS_ERROR("Graph::getRoute: Error getting the route from %d to %d", route[i], route[i + 1]);
            }
            else
                max_speed->push_back(arc.max_speed);
        }
        else
        {
            ROS_ERROR("Graph::GetRoute: Error getting node %d", route[i]);

            return -1;
        }
    }
    // El último nodo
    if (!getNodePosition(route[i], &pos))
    {
        nodes->push_back(pos);
    }
    else
    {
        ROS_ERROR("Graph::getRoute: Last node: Error getting node %d", route[i]);

        return -1;
    }
    return 0;
}

/*! \fn int Graph::getRoute(int from, int to, vector<graph_msgs::GraphNode> *detailed_nodes, vector<geometry_msgs::Pose2D> *nodes){
 * 	\brief Misma función salvo que también obtiene los nodos de la ruta en detalle, no solamente su posición
*/
int Graph::getRoute(std::string from, std::string to, vector<graph_msgs::GraphNode> *detailed_nodes, vector<geometry_msgs::Pose2D> *nodes, vector<double> *max_speed)
{
    vector<std::string> route;
    int i = 0;
    geometry_msgs::Pose2D pos;

    if (dijkstraGraph->getRoute(from, to, &route) != 0) // Ruta incorrecta
        return -1;

    for (i = 0; i < (int)(route.size() - 1); i++)
    { // Recorremos los nodos de la ruta hasta el penultimo
        if (!getNodePosition(route[i], &pos))
        {
            nodes->push_back(pos);
            graph_msgs::GraphArc arc;
            if (getArcBetweenNodes(route[i], route[i + 1], arc) != 0)
            {
                ROS_ERROR("Graph::getRoute: Error getting the route from %d to %d", route[i], route[i + 1]);
            }
            else
                max_speed->push_back(arc.max_speed);
        }
        else
        {
            ROS_ERROR("Graph::GetRoute: Error getting node %d", route[i]);

            return -1;
        }
    }
    // El último nodo
    if (!getNodePosition(route[i], &pos))
    {
        nodes->push_back(pos);
    }
    else
    {
        ROS_ERROR("Graph::getRoute: Last node: Error getting node %d", route[i]);

        return -1;
    }

    graph_msgs::GraphNode node;
    for (int i; i < route.size(); i++)
    {
        node = getNode(route[i]);
        detailed_nodes->push_back(node);
    }

    return 0;
}

/*! \fn int Graph::getRoute(int from, int to, vector<Node> *detailed_nodes, vector<double> *max_speed){
 * 	\brief Misma función salvo que también obtiene los nodos de la ruta en detalle, no solamente su posición
*/
int Graph::getRoute(std::string from, std::string to, vector<graph_msgs::GraphNode> *detailed_nodes, vector<double> *max_speed)
{
    vector<std::string> route;
    int i = 0;

    if (dijkstraGraph->getRoute(from, to, &route) != 0) // Ruta incorrecta
        return -1;

    for (i = 0; i < (int)(route.size() - 1); i++)
    { // Recorremos los nodos de la ruta hasta el penultimo
        graph_msgs::GraphArc arc;
        if (getArcBetweenNodes(route[i], route[i + 1], arc) != 0)
        {
            ROS_ERROR("Graph::getRoute: Error getting the route from %d to %d", route[i], route[i + 1]);
        }
        else
            max_speed->push_back(arc.max_speed);
    }

    graph_msgs::GraphNode node;
    for (int i; i < route.size(); i++)
    {
        node = getNode(route[i]);
        detailed_nodes->push_back(node);
    }

    return 0;
};

/*! \fn int Graph::getNodePosition(int node_id, geometry_msgs::Pose2D *pos)
 * 	\brief Obtiene la posición del nodo
 *  \return 0 if OK
 *  \return -1 si el nodo no existe
*/
// TODO correct seccond ROS_ERROR (should come from getNodeFromId)
int Graph::getNodePosition(std::string node_id, geometry_msgs::Pose2D *pos)
{
    if (dijkstraGraph->isOnEdition())
    {
        ROS_ERROR("Dijkstra::GetNodePosition: Edition must be disabled");
        return -2;
    }
    else
    {
        graph_msgs::GraphNode node = dijkstraGraph->getNodeFromId(node_id)->node;

        pos->x = node.pose.x;
        pos->y = node.pose.y;
        pos->theta = node.pose.theta;

        return 0;
    }
}

/*! \fn std::string Graph::setNode(graph_msgs::GraphNode)
 * 	\brief Obtiene la posición del nodo
 *  \return 0 if OK
 *  \return -1 si el nodo no existe
*/
std::string Graph::setNode(graph_msgs::GraphNode node_info)
{
    if (dijkstraGraph->isOnEdition())
    {
        ROS_ERROR("Dijkstra::SetNode: Edition must be disabled");
        return "Dijkstra::SetNode: Edition must be disabled";
    }

    Node *node = dijkstraGraph->getNodeFromId(node_info.id);

    if (node == 0)
    {
        ROS_ERROR("Dijkstra::SetNode: node %d does not exist", node_info.id);
        return "Dijkstra::SetNode: node " + node_info.id + " does not exist";
    }
    else if (node_info.id == "")
    {
        ROS_ERROR("Dijkstra::SetNode: Error: The ID cannot be an empty string");
        return "Dijkstra::SetNode: Error: The ID cannot be an empty string";
    }
    else if (node_info.name == "")
    {
        ROS_ERROR("Dijkstra::SetNode: Error: The name cannot be an empty string");
        return "Dijkstra::SetNode: Error: The name cannot be an empty string";
    }
    else
    {
        node->node.name = node_info.name;
        node->node.zone = node_info.zone;
        node->node.pose = node_info.pose;

        return "OK";
    }
}

/*! \fn std::string Graph::deleteNode(int node_id)
 * 	\brief Obtiene la posición del nodo
 *  \return 0 if OK
 *  \return -1 si el nodo no existe
*/
std::string Graph::deleteNode(std::string node_id)
{
    return dijkstraGraph->deleteNode(node_id);
}

/*! \fn std::string Graph::deleteNode(int node_id)
 * 	\brief Obtiene la posición del nodo
 *  \return 0 if OK
 *  \return -1 si el nodo no existe
*/
std::string Graph::deleteArc(std::string from_id, std::string to_id)
{
    return dijkstraGraph->deleteArc(from_id, to_id);
}

/*! \fn int Graph::getArcBetweenNodes(int from_id, int to_id, graph_msgs::GraphArc arc)
 * 	\brief Obtiene el arco entre nodos
 *  \return 0 if OK
 *  \return -1 si el nodo no existe
*/
int Graph::getArcBetweenNodes(std::string from_id, std::string to_id, graph_msgs::GraphArc arc)
{
    if (dijkstraGraph->isOnEdition())
    {
        ROS_ERROR("Dijkstra::GetNodePosition: Edition must be disabled");
        return -2;
    }
    else
    {
        graph_msgs::GraphNode node = dijkstraGraph->getNodeFromId(from_id)->node;
        for (int i = 0; i < node.arc_list.size(); i++)
        {
            if (node.arc_list[i].node_dest == to_id)
            {
                arc = node.arc_list[i];
                return 0;
            }
        }
        ROS_ERROR("Dijkstra::GetNodePosition: route to node %d does not exist", to_id);
        return -1;
    }
}

/*! \fn Node* Dijkstra::CheckNodeFree(int idNode, int idRobot)
 * true if node Free, false if used or reserved by a robot
*/
bool Graph::checkNodeFree(std::string idNode, int idRobot)
{
    return dijkstraGraph->checkNodeFree(idNode, idRobot);
}

/*! \fn Node* Dijkstra::CheckNodesFree(std::vector<int> idNode, int idRobot)
 * true if node Free, false if used or reserved by a robot
*/
bool Graph::checkNodesFree(std::vector<std::string> vNodesId, int idRobot)
{
    bool bfree = true;
    for (int i = 0; i < vNodesId.size(); i++)
    {
        if (!dijkstraGraph->checkNodeFree(vNodesId[i], idRobot))
            bfree = false;
    }
    return bfree;
}

/*! \fn bool Graph::checkZoneFree(int idZone, int idRobot)
 * true if Zone Free, false if used or reserved by a robot
*/
bool Graph::checkZoneFree(int idZone, int idRobot)
{
    return dijkstraGraph->checkZoneFree(idZone, idRobot);
}

/*! \fn graph_msgs::GraphNode Graph::getNodeFromID(int iIDNode)
 * 	\brief Get node by nodeID
*/
graph_msgs::GraphNode Graph::getNodeFromId(std::string iIDNode)
{
    return dijkstraGraph->getNodeFromId(iIDNode)->node;
}

/*! \fn int Graph::getRobotFromId(int iIDNode)
 * 	\brief Get node by nodeID
*/
int Graph::getRobotFromId(std::string iIDNode)
{
    return dijkstraGraph->getNodeFromId(iIDNode)->iRobot;
}

/*! \fn int Graph::getResRobotFromId(std::string iIDNode)
 * 	\brief Get node by nodeID
*/
int Graph::getResRobotFromId(std::string iIDNode)
{
    return dijkstraGraph->getNodeFromId(iIDNode)->iResRobot;
}

/*! \fn graph_msgs::GraphNode Graph::getNode(std::string nodeID)
 * 	\brief Gets Node by nodeID
*/
graph_msgs::GraphNode Graph::getNode(std::string node_id)
{
    for (int i = 0; i < dijkstraGraph->vNodes.size(); i++)
    {
        if (dijkstraGraph->vNodes[i]->node.id == node_id)
            return dijkstraGraph->vNodes[i]->node;
        //ROS_ERROR("Dijkstra::getNode: node %d, pNodes[i]->iNode:%d  ", node_id,vNodes[i].iNode);
    }
    //ROS_ERROR("Dijkstra::getNode: node %d  NULL ", node_id);
    return graph_msgs::GraphNode();
}

/*! \fn int Graph::getNumNodes()
 * 	\brief Get the number of nodes
 *  \return Number of nodes
*/
int Graph::getNumNodes()
{
    return dijkstraGraph->vNodes.size();
}

/*! \fn int Graph::deleteAll()
 * 	\brief Deletes all the components
 *  \return 0 if OK
*/
int Graph::deleteAll()
{
    dijkstraGraph->vNodes.clear();

    ROS_INFO("Dijkstra::deleteAll: all data has been removed");
    return 0;
}

/*! \fn int Graph::serialize()
 * 	\brief Save the graph data on a json document
*/
std::string Graph::serialize()
{
    ros::jsonization::Json jsonized_msg;
    ros::jsonization::jsonize(jsonized_msg, getNodesMsg());

    std::ofstream ofile;
    ofile.open(graph_path_);
    ofile << jsonized_msg.dump(4);
    ofile.close();

    return "OK";
}

/*! \fn int Graph::deserialize()
 * 	\brief Read the graph data reading previouly from a json document
*/
std::string Graph::deserialize()
{
    ROS_INFO("Graph::deserialize File: %s", graph_path_.c_str());

    try
    {
        std::ifstream ifile;
        ifile.open(graph_path_);

        ros::jsonization::Json jsonized_msg;
        jsonized_msg << ifile;

        graph_msgs::GraphNodeArray msg;
        ros::jsonization::dejsonize(jsonized_msg, msg);

        setGraph(msg);
    }
    catch (ros::jsonization::JsonBadTypeException)
    {
        ROS_ERROR("Graph::deserialize Error: Bad structure on file %s", graph_path_.c_str());
    }

    // TODO ROS_INFO("Graph::Deserialize: Processing JSON Found %s Nodes, %s Zones", std::to_string(nodes), std::to_string(zones));
    return "OK";
}
