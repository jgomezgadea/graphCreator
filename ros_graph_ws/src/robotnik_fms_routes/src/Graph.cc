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

/*! \fn Graph::Graph(char *fileName)
 * 	\brief Constructor
*/
Graph::Graph(const char *f)
{
    strcpy(fileName, f);
    dijkstraGraph = new Dijkstra();
    graphData = new graph_msgs::GraphNodeArray();
    bInitialized = false;
    nodes = zones = 0;
}

/*! \fn Graph::~Graph()
 * 	\brief Destructor
*/
Graph::~Graph()
{
    delete dijkstraGraph;
    delete graphData;
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

    //TODO load graph from json

    if (bInitialized)
    {
        return "Graph::setup: Already initialized";
    }
    std::string desmsg = deserialize();
    if (desmsg != "OK")
    {
        return "Graph::setup: Error deserializing the graph:" + desmsg;
    }
    desmsg = dijkstraGraph->finalizeEdition(graphData);
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
        ROS_ERROR("Graph::shutDown: Impossible because of it's not initialized");
        return NOT_INITIALIZED;
    }

    dijkstraGraph->deleteAll();

    bInitialized = false;

    return OK;
}

/*! \fn int Graph::deserialize()
 * 	\brief Process the data reading previouly from a json document
*/
std::string Graph::deserialize()
{
    ROS_INFO("Graph::deserialize File: %s", fileName);

    nodes = 5; // Number of nodes
    zones = 1; // Number of zones

    graphData->nodes.resize(3);

    graphData->nodes[0].arc_list.resize(2);
    graphData->nodes[0].id = 0;
    graphData->nodes[0].zone = 1;
    graphData->nodes[0].pose.frame_id = "map";
    graphData->nodes[0].pose.x = 0;
    graphData->nodes[0].pose.y = 0;
    graphData->nodes[0].pose.z = 0;
    graphData->nodes[0].pose.theta = 0;
    graphData->nodes[0].arc_list[0].node_dest = 1;
    graphData->nodes[0].arc_list[1].node_dest = 2;

    graphData->nodes[1].id = 1;
    graphData->nodes[1].zone = 1;
    graphData->nodes[1].pose.frame_id = "map";
    graphData->nodes[1].pose.x = 1;
    graphData->nodes[1].pose.y = 0;
    graphData->nodes[1].pose.z = 0;
    graphData->nodes[1].pose.theta = 0;

    graphData->nodes[2].id = 2;
    graphData->nodes[2].zone = 1;
    graphData->nodes[2].pose.frame_id = "map";
    graphData->nodes[2].pose.x = 0;
    graphData->nodes[2].pose.y = 1;
    graphData->nodes[2].pose.z = 0;
    graphData->nodes[2].pose.theta = 0;

    //ROS_ERROR("%s", std::to_string(graphData->nodes[0].arc_list[1].node_dest).c_str());

    // TODO ROS_INFO("Graph::Deserialize: Processing JSON Found %s Nodes, %s Zones", std::to_string(nodes), std::to_string(zones));
    return "OK";
}

/*! \fn void Graph::print()
 * 	\brief Print current graph
*/
void Graph::print()
{
    if (graphData->nodes.size() > 0)
    {
        ROS_INFO("");
        ROS_INFO("Graph::print %d nodes:", graphData->nodes.size());
        for (int i = 0; i < graphData->nodes.size(); i++)
        {
            ROS_INFO("Node %i:", graphData->nodes[i].id);
            ROS_INFO("  Name: %s", graphData->nodes[i].name.c_str());
            ROS_INFO("  Zone: %i", graphData->nodes[i].zone);
            ROS_INFO("  Pose:");
            ROS_INFO("    x: %f", graphData->nodes[i].pose.x);
            ROS_INFO("    y: %f", graphData->nodes[i].pose.y);
            ROS_INFO("    z: %f", graphData->nodes[i].pose.z);
            ROS_INFO("    theta: %f", graphData->nodes[i].pose.theta);
            ROS_INFO("    frame_id: %s", graphData->nodes[i].pose.frame_id.c_str());
            printArcs(&graphData->nodes[i]);
        }
    }
    else
        ROS_INFO("The graph has 0 nodes");
    ROS_INFO("");
}

/*! \fn void Graph::printArcs()
 * 	\brief Print arcs of current node
*/
void Graph::printArcs(graph_msgs::GraphNode *node)
{
    if (node->arc_list.size() > 0)
    {
        ROS_INFO("  Arcs:");
        for (int i = 0; i < node->arc_list.size(); i++)
        {
            ROS_INFO("    Arc %i:", i);
            ROS_INFO("      Destination node: %i", node->arc_list[i].node_dest);
            ROS_INFO("      Max speed: %f", node->arc_list[i].max_speed);
            ROS_INFO("      Distance: %f", node->arc_list[i].distance);
        }
    }
    else
        ROS_INFO("  Arcs: Empty");
}

/*! \fn int Graph::getNodes()
 * 	\brief
*/
int Graph::getNodes()
{
    return nodes;
}

/*! \fn int Graph::getRoute(int from, int to, vector<int> *route)
 * 	\brief Gets the list of nodes from initial node "from" to the end node "to"
*/
int Graph::getRoute(int from, int to, vector<int> *route)
{
    return dijkstraGraph->getRoute(from, to, route);
}

/*! \fn int Graph::GetNodesUsed(vector<Node *> *route)
 * 	\brief Gets the list of nodes used
*/
bool Graph::getNodesUsed(std::vector<Node *> *route)
{
    return dijkstraGraph->getNodesUsed(route);
}

/*! \fn int Graph::ReserveNode(int iRobot,int iIDNode)
 * 	\brief Reserve Node
*/
bool Graph::reserveNode(int iRobot, int iIDNode)
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

/*! \fn int Graph::getRoute(int from, int to, vector<geometry_msgs::Pose2D> *nodes){
 * 	\brief Obtiene las coordenadas de los nodos y de los imanes ordenadas para esa trayectoria, además de las velocidades entre dichos nodos
*/
int Graph::getRoute(int from, int to, vector<geometry_msgs::Pose2D> *nodes, vector<double> *speed_between_nodes)
{
    vector<int> route;
    int i = 0;
    geometry_msgs::Pose2D pos;
    double speed = 0.0;

    if (dijkstraGraph->getRoute(from, to, &route) != 0) // Ruta incorrecta
        return -1;

    for (i = 0; i < (int)(route.size() - 1); i++)
    { // Recorremos los nodos de la ruta hasta el penultimo
        if (!dijkstraGraph->getNodePosition(graphData, route[i], &pos.x, &pos.y, &pos.theta))
        {
            nodes->push_back(pos);
            if (dijkstraGraph->getArcBetweenNodes(route[i], route[i + 1]) != 0)
            { // Obtenemos los imanes en la ruta al nodo adyacente
                ROS_ERROR("Graph::getRoute: Error getting the magnets from %d to %d", route[i], route[i + 1]);

                return -1;
            }
            else
            {
                speed_between_nodes->push_back(speed);
            }
        }
        else
        {
            ROS_ERROR("Graph::GetRoute: Error getting node %d", route[i]);

            return -1;
        }
    }
    // El último nodo
    if (!dijkstraGraph->getNodePosition(graphData, route[i], &pos.x, &pos.y, &pos.theta))
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

/*! \fn int Graph::getRoute(int from, int to, vector<Node> *detailed_nodes, vector<geometry_msgs::Pose2D> *nodes){
 * 	\brief Misma función salvo que también obtiene los nodos de la ruta en detalle, no solamente su posición
*/
int Graph::getRoute(int from, int to, vector<Node> *detailed_nodes, vector<geometry_msgs::Pose2D> *nodes, vector<double> *speed_between_nodes)
{

    if (dijkstraGraph->getRoute(from, to, detailed_nodes) != 0) // Ruta incorrecta
        return -1;

    if (getRoute(from, to, nodes, speed_between_nodes) != 0) // Llamamos a la función de obtener ruta normal
        return -1;

    return 0;
}

/*! \fn int Graph::getRoute(int from, int to, vector<Node> *detailed_nodes, vector<double> *speed_between_nodes){
 * 	\brief Misma función salvo que también obtiene los nodos de la ruta en detalle, no solamente su posición
*/
int Graph::getRoute(int from, int to, vector<Node> *detailed_nodes, vector<double> *speed_between_nodes)
{
    vector<int> route;
    int i = 0;
    geometry_msgs::Pose2D pos;
    double speed = 0.0;

    if (dijkstraGraph->getRoute(from, to, detailed_nodes) != 0) // Ruta incorrecta
        return -1;

    for (i = 0; i < (int)(detailed_nodes->size() - 1); i++)
    { // Recorremos los nodos de la ruta hasta el penultimo
        if (!dijkstraGraph->getNodePosition(graphData, (*detailed_nodes)[i].iNode, &pos.x, &pos.y, &pos.theta))
        {
            if (dijkstraGraph->getArcBetweenNodes((*detailed_nodes)[i].iNode, (*detailed_nodes)[i + 1].iNode) != 0)
            { // Obtenemos los imanes en la ruta al nodo adyacente
                ROS_ERROR("Graph::getRoute: Error getting the magnets from %d to %d", (*detailed_nodes)[i].iNode, (*detailed_nodes)[i + 1].iNode);
                return -1;
            }
            else
            {
                speed_between_nodes->push_back(speed);
            }
        }
        else
        {
            ROS_ERROR("Graph::getRoute: Error getting node %d", route[i]);
            return -1;
        }
    }
    return 0;
};

/*! \fn int Graph::getNodePosition(int num_node, geometry_msgs::Pose2D *pos)
 * 	\brief Obtiene la posición del nodo
 *  \return 0 if OK
 *  \return -1 si el nodo no existe
*/
int Graph::getNodePosition(int num_node, geometry_msgs::Pose2D *pos)
{
    double x = 0.0, y = 0.0, a = 0.0;
    if (!dijkstraGraph->getNodePosition(graphData, num_node, &x, &y, &a))
    {
        pos->x = x;
        pos->y = y;
        pos->theta = a;
        return 0;
    }
    return -1;
}

/*! \fn Node* Dijkstra::CheckNodeFree(int idNode, int idRobot)
 * true if node Free, false if used or reserved by a robot
*/
bool Graph::checkNodeFree(int idNode, int idRobot)
{
    return dijkstraGraph->checkNodeFree(idNode, idRobot);
}

/*! \fn Node* Dijkstra::CheckNodesFree(std::vector<int> idNode, int idRobot)
 * true if node Free, false if used or reserved by a robot
*/
bool Graph::checkNodesFree(std::vector<int> vNodesId, int idRobot)
{
    bool bfree = true;
    for (int i = 0; i < vNodesId.size(); i++)
    {
        if (!dijkstraGraph->checkNodeFree(vNodesId[i], idRobot))
            bfree = false;
    }
    return bfree;
}

/*! \fn Node* Dijkstra::CheckZoneFree(int idZone, int idRobot)
 * true if Zone Free, false if used or reserved by a robot
*/
bool Graph::checkZoneFree(int idZone, int idRobot)
{
    return dijkstraGraph->checkZoneFree(idZone, idRobot);
}

/*! \fn Node* Dijkstra::getNode(unsigned int nodeID)
 * 	\brief Gets Node by nodeID
*/
Node *Graph::getNode(unsigned int node_id)
{
    return dijkstraGraph->getNode(node_id);
}