/** \file Graph.h
 * \author Robotnik Automation S.L.L.
 * \version 1.1
 * \date    2011
 * \brief Class for managing the nodes, magnets and routes of the system
 * (C) 2011 Robotnik Automation, SLL. All rights reserved
*/

#ifndef __GRAPH_H
#define __GRAPH_H

#include <iostream>
#include <string>

#include <robotnik_fms_routes/Dijkstra.h>
#include <geometry_msgs/Pose2D.h>
#include <graph_msgs/GraphNodeArray.h>

//XERCES_CPP_NAMESPACE_USE
using namespace std;

//! Defines return values for methods and functions
enum ReturnValue
{
	OK = 0,
	INITIALIZED,
	THREAD_RUNNING,
	ERROR = -1,
	NOT_INITIALIZED = -2,
	THREAD_NOT_RUNNING = -3,
	COM_ERROR = -4,
	NOT_ERROR = -5
};

class Graph
{

private:
	//! Nombre del fichero json
	char fileName[128];
	//! Controls if has been initialized succesfully
	bool bInitialized;

public:
	//! Grafo con los nodos y arcos del sistema
	Dijkstra *dijkstraGraph;

public:
	//! Constructor
	Graph(const char *xmlFile);
	//! Destructor
	~Graph();
	//!
	std::string setup();
	//!
	int shutDown();
	//! Print nodes of current graph
	void printNodes();
	//! Print zones of current graph
	void printZones();
	//! Adds a new node
	int addNode(graph_msgs::GraphNode node);
	//! Adds a new node
	graph_msgs::GraphNode addNode(int node, int zone, double x, double y, double z, double theta, std::string frame, std::string name);
	//! Adds arc from a node to another with constant weight
	int addArc(int from_node_id, int to_node_id);
	//! Adds arc from a node to another with weight
	int addArc(int from_node_id, int to_node_id, float weight);
	//! Adds arc from a node to another with weight and max speed
	int addArc(int from_node_id, int to_node_id, float weight, float max_speed);

	/* TODO
	//! Delete all the nodes
	int deleteNodes();
	//! Delete arc
	int deleteArc(int from_node, int to_node);
	//! Delete all arcs of a node
	int deleteArcs(int from_node);
	*/
	//! Obtiene las coordenadas de los nodos ordenadas para esa trayectoria, además de las velocidades entre dichos nodos
	int getRoute(int from, int to, vector<geometry_msgs::Pose2D> *nodes, vector<double> *max_speed);
	//! Misma función salvo que obtiene los nodos de la ruta en detalle, no solamente su posición
	int getRoute(int from, int to, vector<graph_msgs::GraphNode> *detailed_nodes, vector<geometry_msgs::Pose2D> *nodes, vector<double> *max_speed);
	//! Obtiene los nodos por los que pasa la ruta que va desde el nodo "from" al nodo "to"
	int getRoute(int from, int to, vector<int> *route);
	//! Obtiene los nodos de la ruta de forma detallada
	int getRoute(int from, int to, vector<graph_msgs::GraphNode> *detailed_nodes, vector<double> *max_speed);
	//! Obtiene la posición del nodo indicado
	int getNodePosition(int num_node, geometry_msgs::Pose2D *pos);
	//! Obtiene el arco entre dos nodos
	int getArcBetweenNodes(int from_id, int to_id, graph_msgs::GraphArc arc);
	//! Get list of the graph nodes
	std::vector<graph_msgs::GraphNode> getNodes();
	//! Get msg with the graph nodes
	graph_msgs::GraphNodeArray getNodesMsg();
	//! Get list of used nodes
	std::vector<graph_msgs::GraphNode> getNodesUsed();
	//! Get msg with the used nodes
	graph_msgs::GraphNodeArray getNodesUsedMsg();
	//! Reserve a node
	bool reserveNode(int iRobot, int iIDNode);
	//! Unblock all nodes
	bool unBlockAll(int iRobot);
	//! Get the number of nodes
	int getNumNodes();
	//! Deletes all nodes
	int deleteAll();

	//! Gets Node by nodeID
	graph_msgs::GraphNode getNode(unsigned int node_id);

	bool checkNodeFree(int idNode, int idRobot);
	bool checkNodesFree(std::vector<int> vNodesId, int idRobot);
	bool checkZoneFree(int idZone, int idRobot);

	//! Get Node From ID
	graph_msgs::GraphNode getNodeFromId(int iIDNode);

	//! Get iRobot from ID
	int getRobotFromId(int iIDNode);
	//! Set iRobot from ID
	bool setRobotById(int iIDNode);
	//! Get iResRobot from ID
	int getResRobotFromId(int iIDNode);

private:
	//! Serializes a json file, saving the graph
	std::string serialize();
	//! Deserializes a json file, extracting the graph
	std::string deserialize();
};

#endif
