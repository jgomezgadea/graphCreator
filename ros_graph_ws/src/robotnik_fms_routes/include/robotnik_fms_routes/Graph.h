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
#include <fstream>
#include <string>

#include <robotnik_fms_routes/Dijkstra.h>
#include <geometry_msgs/Pose2D.h>
#include <graph_msgs/GraphNodeArray.h>

// JSONIZATION
#include <jsonization/json.hpp>
#include <jsonization/jsonization.h>

#include <graph_msgs/GraphNodeArray.json.h>

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
	//! Path of the json file
	std::string graph_path_;
	//! Controls if has been initialized succesfully
	bool bInitialized;

public:
	//! Grafo con los nodos y arcos del sistema
	Dijkstra *dijkstraGraph;

public:
	//! Constructor
	Graph(std::string graph_path);
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
	int setGraph(graph_msgs::GraphNodeArray graph);
	//! Adds a new node
	std::string addNode(graph_msgs::GraphNode node);
	//! Adds a new node
	graph_msgs::GraphNode addNode(std::string node, int zone, double x, double y, double z, double theta, std::string frame, std::string name);
	//! Adds arc from a node to another with constant weight
	int addArc(std::string from_node_id, std::string to_node_id);
	//! Adds arc from a node to another with weight
	int addArc(std::string from_node_id, std::string to_node_id, float weight);
	//! Adds arc from a node to another with weight and max speed
	int addArc(std::string from_node_id, std::string to_node_id, float weight, float max_speed);

	/* TODO
	//! Delete all the nodes
	int deleteNodes();
	//! Delete arc
	int deleteArc(int from_node, int to_node);
	//! Delete all arcs of a node
	int deleteArcs(int from_node);
	return vNodes[from_node]->deleteAdjacent(to_node);
	*/
	//! Obtiene las coordenadas de los nodos ordenadas para esa trayectoria, además de las velocidades entre dichos nodos
	int getRoute(std::string from, std::string to, vector<geometry_msgs::Pose2D> *nodes, vector<double> *max_speed);
	//! Misma función salvo que obtiene los nodos de la ruta en detalle, no solamente su posición
	int getRoute(std::string from, std::string to, vector<graph_msgs::GraphNode> *detailed_nodes, vector<geometry_msgs::Pose2D> *nodes, vector<double> *max_speed);
	//! Obtiene los nodos por los que pasa la ruta que va desde el nodo "from" al nodo "to"
	int getRoute(std::string from, std::string to, vector<std::string> *route);
	//! Obtiene los nodos de la ruta de forma detallada
	int getRoute(std::string from, std::string to, vector<graph_msgs::GraphNode> *detailed_nodes, vector<double> *max_speed);
	//! Obtiene la posición del nodo indicado
	int getNodePosition(std::string node_id, geometry_msgs::Pose2D *pos);
	//! Modifica la posición del nodo indicado
	std::string setNodePosition(std::string node_id, graph_msgs::GraphNodePose pos);
	//! Elimina el nodo indicado
	std::string deleteNode(std::string node_id);
	//! Obtiene el arco entre dos nodos
	int getArcBetweenNodes(std::string from_id, std::string to_id, graph_msgs::GraphArc arc);
	//! Get list of the graph nodes
	std::vector<graph_msgs::GraphNode> getNodes();
	//! Get msg with the graph nodes
	graph_msgs::GraphNodeArray getNodesMsg();
	//! Get list of used nodes
	std::vector<graph_msgs::GraphNode> getNodesUsed();
	//! Get msg with the used nodes
	graph_msgs::GraphNodeArray getNodesUsedMsg();
	//! Reserve a node
	bool reserveNode(int iRobot, std::string iIDNode);
	//! Unblock all nodes
	bool unBlockAll(int iRobot);
	//! Get the number of nodes
	int getNumNodes();
	//! Deletes all nodes
	int deleteAll();

	//! Gets Node by nodeID
	graph_msgs::GraphNode getNode(std::string node_id);

	bool checkNodeFree(std::string idNode, int idRobot);
	bool checkNodesFree(std::vector<std::string> vNodesId, int idRobot);
	bool checkZoneFree(int idZone, int idRobot);

	//! Get Node From ID
	graph_msgs::GraphNode getNodeFromId(std::string iIDNode);

	//! Get iRobot from ID
	int getRobotFromId(std::string iIDNode);
	//! Get iResRobot from ID
	int getResRobotFromId(std::string iIDNode);

private:
	//! Serializes a json file, saving the graph
	std::string serialize();
	//! Deserializes a json file, extracting the graph
	std::string deserialize();
};

#endif
