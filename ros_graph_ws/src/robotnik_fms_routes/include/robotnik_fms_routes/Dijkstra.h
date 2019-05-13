/** \file Dijkstra.h
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2010
 *
 * \brief Dijkstra header
 * Implementa el algoritmo de Dijkstra
 * (C) 2010 Robotnik Automation, SLL
*/

#include <cstdlib>
#include <iostream>
#include <queue>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <graph_msgs/GraphNodeArray.h>

using namespace std;

#ifndef __DIJKSTRA_H
#define __DIJKSTRA_H

#define INFINITE 99999999
#define NO_PARENT -99999

#define MAX_STRING_LENGTH 300

//! Class Node utilizada por Dijkstra para representar los nodos del grafo
class Node
{
public:
	//! Node
	graph_msgs::GraphNode node;
	//! Distance from previous node
	int iDist;
	//! Id from previous node
	int iParent;
	//! Flag para marcar cuando se utiliza el nodo para el cálculo de
	// distancias (solo se utiliza una vez), para no volverlo a encolar
	bool bUsed;
	//! vector de Zonas
	std::vector<int> viZones;
	//! iRobot
	int iRobot;
	//! iRobot Reserved
	int iResRobot;

public:
	//! Constructor
	Node()
	{
		node = graph_msgs::GraphNode();
		node.id = -1;
		iDist = INFINITE;
		iParent = NO_PARENT;
		bUsed = false;
		iRobot = -1;
		iResRobot = -1;
	}
	//! Constructor
	Node(graph_msgs::GraphNode new_node)
	{
		node = new_node;
		iDist = INFINITE;
		iParent = NO_PARENT;
		bUsed = false;
		iRobot = -1;
		iResRobot = -1;
	}
	//! Destructor
	~Node()
	{
		//delete log;
		//std::cout << "Node::~Node:" << std::endl;
	}
	//! Adds new node adjacent with default weight and a list of magnets in the way
	//!	\returns 0 if OK
	int addNodeAdjacent(graph_msgs::GraphArc arc)
	{
		int size = node.arc_list.size();

		if (size > 0)
		{
			for (int i = 0; i < size; i++)
			{ //Comprobamos que no esté repetido
				if (node.arc_list[i].node_dest == arc.node_dest)
				{ // Si está repetido no lo insertamos
					//std::cout << "Node::AddNodeAdjacent: Error: node " << node_id << " already adjacent" << std::endl;
					return 1;
				}
			}
		}
		//
		// Añadimos el arco
		node.arc_list.push_back(arc);
		return 0;
	}
	//! Resets values for new routes
	void reset()
	{
		iDist = INFINITE;
		iParent = NO_PARENT;
		bUsed = false;
	}
	//! Format the node as initial
	void setInitial()
	{
		iDist = 0;
		iParent = NO_PARENT;
		bUsed = false;
	}
	//! Sets the parent's node in the route
	void setParent(int parent)
	{
		iParent = parent;
	}
	//! Deletes selected adjacent node
	//!	\returns 0 if OK
	int deleteAdjacent(int node_id)
	{
		int size = node.arc_list.size();
		if (size > 0)
		{
			for (int i = 0; i < size; i++)
			{
				if (node.arc_list[i].node_dest == node_id)
				{ //Encontrado
					node.arc_list.erase(node.arc_list.begin() + i);
					return 0;
				}
			}
		}
		return -1;
	}
	//! Deletes all adjacent nodes
	int deleteAdjacent()
	{
		node.arc_list.clear();

		return 0;
	}
	//! Returns if the node has been used in the algorithm
	bool isUsed()
	{
		return bUsed;
	}
	//! Sets the node as used
	void setUsed()
	{
		bUsed = true;
	}
	//! Returns the current distance stored in the node
	int getDistance()
	{
		return iDist;
	}
	//! Sets the value of the distance
	void setDistance(int distance)
	{
		iDist = distance;
	}
	//! returns the value of the parent
	int getParent()
	{
		return iParent;
	}
	//! returns the value of the id
	int getId()
	{
		return node.id;
	}
	//! returns the name of the node
	std::string getName()
	{
		return node.name;
	}
	//! Print node
	void print()
	{
		ROS_INFO("Node %i:", node.id);
		ROS_INFO("  Name: %s", node.name.c_str());
		ROS_INFO("  Zone: %i", node.zone);
		ROS_INFO("  iRobot %i:", iRobot);
		ROS_INFO("  iResRobot %i:", iResRobot);
		ROS_INFO("  Pose:");
		ROS_INFO("    x: %f", node.pose.x);
		ROS_INFO("    y: %f", node.pose.y);
		ROS_INFO("    z: %f", node.pose.z);
		ROS_INFO("    theta: %f", node.pose.theta);
		ROS_INFO("    frame_id: %s", node.pose.frame_id.c_str());
		printArcs(node);
	}
	//! Print node arcs
	void printArcs(graph_msgs::GraphNode node)
	{
		if (node.arc_list.size() > 0)
		{
			ROS_INFO("  Arcs:");
			for (int i = 0; i < node.arc_list.size(); i++)
			{
				ROS_INFO("    Arc %i:", i);
				ROS_INFO("      Destination node: %i", node.arc_list[i].node_dest);
				ROS_INFO("      Max speed: %f", node.arc_list[i].max_speed);
				ROS_INFO("      Distance: %f", node.arc_list[i].distance);
			}
		}
		else
			ROS_INFO("  Arcs: Empty");
	}
};

//! Class Dijkstra
//! El algoritmo de Dijkstra, también llamado algoritmo de caminos mínimos,
//! es un algoritmo para la determinación del camino más corto dado un vértice origen al resto de vértices en un grafo
//! dirigido y con pesos en cada arista
//! Clases utilizadas y su estructura:
//!		- Dijkstra			-> Clase principal
//!			- Node			-> Nodos del grafo
//!				- Arc		-> Arcos que salen de cada nodo
//!			- PointerNode	-> Puntero a nodo. Se utilizará para su utilización en una cola de prioridad
//!			- Route			-> Contiene la lista de todas las rutas calculadas hasta el momento, con ello evitaremos tener que recalcular
//!							   una misma ruta varias veces.
//!				- NodesRoute-> Lista de nodos de una ruta (Ej. para ir de A->B, lista de todos los nodos intermedios A->E->F->R->B)
//!			- NodeComparison-> Clase utilizada para hacer la comparación entre nodos, cada vez que insertamos uno en la cola de prioridad
class Dijkstra
{

	//! Class Route, contiene (puede llegar a contener, puesto que se calcula conforme va haciendo falta) la ruta óptima
	//!	entre todos los nodos
	class Route
	{
		//! Class NodesRoute
		//! Clase que contiene la lista de nodos de una ruta calculada
		class NodesRoute
		{
		public:
			//! Array con los nodos de esa ruta
			std::vector<int> vListNodes;
			//! Vector con los atributos detallados de cada nodo de la ruta
			std::vector<Node *> vListPointerNode;

		public:
			//! Public constructor
			NodesRoute() {}
			//! Public destructor
			~NodesRoute()
			{
				//	std::cout << "NodesRoute::~NodesRoute:" << std::endl;
			}
		};

		//! Clase que contiene la lista de nodos de una ruta
	private:
		//! Matriz para almacenar todas las posibles combinaciones
		NodesRoute **iRoute;
		//! Número de nodos del grafo
		int nodes;

	public:
		//! Public Constructor
		Route(int num_nodes)
		{
			nodes = num_nodes;
			//ROS_INFO("Route::Route: nodes = %i", nodes);
			iRoute = new NodesRoute *[nodes];
			for (int i = 0; i < nodes; i++)
			{
				iRoute[i] = new NodesRoute[nodes];
			}
		}
		//! destructor
		~Route()
		{

			for (int i = 0; i < nodes; i++)
				delete[] iRoute[i];

			delete[] iRoute;
		}
		//! Añade un nodo en la ruta para llegar al nodo objetivo "to_node"
		//! La ruta se añade de manera inversa, por lo que insertamos elementos siempre al principio del vector
		int addNode(int from_node, int to_node, int node)
		{
			if ((from_node < 0) || (from_node > nodes - 1) || (to_node < 0) || (to_node > nodes - 1))
			{
				return -1;
			}
			std::vector<int>::iterator it;

			it = iRoute[from_node][to_node].vListNodes.begin();
			it = iRoute[from_node][to_node].vListNodes.insert(it, node);
			//std::cout << "Route::AddNode: Added from " << from_node << " to " << to_node << ": " << node << std::endl;
			return 0;
		}
		//! Añade un nodo en la ruta para llegar al nodo objetivo "to_node"
		//! La ruta se añade de manera inversa, por lo que insertamos elementos siempre al principio del vector
		int addNode(int from_node, int to_node, Node *node)
		{
			if ((from_node < 0) || (from_node > nodes - 1) || (to_node < 0) || (to_node > nodes - 1))
			{
				return -1;
			}
			std::vector<Node *>::iterator it;

			it = iRoute[from_node][to_node].vListPointerNode.begin();
			it = iRoute[from_node][to_node].vListPointerNode.insert(it, node);
			//std::cout << "Route::AddNode: Added from " << from_node << " to " << to_node << ": " << node << std::endl;
			return 0;
		}
		//! Devuelve la ruta deseada
		std::vector<int> getRoute(int from_node, int to_node)
		{
			if ((from_node < 0) || (from_node > nodes - 1) || (to_node < 0) || (to_node > nodes - 1))
			{
				std::vector<int> aux;
				return aux;
			}

			return iRoute[from_node][to_node].vListNodes;
		}
		std::vector<Node *> getDetailRoute(int from_node, int to_node)
		{
			if ((from_node < 0) || (from_node > nodes - 1) || (to_node < 0) || (to_node > nodes - 1))
			{
				std::vector<Node *> aux;
				return aux;
			}
			//cout << "GetDetailedRoute. Size = " << iRoute[from_node][to_node].vListPointerNode.size() << endl;
			return iRoute[from_node][to_node].vListPointerNode;
		}

		//! Consulta si existe la ruta
		bool existRoute(int from_node, int to_node)
		{
			if ((from_node < 0) || (from_node > nodes - 1) || (to_node < 0) || (to_node > nodes - 1))
			{
				return false;
			}
			if (iRoute[from_node][to_node].vListNodes.size() > 0)
				return true;
			else
				return false;
		}
		//! Imprime la ruta
		void print(int from_node, int to_node)
		{
			if (existRoute(from_node, to_node))
			{
				int size = iRoute[from_node][to_node].vListNodes.size();
				int i = 0;
				std::cout << "Route::Print: Route from " << from_node << " to " << to_node << ": " << size << " nodes" << std::endl
						  << "\t";
				for (i = 0; i < size - 1; i++)
				{
					std::cout << iRoute[from_node][to_node].vListNodes[i] << " -> ";
				}
				std::cout << iRoute[from_node][to_node].vListNodes[i] << std::endl;
			}
			else
				std::cout << "Route::Print: This route doesn't exist" << std::endl;
		}

		//! Devuelve el número de rutas
		int getNumOfRoutes()
		{
			return nodes;
		}

		//! Deletes the selected route
		int deleteRoute(int from_node, int to_node)
		{
			if ((from_node < 0) || (from_node > nodes - 1) || (to_node < 0) || (to_node > nodes - 1))
			{
				//std::cout << "Route::DeleteRoute: This route doesn't exist: " << from_node << " -> " << to_node << std::endl;
				return -1;
			}
			iRoute[from_node][to_node].vListNodes.clear();		 //Limpiamos la ruta
			iRoute[from_node][to_node].vListPointerNode.clear(); //Limpiamos la ruta
			return 0;
		}

		//! Deletes all the routes
		void reset()
		{
			for (int i = 0; i < nodes; i++)
			{
				for (int j = 0; j < nodes; j++)
				{
					deleteRoute(i, j);
				}
			}
		}
	};

	//! Class Zone
	class Zone
	{
	public:
		//! ID Zone
		int iIDZone;
		//! Max Robots in Zone
		int iMaxRobots;
		//! Vector Lista Nodos que contiene la zona
		std::vector<Node *> vpNodes;

		Zone(int iZ)
		{
			iIDZone = iZ;
			iMaxRobots = 1;
		}

		Zone(int iZ, int iMaxR)
		{
			iIDZone = iZ;
			iMaxRobots = iMaxR;
		}

		~Zone()
		{
		}

		//! Adds a new Zone
		//! return -1 if the node already exists
		//! return 0 if OK
		int addNodeToZone(Node *pNode)
		{
			for (int i = 0; i < (int)vpNodes.size(); i++)
			{
				if (vpNodes[i]->node.id == pNode->node.id)
				{
					return -1;
				}
			}
			vpNodes.push_back(pNode);
			return 0;
		}
	};

	//! Class PointerNode se utiliza para poder meter las referencias de los nodos en una cola con prioridad
	class PointerNode
	{
	public:
		Node *node;

	public:
		PointerNode(Node *new_node)
		{
			node = new_node;
		}
		PointerNode()
		{
			node = NULL;
		}
	};

	//! Clase utilizada para hacer la comparación entre nodos, cada vez que insertamos uno en la cola de prioridad
	class NodeComparison
	{
	public:
		NodeComparison() {}
		bool operator()(const PointerNode &lhs, const PointerNode &rhs) const
		{ //Utiliza punteros a nodo, puesto que la distancia del nodo puede ser modificada una vez introducida en la cola
			//std::cout << "NodeComparison: (" << lhs.node->GetId() << ")" << lhs.node->iDist << " > ("<< rhs.node->GetId() << ") " << rhs.node->iDist << "?" << std::endl;
			return (lhs.node->iDist > rhs.node->iDist);
		}
	};

private:
	//! Controls if it's possible to edit the graph
	bool bEdit;
	//!	Objeto de tipo Route con todas las rutas posibles
	Route *rRoutes;
	//! Cola de prioridad necesaria para el algoritmo
	std::priority_queue<PointerNode, std::deque<PointerNode>, NodeComparison> pqQueue;
	//! max value of a node id
	int iMaxNodeId;

public:
	//! Vector con los nodos del grafo
	std::vector<Node *> vNodes;

	//! Zones Vector
	std::vector<Zone> vZones;

public:
	//! Public constructor
	Dijkstra();
	//! Public Destructor
	~Dijkstra();
	//! Disables the edition of nodes and arcs. Necesario para poder calcular rutas sin incoherencias
	std::string finalizeEdition(std::vector<Node *> graph);
	//! Enable the edition of nodes and arcs. Necesario para poder añadir nodos y aristas. Todos los cálculos realizados se perderán
	void enableEdition();
	//! Returns if the graph is being edited
	bool isOnEdition();
	//! Reset the calculated route
	int resetRoutes();
	//! Deletes All the nodes
	int deleteNodes();
	//! Delete the node_id node
	std::string deleteNode(int node_id);
	//! Deletes selected edge
	int deleteArc(int from_node, int to_node);
	//! Deletes all the edge from the node
	int deleteArcs(int from_node);
	//! Gets the optimum calculated route between selected nodes
	int getRoute(int inital_node, int end_node, std::vector<int> *route);
	int getRoute(int inital_node, int end_node, std::vector<Node *> *route);
	//! Controls if the graph is being edited
	void setEditable(bool editable);
	//! Get list of graph nodes
	std::vector<Node *> getNodes();
	//! Get list of nodes used or blocked
	std::vector<graph_msgs::GraphNode> getNodesUsed();

	bool reserveNode(int iRobot, int iIDNode);

	//! GetNearestNode
	int getNearestNodeID(double x, double y, string frame);

	//! bool unBlockAll(int iRobot)
	bool unBlockAll(int iRobot);

	//! Adds a new node
	std::string addNode(graph_msgs::GraphNode node);

	//! Adds edge from a node to another
	int addArc(int from_node, graph_msgs::GraphArc new_arc);
	//! Gets the arc between two nodes
	int getArcBetweenNodes(int from_node, int to_node);
	//! Gets the index of the node using his ID
	int getNodeIndex(int nodeID);
	//! Add Node to Zone
	int addNodeToZone(int iIDNode, int iIDZone);
	//! Add Zone to Graph
	int addZone(int iIDZone, int iMaxRobots);

	bool checkNodeFree(int iIDNode, int iIDRobot);
	bool checkZoneFree(int iIDZone, int iIDRobot);

	//! Get Node From ID
	Node *getNodeFromId(int iIDNode);

	//! Print nodes of current graph
	void printNodes();
	//! Print zones of current graph
	void printZones();
	//! Print node arcs
	void printArcs(graph_msgs::GraphNode node);

private:
	//! Set the initial node
	int setInitialNode(int node);
	//! Reset the value of the nodes
	int resetNodes();
	//! Calculates the optimum route
	int calculateRoute(int initial_node, int end_node);
	//! Resets the priority queue
	int resetQueue();
	//! Push a node into the queue
	int pushIntoQueue(int node);
	//! Pops the node with more priority
	PointerNode popFromQueue();
	//! Queues all the nodes
	int queueNodes();
	//! Updates the queue after modifiying values of the nodes.
	void updateQueue();
};

#endif
