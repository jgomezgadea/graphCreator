/** \file Dijkstra.h
 * \author Robotnik Automation S.L.L.
 * \version 1.4
 * \date    2010
 *
 * \brief Dijkstra header
 * Implementa el algoritmo de Dijkstra
 * 1.3 - En esta versión los nodos se acceden no por posicion sino por Id, a parte de que tendrá nuevos campos
 * 1.4 - NodeRoute almacenará a parte de los Ids de nodo, los propios objetos de nodo, para poder extraer toda la información correspondiente
 * (C) 2010 Robotnik Automation, SLL
*/

#include <cstdlib>
#include <iostream>
#include <queue>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace std;

#ifndef __DIJKSTRA_H
#define __DIJKSTRA_H

#define INFINITE 99999999
#define NO_PARENT -99999

#define MAX_STRING_LENGTH 300

//! Class Node utilizada por Dijstra para representar los nodos del grafo
class Node
{

	//! Class Arc utilizada por Node para indicar el peso y al nodo al que está conectado
	class Arc
	{
	  public:
		//! Peso de la arista
		int iWeight;
		//! Nodo con el conecta la arista
		int iNextNode;
		//! velocidad máxima en ese arco (m/s)
		float dSpeed;

	  public:
		//! Constructor
		Arc(int next_node, double speed)
		{
			iWeight = 1000; //Peso por defecto
			iNextNode = next_node;
			dSpeed = speed; //Velocidad lenta por defecto
		};
		//! Constructor
		Arc(int next_node, int weight, double speed)
		{
			iWeight = weight; //Peso asignado
			iNextNode = next_node;
			dSpeed = speed;
		};
		//! Destructor
		~Arc(){
			//std::cout << "Arc::~Arc:" << std::endl;
		};
		//! Establece la velocidad en ese arco
		int setSpeed(double value)
		{
			dSpeed = value;
			return 0;
		}
		//! Gets the node connected
		int getNode()
		{
			return iNextNode;
		}
		//! Gets the weight of the arc
		int getWeight()
		{
			return iWeight;
		}
	};

  public:
	//! Identificador/Número de nodo
	int iNode;
	//! Distancia al nodo
	int iDist;
	//! Nodo antecesor en la ruta
	int iParent;
	//! Flag para marcar cuando se utiliza el nodo para el cálculo de
	// distancias (solo se utiliza una vez), para no volverlo a encolar
	bool bUsed;
	//! vector de arcos a nodos adyacentes
	std::vector<Arc> vAdjacent;
	//! Coordenadas del nodo
	double dX, dY, dZ;
	//! Theta
	double dTheta;
	//! Frame
	std::string sFrame_id;
	//! Nombre del nodo
	char cName[MAX_STRING_LENGTH];

  public:
	//! Constructor
	Node(int node, double x, double y, double z, double dThet, std::string sFram,
		 char *name)
	{

		iNode = node;
		iParent = NO_PARENT;

		dX = x;
		dY = y;
		dZ = z;

		dTheta = dThet;

		sFrame_id = sFram;

		//strcpy(cName, name); Esto puede tener un overflow tremendo
		strncpy(cName, name, MAX_STRING_LENGTH); // MAX_STRING_LENGTH es el tamaño reservado para cName, asi que se copian como mucho esos caracteres
		cName[MAX_STRING_LENGTH - 1] = 0;		 //y ademas, por si acaso, asignamos NULL al final
	}
	//! Destructor
	~Node()
	{
		//delete log;
		//std::cout << "Node::~Node:" << std::endl;
	}
	//! Sets the node's coordinates
	void setPosition(double x, double y, double z, double theta, std::string sFram)
	{
		dX = x;
		dY = y;
		dZ = z;
		dTheta = theta;
		sFrame_id = sFram;
	}
	//! Sets the node's name
	void setName(char *name)
	{
		//strcpy(cName, name); Esto puede tener un overflow tremendo
		strncpy(cName, name, MAX_STRING_LENGTH); // MAX_STRING_LENGTH es el tamaño reservado para cName, asi que se copian como mucho esos caracteres
		cName[MAX_STRING_LENGTH - 1] = 0;		 //y ademas, por si acaso, asignamos NULL al final
	}
	//! Adds new node adjacent with default weight
	//!	\returns 0 if OK
	int addNodeAdjacent(int node_id, double speed)
	{
		int size = vAdjacent.size();

		if (size > 0)
		{
			for (int i = 0; i < size; i++)
			{ // Comprobamos que no esté repetido
				if (vAdjacent[i].iNextNode == node_id)
				{ // Si está repetido no lo insertamos

					//std::cout << "Node::AddNodeAdjacent: Error: node " << node_id << " already adjacent" << std::endl;
					return -1;
				}
			}
		}

		// Añadimos el arco
		Arc new_arc(node_id, speed);
		vAdjacent.push_back(new_arc);
		return 0;
	}
	//! Adds new node adjacent with selected weight
	//!	\returns 0 if OK
	int addNodeAdjacent(int node_id, double speed, int weight)
	{
		int size = vAdjacent.size();

		if (size > 0)
		{
			for (int i = 0; i < size; i++)
			{ //Comprobamos que no esté repetido
				if (vAdjacent[i].iNextNode == node_id)
				{ //Si está repetido no lo insertamos
					//std::cout << "Node::AddNodeAdjacent: Error: node " << node_id << " already adjacent" << std::endl;

					return -1;
				}
			}
		}
		Arc new_arc(node_id, abs(weight), speed); //Le pasamos el peso en valor absoluto, para evitarnos comprobaciones
		vAdjacent.push_back(new_arc);
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
		int size = vAdjacent.size();
		if (size > 0)
		{
			for (int i = 0; i < size; i++)
			{
				if (vAdjacent[i].iNextNode == node_id)
				{ //Encontrado
					vAdjacent.erase(vAdjacent.begin() + i);
					return 0;
				}
			}
		}

		//std::cout << "Node::AddNodeAdjacent: Error: node " << node_id << " not adjacent" << std::endl;
		return -1;
	}
	//! Deletes all adjacent nodes
	int deleteAdjacent()
	{
		vAdjacent.clear();

		return 0;
	}
	//! Prints current arcs
	void printArcs()
	{
		int size = vAdjacent.size();

		if (size > 0)
		{
			std::cout << "\t Node::PrintArcs: " << size << " arcs" << std::endl;
			for (int i = 0; i < size; i++)
			{
				std::cout << "\t\tArc " << i << ": Weight = " << vAdjacent[i].iWeight << ", Point to " << vAdjacent[i].iNextNode << std::endl;
				std::cout << "\t\tMagnets: " << std::endl
						  << "\t\t\t";
				std::cout << std::endl;
			}
		}
		else
			std::cout << "\tNode::PrintArcs: No arcs.." << std::endl;
	}

	//! returns if the node has been used in the algorithm
	bool isUsed()
	{
		return bUsed;
	}
	//! Sets the node as used
	void used()
	{
		bUsed = true;
	}
	//! Returns the current distance stored in the node
	int distance()
	{
		return iDist;
	}
	//! Sets the value of the distance
	void setDistance(int distance)
	{
		iDist = distance;
	}
	//! returns the value of the id
	int getId()
	{
		return iNode;
	}
	//! returns the value of the parent
	int getParent()
	{
		return iParent;
	}
	//! returns the position of the node
	int getPosition(double *x, double *y, double *z)
	{
		*x = dX;
		*y = dY;
		*z = dZ;

		return 0;
	}
	//! returns the name of the node
	char *getName()
	{
		return cName;
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
			std::vector<Node> vListPointerNode;

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
			//	std::cout << "Route::Route: nodes = " << nodes << std::endl;

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
			//std::cout << "Route::~Route:" << std::endl;
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
		int addNode(int from_node, int to_node, Node node)
		{
			if ((from_node < 0) || (from_node > nodes - 1) || (to_node < 0) || (to_node > nodes - 1))
			{
				return -1;
			}
			std::vector<Node>::iterator it;

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
		std::vector<Node> getDetailRoute(int from_node, int to_node)
		{
			if ((from_node < 0) || (from_node > nodes - 1) || (to_node < 0) || (to_node > nodes - 1))
			{
				std::vector<Node> aux;
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
	//! Checks if it's possible to edit the graph
	bool bEdit;
	//!	Objeto de tipo Route con todas las rutas posibles
	Route *rRoutes;
	//! Cola de prioridad necesaria para el algoritmo
	std::priority_queue<PointerNode, std::deque<PointerNode>, NodeComparison> pqQueue;
	//! Array de punteros a nodo, para acceder a los nodos directamente. Referenciados por su ID
	Node **pNodes;
	//! max value of a node id
	int iMaxNodeId;

  public:
	//! Vector con los nodos del grafo
	std::vector<Node> vNodes;

  public:
	//! Public constructor
	Dijkstra();
	//! Public Destructor
	~Dijkstra();
	//! Disables the edition of nodes and arcs. Necesario para poder calcular rutas sin incoherencias
	//! Return -1 if exist errors (Nodos con arcos a nodos que no existen)
	int finalizeEdition(string *msg);
	//! Enable the edition of nodes and arcs. Necesario para poder añadir nodos y aristas. Todos los cálculos realizados se perderán
	void enableEdition();
	//! Reset the calculated route
	int resetRoutes();
	//! Deletes All the nodes
	int deleteNodes();
	//! Deletes selected arc
	int deleteArc(int from_node, int to_node);
	//! Deletes all the arcs from the node
	int deleteArcs(int from_node);
	//! Gets the optimum calculated route between selected nodes
	int getRoute(int inital_node, int end_node, std::vector<int> *route);
	int getRoute(int inital_node, int end_node, std::vector<Node> *route);

	//! Get list of nodes used or blocked
	bool getNodesUsed(std::vector<Node *> *route);

	//! GetNearestNode
	int getNearestNodeID(double x, double y, string frame);

	//! Adds a new node
	int addNode(int node, double x, double y, double z, double theta, std::string frame, char *name);

	//! Adds arc from a node to another with constant weight
	int addArc(int from_node, int to_node, double speed);
	//! Adds arc from a node to another with weight
	int addArc(int from_node, int to_node, double speed, int weight);
	//! Get the node with this id
	int getNodePosition(int node_id, double *x, double *y, double *z);
	//! Gets the arc between two nodes
	int getArcBetweenNodes(int from_node, int to_node, double *speed);
	//! Prints current nodes
	void printNodes();
	//! Deletes all the nodes
	int deleteAll();
	//! Gets the index of the node using his ID
	int getNodeIndex(int nodeID);
	//! Gets Node by nodeID
	Node *getNode(unsigned int node_id);
	//! Add Node to Zone
	int addNodeToZone(int iIDNode, int iIDZone, bool bMan);

	bool checkNodeFree(int iIDNode, int iIDRobot);

	//! Get Node From ID
	Node *getNodeFromID(int iIDNode);

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
	int popFromQueue(PointerNode *pn);
	//! Queues all the nodes
	int queueNodes();
	//! Updates the queue after modifiying values of the nodes.
	void updateQueue();
};

#endif
