/** \file Dijkstra.cc
 * \author Robotnik Automation S.L.L.
 * \version 1.3
 * \date    2009
 *
 * \brief Dijkstra class.
 * Implementa el algoritmo de Dijkstra
 * (C) 2009 Robotnik Automation, SLL
*/

#include <robotnik_fms_routes/Dijkstra.h>
#include <ros/ros.h>

using namespace std;

/*! \fn Dijkstra::Dijkstra()
 * 	\brief Public constructor
*/
Dijkstra::Dijkstra()
{
	bEdit = true;
	iMaxNodeId = -1;
}

/*! \fn Dijkstra::~Dijkstra()
 * 	\brief Public Destructor
*/
Dijkstra::~Dijkstra()
{
	delete rRoutes;
	delete[] pNodes;
	//cout << "Dijkstra::~Dijkstra" << endl;
}

/*! \fn int Dijkstra::finalizeEdition()
 * 	\brief Disables the edition of nodes and arcs. Necesario para poder calcular rutas sin incoherencias
 *  \return 0 if graph's elements are OK
*/
int Dijkstra::finalizeEdition(std::string *msg)
{
	int max_id = 0;
	bool bNodeFound = false;

	if (vNodes.size() > 0)
	{ // Si no hay nodos no tiene sentido
		// Comprobamos coherencia de los arcos de cada nodo. No pueden haber arcos que apunten a nodos inexistentes.
		// Esta comprobación no se realiza en el momento en que se inserta un nuevo arco, puesto que necesitamos
		// que sea flexible a la hora de leer el grafo desde un archivo xml.
		//Buscamos el mayor Id y creamos una ruta con todos esos nodos(no es muy eficiente en espacio pero podemos reutilizar rRoute)
		for (int i = 0; i < (int)vNodes.size(); i++)
		{
			//
			//Comprobamos si existen los adyacentes que tiene asignados cada nodo
			for (int j = 0; j < (int)vNodes[i].vAdjacent.size(); j++)
			{
				bNodeFound = false;
				for (int k = 0; k < (int)vNodes.size(); k++)
				{
					if (vNodes[i].vAdjacent[j].getNode() == vNodes[k].getId())
					{
						bNodeFound = true;
						break;
					}
				}
				if (!bNodeFound)
				{ // Nodo adyacente no existe
					//cout << "Dijkstra::FinalizeEdition: Error: Arco entre node " << vNodes[i].getId() << " y node " << vNodes[i].vAdjacent[j].GetNode() << " no existe." << endl;
					ROS_ERROR("Dijkstra::FinalizeEdition: Error: Arco entre node %d y node %d", vNodes[i].getId(), vNodes[i].vAdjacent[j].getNode());
					*msg = "Dijkstra::FinalizeEdition: Error: Arco entre node:" + std::to_string(vNodes[i].getId()) + " y node:" + std::to_string(vNodes[i].vAdjacent[j].getNode());

					return -1;
				}
			}
			if (vNodes[i].getId() > max_id)
				max_id = vNodes[i].getId();
		}
		//	cout << "Dijkstra::FinalizeEdition: Creamos matriz de " << max_id << "x" << max_id << endl;
		bEdit = false;
		rRoutes = new Route(max_id + 1); //Creamos matriz de posibles rutas
		pNodes = new Node *[max_id + 1]; //Creamos vector de punteros a cada nodo, refereciados por su ID
		iMaxNodeId = max_id;
		//! Enlazamos punteros
		for (int i = 0; i < (int)vNodes.size(); i++)
		{
			pNodes[vNodes[i].getId()] = &vNodes[i];
			//cout << "Dijkstra::FinalizeEdition: Nodo enlazado: " << pNodes[vNodes[i].getId()]->getId() << endl;
		}
	}

	return 0;
}

/*! \fn void Dijkstra::enableEdition()
 * 	\brief Disables the edition of nodes, edges. Necesario para poder calcular rutas sin incoherencias
*/
void Dijkstra::enableEdition()
{
	bEdit = true;
	iMaxNodeId = -1;

	delete rRoutes;
	delete[] pNodes;
}

/*! \fn int Dijkstra::DeleteNodes()
 * 	\brief Deletes All the nodes
 *  \return 0 if OL
*/
int Dijkstra::deleteNodes()
{
	if (!bEdit)
	{
		ROS_ERROR("Dijkstra::DeleteNodes: Error: Graph's edition must be disabled");
		return -2;
	}

	for (int i = 0; i < (int)vNodes.size(); i++)
	{
		vNodes[i].deleteAdjacent();
	}
	vNodes.clear();
	iMaxNodeId = -1;

	return 0;
}

/*! \fn int Dijkstra::DeleteAll()
 * 	\brief Deletes all the components
 *  \return 0 if OK
*/
int Dijkstra::deleteAll()
{
	enableEdition();

	for (int i = 0; i < (int)vNodes.size(); i++)
	{
		vNodes[i].deleteAdjacent();
	}

	vNodes.clear();
	ROS_INFO("Dijkstra::deleteAll: all the data has beee removed");
	return 0;
}

/*! \fn int Dijkstra::addNode(int node, double x, double y, double z,
							  double theta, std::string frame, char *name)
 * 	\brief  Adds a new node
 *  \returns The id of the node if OK
*/
//!
int Dijkstra::addNode(int node, double x, double y, double z, double theta,
					  std::string frame, char *name)
{
	if (!bEdit)
	{
		ROS_ERROR("Dijkstra::addNode: Error: Graph's edition must be disabled");
		return -2;
	}

	int size = vNodes.size();

	for (int i = 0; i < size; i++)
	{
		if (vNodes[i].getId() == node) // Nodo con id repetido
			return -1;
	}

	if (node < 0)
	{
		ROS_ERROR("Dijkstra::addNode: el id del nodo debe ser un numero positivo");
		return -1;
	}

	Node *new_node = new Node(node, x, y, z, theta, frame, name);

	vNodes.push_back(*new_node);

	return size + 1;
}

/*! \fn int Dijkstra::getNodeFromID(int iIDNode){
 * 	\brief Gets Pointer to a Node
 *  \return NULL if Error
*/
Node *Dijkstra::getNodeFromID(int iIDNode)
{
	Node *nod = NULL;

	int iNode = getNodeIndex(iIDNode);
	nod = &vNodes[iNode];
	return nod;
}

/*! \fn int Dijkstra::addArc(int from_node, int to_node, double speed)
 * 	\brief Adds an arc from a node to another with constant weight
 *  No controlamos que el nodo objetivo esté en la lista de nodos, puesto que puede que se introduzca después
*/
int Dijkstra::addArc(int from_node, int to_node, double speed)
{
	int locatedFrom = -1; // Id del nodo

	//ROS_INFO("Adding Arc from Node:%d to Node:%d",from_node,to_node);

	if (!bEdit)
	{ // Edición activa
		ROS_ERROR("Dijkstra::addArc: Error: Graph's edition must be disabled");
		return -2;
	}

	int size = vNodes.size();

	if (size == 0)
	{ //  No hay nodos, no puede añadirse ningún arco
		ROS_ERROR("Dijkstra::addArc: Error: No nodes");
		return -1;
	}

	if (from_node == to_node)
	{ // Los ciclos en un mismo nodo no están permitidos
		ROS_ERROR("Dijkstra::addArc: Error: graph cycles are not permitted: from %d to %d", from_node, to_node);
		return -1;
	}

	for (int i = 0; i < size; i++)
	{ // Comprobamos que el nodo  origen exista
		if (vNodes[i].getId() == from_node)
			locatedFrom = i;
	}
	if (locatedFrom < 0)
	{
		ROS_ERROR("Dijkstra::addArc: Error: Incorrect node number (%d)", from_node);

		return -1;
	}

	return vNodes[locatedFrom].addNodeAdjacent(to_node, speed);
}

/*! \fn int Dijkstra::addArc(int from_node, int to_node, double speed, int weight)
 * 	\brief Adds an arc from a node to another with weight
*/
int Dijkstra::addArc(int from_node, int to_node, double speed, int weight)
{
	int locatedFrom = -1; //Id del nodo

	if (!bEdit)
	{ // Edición activa
		ROS_ERROR("Dijkstra::addArc: Error: Graph's edition must be disabled");
		return -2;
	}

	int size = vNodes.size();

	if (size == 0)
	{
		ROS_ERROR("Dijkstra::addArc: Error: No nodes");
		return -1;
	}

	if (from_node == to_node)
	{
		ROS_ERROR("Dijkstra::addArc: Error: graph cycles are not permitted: from %d to %d", from_node, to_node);
		return -1;
	}

	for (int i = 0; i < size; i++)
	{
		if (vNodes[i].getId() == from_node) //Existe el nodo
			locatedFrom = i;
	}

	if (locatedFrom < 0)
	{
		ROS_ERROR("Dijkstra::addArc: Error: Incorrect node number (%d)", from_node);
		return -1;
	}

	return vNodes[locatedFrom].addNodeAdjacent(to_node, speed, weight);
}

/*! \fn int Dijkstra::deleteArc(int from_node, int to_node)
 * 	\brief Deletes selected arc
 *  \return 0 if OK
*/
int Dijkstra::deleteArc(int from_node, int to_node)
{
	if (!bEdit)
	{
		return -2;
	}
	int size = vNodes.size();

	if (size == 0)
	{
		cout << "Dijkstra::addArc: Error: No nodes" << endl;
		return -1;
	}

	if ((from_node < 0) || (from_node > size - 1) || (to_node < 0) || (to_node > size - 1))
	{
		cout << "Dijkstra::addArc: Error: Incorrect nodes number" << endl;
		return -1;
	}

	return vNodes[from_node].deleteAdjacent(to_node);
}

/*! \fn int Dijkstra::deleteArcs(int from_node)
 * 	\brief Deletes all the arc from the node
 *  \return 0 if OK
*/
int Dijkstra::deleteArcs(int from_node)
{
	if (!bEdit)
	{
		return -2;
	}
	int size = vNodes.size();

	if (size == 0)
	{
		cout << "Dijkstra::addArc: Error: No nodes" << endl;
		return -1;
	}

	if ((from_node < 0) || (from_node > size - 1))
	{
		cout << "Dijkstra::addArc: Error: Incorrect nodes number" << endl;
		return -1;
	}

	return vNodes[from_node].deleteAdjacent();
}

/*! \fn int Dijkstra::resetRoutes()
 * 	\brief Reset the calculated routes
 *  \return 0 if OK
*/
int Dijkstra::resetRoutes()
{
	if (bEdit)
	{
		ROS_ERROR("Dijkstra::resetRoutes: Error: Graph's edition must be disabled");
		return -2;
	}
	rRoutes->reset();
	return 0;
}

/*! \fn int Dijkstra::GetNearestNodeID(double x, double y)
 * 	\brief return Nearest Node ID
*/
int Dijkstra::getNearestNodeID(double x, double y, std::string frame)
{
	double dist = 999999999.9;
	int current_node = -1;
	for (int in = 0; in < vNodes.size(); in++)
	{
		if (vNodes[in].dZ == 0)
		{
			if (vNodes[in].sFrame_id == frame)
			{
				double new_dist = sqrt(((vNodes[in].dX - x) * (vNodes[in].dX - x)) +
									   ((vNodes[in].dY - y) * (vNodes[in].dY - y)));
				if (new_dist <= dist)
				{
					dist = new_dist;
					current_node = vNodes[in].iNode;
				}
			}
		}
	}
	//ROS_INFO("Near Node:%d",current_node);
	return current_node;
}

/*! \fn int Dijkstra::getRoute(int initial_node, int end_node, std::vector<int> *route)
 * 	\brief Gets the best calculated route between selected nodes
 *  \return 0 if OK
*/
int Dijkstra::getRoute(int initial_node, int end_node, std::vector<int> *route)
{
	int positionInitialNode = -1, positionEndNode = -1;

	if (bEdit)
	{
		ROS_ERROR("Dijkstra::getRoute: Error: Graph's edition must be disabled");
		return -2;
	}

	if ((initial_node < 0) || (end_node < 0))
	{
		ROS_ERROR("Dijkstra::getRoute: Error: node id must be greater than 0");
		return -1;
	}
	int size = vNodes.size();

	// Buscamos las posiciones dentro del vector, si es que existen los nodos
	for (int i = 0; i < size; i++)
	{
		if (vNodes[i].getId() == initial_node) //Existe el nodo
			positionInitialNode = i;
		else if (vNodes[i].getId() == end_node) //Existe el nodo
			positionEndNode = i;
	}
	if (positionInitialNode < 0)
	{ // El nodo inicial no existe
		ROS_ERROR("Dijkstra::getRoute: Error: initial  node %d does not exist", initial_node);
		return -1;
	}
	if (positionEndNode < 0)
	{ // El nodo final no existe
		ROS_ERROR("Dijkstra::getRoute: Error: end node %d does not exist(initial = %d)", end_node, initial_node);
		return -1;
	}

	if (rRoutes->existRoute(initial_node, end_node))
	{ //Si la ruta existe la devolvemos directamente
		//rRoutes->print(initial_node, end_node);
		*route = rRoutes->getRoute(initial_node, end_node);
		return 0;
	}

	// si no existe la ruta la calculamos
	//cout << "Dijkstra::getRoute: Calculamos ruta" << endl;
	if (calculateRoute(positionInitialNode, positionEndNode) == 0)
	{
		//Vamos añadiendo los nodos a la ruta
		int k = positionEndNode;
		int id = vNodes[k].getId();
		while (id != NO_PARENT)
		{ // Recorremos la ruta de manera inversa hasta llegar al nodo inicial, cuyo padre es null
			if (rRoutes->addNode(initial_node, end_node, id) != 0)
			{ //Los dos primeros parámetros indentifican la ruta, el último añade los nodos por donde transcurre la ruta
				ROS_ERROR("Dijkstra::getRoute: Error adding the node %d on route (%d, %d)", id, initial_node, end_node);
			}
			if (rRoutes->addNode(initial_node, end_node, vNodes[k]) != 0)
			{ //Los dos primeros parámetros indentifican la ruta, el último añade los nodos por donde transcurre la ruta
				ROS_ERROR("Dijkstra::getRoute: Error adding the node %d on route (%d, %d)", id, initial_node, end_node);
			}
			id = vNodes[k].getParent();
			k = getNodeIndex(id);
		}
		//Ruta añadida
		//rRoutes->print(initial_node, end_node);
		*route = rRoutes->getRoute(initial_node, end_node);
		return 0;
	}
	else
	{
		ROS_ERROR("Dijkstra::getRoute: Error: CalculateRoute failed");
		return -1;
	}
}

/*! \fn int Dijkstra::getRoute(int initial_node, int end_node, std::vector<Node> *route)
 * 	\brief Gets the best calculated route between selected nodes
 *  \return 0 if OK. And the the array with the nodes of the route
*/
int Dijkstra::getRoute(int initial_node, int end_node, std::vector<Node> *route)
{
	int positionInitialNode = -1, positionEndNode = -1;

	if (bEdit)
	{
		ROS_ERROR("Dijkstra::getRoute: Error: Graph's edition must be disabled");
		return -2;
	}

	if ((initial_node < 0) || (end_node < 0))
	{
		ROS_ERROR("Dijkstra::getRoute: Error: node id must be greater than 0");
		return -1;
	}
	int size = vNodes.size();

	// Buscamos las posiciones dentro del vector, si es que existen los nodos
	for (int i = 0; i < size; i++)
	{
		if (vNodes[i].getId() == initial_node) // Existe el nodo
			positionInitialNode = i;
		else if (vNodes[i].getId() == end_node) // Existe el nodo
			positionEndNode = i;
	}
	if (positionInitialNode < 0)
	{ // El nodo inicial no existe
		ROS_ERROR("Dijkstra::GetRoute: Error: initial  node %d does not exist", initial_node);
		return -1;
	}
	if (positionEndNode < 0)
	{ // El nodo inicial no existe
		ROS_ERROR("Dijkstra::GetRoute: Error: end  node %d does not exist", end_node);
		return -1;
	}

	if (rRoutes->existRoute(initial_node, end_node))
	{ // Si la ruta existe la devolvemos directamente
		*route = rRoutes->getDetailRoute(initial_node, end_node);
		//rRoutes->print(initial_node, end_node);
		return 0;
	}

	// Si no existe la ruta la calculamos
	if (calculateRoute(positionInitialNode, positionEndNode) == 0)
	{
		// Vamos añadiendo los nodos a la ruta
		int k = positionEndNode;
		int id = vNodes[k].getId();
		while (id != NO_PARENT)
		{ // Recorremos la ruta de manera inversa hasta llegar al nodo inicial, cuyo padre es null
			if (rRoutes->addNode(initial_node, end_node, vNodes[k]) != 0)
			{ //Los dos primeros parámetros indentifican la ruta, el último añade los nodos por donde transcurre la ruta
				ROS_ERROR("Dijkstra::getRoute: Error adding the node %d on route (%d, %d)", id, initial_node, end_node);
			}
			if (rRoutes->addNode(initial_node, end_node, id) != 0)
			{ //Los dos primeros parámetros indentifican la ruta, el último añade los nodos por donde transcurre la ruta
				ROS_ERROR("Dijkstra::getRoute: Error adding the node %d on route (%d, %d)", id, initial_node, end_node);
			}
			id = vNodes[k].getParent();
			k = getNodeIndex(id);
		}
		//Ruta añadida
		//rRoutes->Print(initial_node, end_node);
		*route = rRoutes->getDetailRoute(initial_node, end_node);
		return 0;
	}
	else
	{
		ROS_ERROR("Dijkstra::GetRoute: Error: CalculateRoute failed");
		return -1;
	}
}

/*! \fn int Dijkstra::resetQueue()
 * 	\brief Resets the priority queue
*/
int Dijkstra::resetQueue()
{
	while (!pqQueue.empty()) // Mientras la cola no esté vacia, vamos extrayendo elementos
		pqQueue.pop();
	return 0;
}

/*! \fn int Dijkstra::resetNodes()
 * 	\brief Resets the values of every node
*/
int Dijkstra::resetNodes()
{
	for (int i = 0; i < (int)vNodes.size(); i++)
	{
		vNodes[i].reset();
	}
	return 0;
}

/*! \fn int Dijkstra::setInitialNode(int node)
 * 	\brief Set the initial node
*/
int Dijkstra::setInitialNode(int node)
{
	vNodes[node].setInitial();
	return 0;
}

/*! \fn int Dijkstra::pushIntoQueue(int node)
 * 	\brief Push a node into the queue
*/
int Dijkstra::pushIntoQueue(int node)
{
	PointerNode pNode(&vNodes[node]);
	pqQueue.push(pNode);

	return 0;
}

/*! \fn int Dijkstra::popFromQueue(PointerNode *pn)
 * 	\brief Pops the node with more priority
 *  \return -1 if the queue is empty
 *  \return 0 if OK
*/
int Dijkstra::popFromQueue(PointerNode *pn)
{
	if (pqQueue.empty())
	{
		ROS_ERROR("Dijkstra::PopFromQueue: the queue is empty");
		return -1;
	}

	*pn = pqQueue.top(); // Get the node
	pqQueue.pop();		 // Extracts the node

	return 0;
}

/*! \fn int Dijkstra::queueNodes()
 * 	\brief Queues all the nodes
*/
int Dijkstra::queueNodes()
{
	for (int i = 0; i < (int)vNodes.size(); i++)
	{
		pushIntoQueue(i);
	}
	return 0;
}

/*! \fn void Dijkstra::updateQueue()
 * 	\brief Updates the queue after modifiying values of the nodes
*/
void Dijkstra::updateQueue()
{
	vector<PointerNode> vPN;

	while (!pqQueue.empty())
	{ // Extraemos todos los nodos
		vPN.push_back(pqQueue.top());
		pqQueue.pop();
	}
	for (int i = 0; i < (int)vPN.size(); i++)
	{ // Los volvemos a encolar
		pqQueue.push(vPN[i]);
	}
}

/*! \fn int Dijkstra::calculateRoute(int initial_node, int end_node)
 * 	\brief Calculates the best route
 *  \return 0 if OK
*/
int Dijkstra::calculateRoute(int initial_node, int end_node)
{
	PointerNode pNodeMin;
	double diffX = 0.0; // Lo utilizaremos para el cálculo de la distancia en X entre dos nodos
	double diffY = 0.0; // Lo utilizaremos para el cálculo de la distancia en Y entre dos nodos
	double distance = 0.0;

	resetNodes();
	setInitialNode(initial_node);
	resetQueue();

	/// Empezamos algoritmo de Dijkstra propiamente dicho
	queueNodes(); //Encolamos todos los nodos del grafo

	//cout << "Dijkstra::CalculateRoute: Traza: " << pqQueue.size() << " nodos encolados" << endl;
	while (!pqQueue.empty())
	{ //Mientras la cola no esté vacía
		if (popFromQueue(&pNodeMin) == 0)
		{ //Extraemos el nodo con distancia mínima de la cola
			//cout << "\t Extraemos nodo " << pNodeMin.node->getId() << " .Quedan " << pqQueue.size() << endl;
			//Para todos los nodos adyacentes al extraido calculamos distancias
			for (int i = 0; i < (int)pNodeMin.node->vAdjacent.size(); i++)
			{
				int nodeAdjacent = pNodeMin.node->vAdjacent[i].getNode();

				int nodeAdjIndex = getNodeIndex(nodeAdjacent);
				if (nodeAdjIndex < 0)
				{
					ROS_ERROR("Dijkstra::calculateRoute: Error: node adjacent %d does not exist", nodeAdjacent);
					return -1;
				}

				// Añadimos al peso la distancia entre los nodos
				diffX = vNodes[nodeAdjIndex].dX - pNodeMin.node->dX;
				diffY = vNodes[nodeAdjIndex].dY - pNodeMin.node->dY;
				distance = sqrt(diffX * diffX + diffY * diffY) * 1000.0;
				int weight = (int)(pNodeMin.node->vAdjacent[i].getWeight());

				if (!vNodes[nodeAdjIndex].isUsed())
				{ // Si no hemos utilizado el nodo
					if (vNodes[nodeAdjIndex].distance() > (pNodeMin.node->distance() + weight))
					{																		  // distancia[v] > distancia[u] + peso (u, v)
						vNodes[nodeAdjIndex].setDistance(pNodeMin.node->distance() + weight); // distancia[v] = distancia[u] + peso (u, v)
						vNodes[nodeAdjIndex].setParent(pNodeMin.node->getId());				  // padre[v] = u
					}
				} // else
			}
			pNodeMin.node->used(); // Marcamos el nodo como utilizado
			updateQueue();
		}
	}
	// Cuando se vacíe la cola llegaremos al final del algoritmo puesto que habremos recorrido todos los nodos conexos del grafo
	if (vNodes[end_node].getParent() == NO_PARENT)
	{ // Si el nodo final no tiene nodo antecesor es que el grafo no es conexo
		ROS_ERROR("Dijkstra::calculateRoute: Error: nodes %d and %d are not conex", initial_node, end_node);
		return -1;
	}
	return 0;
}

/*! \fn void Dijkstra::printNodes()
 * 	\brief Prints current nodes
*/
void Dijkstra::printNodes()
{
	double x = 0.0, y = 0.0, z = 0.0;
	int size = vNodes.size();

	if (size > 0)
	{
		cout << "Dijkstra::PrintNodes: " << size << " nodes" << endl;
		for (int i = 0; i < size; i++)
		{
			cout << "\tNode " << vNodes[i].getId() << ": Dist = " << vNodes[i].iDist << endl;
			cout << "\tName = " << vNodes[i].getName() << " , parent = " << vNodes[i].iParent << endl;
			vNodes[i].getPosition(&x, &y, &z);
			cout << "\tPosition: x = " << x << " y = " << y << " z = " << z << endl;
			vNodes[i].printArcs();
			cout << endl;
		}
	}
	else
		cout << "Dijkstra::printNodes: No nodes.." << endl;
}

/*! \fn int Dijkstra::getNodeIndex(int nodeID)
 * 	\brief Gets the index of the node using his ID
 *  \return -1 if id does not exist
*/
int Dijkstra::getNodeIndex(int nodeID)
{
	int ret = -1;
	for (int i = 0; i < (int)vNodes.size(); i++)
	{
		if (vNodes[i].getId() == nodeID)
		{
			ret = i;
			break;
		}
	}
	return ret;
}

/*! \fn int Dijkstra::getNodePosition(int node_id, double *x, double *y, double *z)
 * 	\brief Gets the node's position with this id
 *  \return 0 if OK
*/
int Dijkstra::getNodePosition(int node_id, double *x, double *y, double *z)
{
	if (bEdit)
	{
		ROS_ERROR("Dijkstra::getNodePosition: Edition must be disabled");
		return -2;
	}

	if ((node_id > iMaxNodeId) || (node_id < 0))
	{
		ROS_ERROR("Dijkstra::getNodePosition: node %d does not exist", node_id);
		return -1;
	}

	pNodes[node_id]->getPosition(x, y, z);
	return 0;
}

/*! \fn int Dijkstra::getArcBetweenNodes(int from_node, int to_node, std::vector<Magnet> *magnets_on_arc, double *speed){
 * 	\brief Gets the arc's values between nodes
 *  \return 0 if OK
*/
int Dijkstra::getArcBetweenNodes(int from_node, int to_node, double *speed)
{
	// La edición debe estar finalizada
	if (bEdit)
	{
		//cout << "Dijkstra::GetMagnetsBetweenNodes: Edition must be disabled" << endl;
		ROS_ERROR("Dijkstra::GetMagnetsBetweenNodes: Edition must be disabled");
		return -2;
	}
	// Comprobación rango de nodos
	if ((from_node > iMaxNodeId) || (from_node < 0) || (to_node > iMaxNodeId) || (to_node < 0))
	{
		//	cout << "Dijkstra::GetMagnetsBetweenNodes: Bad number of nodes" << endl;
		ROS_ERROR("Dijkstra::GetMagnetsBetweenNodes: Bad id of nodes: %d to %d", from_node, to_node);
		return -1;
	}
	return 0;
}

/*! \fn Node* Dijkstra::getNode(unsigned int nodeID)
 * 	\brief Gets Node by nodeID
*/
Node *Dijkstra::getNode(unsigned int node_id)
{
	//if( (node_id > iMaxNodeId) || (node_id < 0)){
	//	ROS_ERROR("Dijkstra::getNode: node %d does not exist", node_id);
	//	return NULL;
	//}
	//ROS_ERROR("Dijkstra::getNode: node %d  ", node_id);

	for (int i = 0; i < vNodes.size(); i++)
	{
		if (vNodes[i].iNode == node_id)
			return &vNodes[i];
		//ROS_ERROR("Dijkstra::getNode: node %d, pNodes[i]->iNode:%d  ", node_id,vNodes[i].iNode);
	}
	//ROS_ERROR("Dijkstra::getNode: node %d  NULL ", node_id);
	return NULL;
}
