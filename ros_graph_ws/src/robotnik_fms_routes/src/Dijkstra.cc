/** \file Dijkstra.cc
 * \author Robotnik Automation S.L.L.
 * \version 1.3
 * \date    2009
 *
 * \brief Dijkstra class.
 * Implementa el algoritmo de Dijkstra
 * (C) 2009 Robotnik Automation, SLL
*/

#include <ros/ros.h>
#include <robotnik_fms_routes/Dijkstra.h>

using namespace std;

/*! \fn Dijkstra::Dijkstra()
 * 	\brief Public constructor
*/
Dijkstra::Dijkstra()
{
	bEdit = false; // TODO Now, bEdit has to be always disabled
	iMaxNodeId = -1;
}

/*! \fn Dijkstra::~Dijkstra()
 * 	\brief Public Destructor
*/
Dijkstra::~Dijkstra()
{
}

/*! \fn int Dijkstra::finalizeEdition(graph_msgs::GraphNodeArray *graphData)
 * 	\brief Disables the edition of nodes and arcs. Necesario para poder calcular rutas sin incoherencias
 *  \return "OK" if graph's elements are OK
*/
std::string Dijkstra::finalizeEdition(std::vector<Node *> graph)
{
	int max_id = 0;
	bool bNodeFound = false;

	if (graph.size() > 0)
	{ // Si no hay nodos no tiene sentido
		// Comprobamos coherencia de los arcos de cada nodo. No pueden haber arcos que apunten a nodos inexistentes.
		// Esta comprobación no se realiza en el momento en que se inserta un nuevo arco, puesto que necesitamos
		// que sea flexible a la hora de leer el grafo desde un archivo json.
		//Buscamos el mayor Id y creamos una ruta con todos esos nodos(no es muy eficiente en espacio pero podemos reutilizar rRoute)
		for (int i = 0; i < (int)graph.size(); i++)
		{

			// Comprobamos si existen los adyacentes que tiene asignados cada nodo
			for (int j = 0; j < (int)graph[i]->node.arc_list.size(); j++)
			{
				bNodeFound = false;
				for (int k = 0; k < (int)graph.size(); k++)
				{
					if (graph[i]->node.arc_list[j].node_dest == graph[k]->node.id)
					{
						bNodeFound = true;
						break;
					}
				}
				if (!bNodeFound)
				{ // Nodo adyacente no existe
					ROS_ERROR("Dijkstra::FinalizeEdition: Error: Arco entre nodo %d y nodo %d", graph[i]->node.id, graph[i]->node.arc_list[j].node_dest);
					return "Dijkstra::FinalizeEdition: Error: Arco entre node:" + std::to_string(graph[i]->node.id) + " y node:" + std::to_string(graph[i]->node.arc_list[j].node_dest);
				}
			}
			max_id = max(max_id, (int)graph[i]->node.id);
		}
		//bEdit = false;
		rRoutes = new Route(max_id + 1); // Creamos matriz de posibles rutas
		iMaxNodeId = max_id;
	}
	return "OK";
}

/*! \fn void Dijkstra::enableEdition()
 * 	\brief Disables the edition of nodes, edges. Necesario para poder calcular rutas sin incoherencias
*/
void Dijkstra::enableEdition()
{
	bEdit = true;
	iMaxNodeId = -1;

	delete rRoutes;
}

/*! \fn void Dijkstra::enableEdition()
 * 	\brief Disables the edition of nodes, edges. Necesario para poder calcular rutas sin incoherencias
*/
bool Dijkstra::isOnEdition()
{
	return bEdit;
}

/*! \fn int Dijkstra::DeleteNodes()
 * 	\brief Deletes All the nodes
 *  \return 0 if OL
*/
int Dijkstra::deleteNodes()
{
	if (bEdit)
	{
		ROS_ERROR("Dijkstra::DeleteNodes: Error: Graph's edition must be enabled");
		return -2;
	}

	for (int i = 0; i < (int)vNodes.size(); i++)
	{
		vNodes[i]->deleteAdjacent();
	}
	vNodes.clear();
	iMaxNodeId = -1;

	return 0;
}

/*! \fn std::string Dijkstra::deleteNode(int node_id)
 * 	\brief  Deletes a node
 *  \returns "OK"
*/
//!
std::string Dijkstra::deleteNode(int node_id)
{
	int index = getNodeIndex(node_id);

	if (bEdit)
	{
		ROS_ERROR("Dijkstra::addNode: Error: Graph's edition must be enabled");
		return "Dijkstra::addNode: Error: Graph's edition must be enabled";
	}
	else if (index == -1)
	{
		ROS_ERROR("Dijkstra::GetNodePosition: node %d does not exist", node_id);
		return "Dijkstra::GetNodePosition: node " + std::to_string(node_id) + " does not exist";
	}

	for (int i = 0; i < vZones.size(); i++)
	{
		for (int j = 0; j < vZones[i].vpNodes.size(); j++)
		{
			if (vZones[i].vpNodes[j]->node.id == node_id)
			{
				vZones[i].vpNodes.erase(vZones[i].vpNodes.begin() + j);
			}
		}
	}

	vNodes.erase(vNodes.begin() + index);

	for (int i = 0; i < vNodes.size(); i++)
	{
		for (int j = 0; j < vNodes[i]->node.arc_list.size(); j++)
		{
			if (vNodes[i]->node.arc_list[j].node_dest == node_id)
			{
				vNodes[i]->node.arc_list.erase(vNodes[i]->node.arc_list.begin() + j);
			}
		}
	}

	return "OK";
}

/*! \fn std::string Dijkstra::addNode(graph_msgs::GraphNode node)
 * 	\brief  Adds a new node
 *  \returns The id of the node if OK
*/
//!
std::string Dijkstra::addNode(graph_msgs::GraphNode node)
{
	if (bEdit)
	{
		ROS_ERROR("Dijkstra::addNode: Error: Graph's edition must be enabled");
		return "Dijkstra::addNode: Error: Graph's edition must be enabled";
	}

	int size = vNodes.size();

	if (node.id < 0)
	{
		ROS_ERROR("Dijkstra::addNode: Error: The ID has to be an integer");
		return "Dijkstra::addNode: Error: The ID has to be an integer";
	}

	for (int i = 0; i < size; i++)
	{
		if (vNodes[i]->getId() == node.id)
		{ // Nodo con id repetido
			ROS_ERROR("Dijkstra::addNode: Error: The ID is in use");
			return "Dijkstra::addNode: Error: The ID is in use";
		}
	}

	vNodes.push_back(new Node(node));

	for (int i = 0; i < node.arc_list.size(); i++)
	{
		if (node.arc_list[i].distance == 0)
			node.arc_list[i].distance = 1;
		if (node.arc_list[i].max_speed == 0)
			node.arc_list[i].max_speed = 1.5;
		if (addArc(node.id, node.arc_list[i]) < 0)
		{
			vNodes.pop_back();
			return "Error adding arc " + i;
		}
	}

	addZone(node.zone, 1);
	addNodeToZone(node.id, node.zone);

	return "OK";
}

/*! \fn int Dijkstra::AddZone(int iIDZone, int iMaxRobots){
 * 	\brief Adds a Zone
 *  \return 0 if OK
*/
int Dijkstra::addZone(int iIDZone, int iMaxRobots)
{
	for (int i = 0; i < (int)vZones.size(); i++)
	{
		if (vZones[i].iIDZone == iIDZone)
			return -1;
	}
	vZones.push_back(Zone(iIDZone, iMaxRobots));
	return 0;
}

/*! \fn int Dijkstra::FromID(int iIDNode){
 * 	\brief Gets Pointer to a Node
 *  \return NULL if Error
*/
Node *Dijkstra::getNodeFromId(int iIDNode)
{
	int iNode = getNodeIndex(iIDNode);

	if (iNode == -1)
		return 0;
	else
		return vNodes[iNode];
}

/*! \fn int Dijkstra::addNodetoZone(int iIDNode,int iIDZone){
 * 	\brief Adds Node to a Zone
 *  \return 0 if OK
*/
int Dijkstra::addNodeToZone(int iIDNode, int iIDZone)
{
	Node *node = getNodeFromId(iIDNode);

	if (node)
	{
		for (int i = 0; i < vZones.size(); i++)
		{
			if (vZones[i].iIDZone == iIDZone)
			{
				return vZones[i].addNodeToZone(node);
			}
		}
		return 0;
	}
	else
	{
		return -1;
	}
}

/*! \fn bool Dijkstra::CheckZoneFree(int iIDZone,int iIDRobot){
 * 	\brief Checks if zone Free
 *  \return true if free, false in other case
*/
bool Dijkstra::checkZoneFree(int iIDZone, int iIDRobot)
{
	//ROS_INFO("Checking Zone: %d for Robot: %d ", iIDZone, iIDRobot);

	int iZone = -1;		 // Zone position on vZones array
	bool bFound = false; // Zone found
	int iMaxRobots = 0;  // Max num of robot in this zone

	for (int i = 0; i < (int)vZones.size(); i++)
	{
		if (vZones[i].iIDZone == iIDZone)
		{
			iMaxRobots = vZones[i].iMaxRobots;
			bFound = true;
			iZone = i;
			break;
		}
	}
	if (!bFound)
	{
		return true;
	}

	bool bPresRobot[100]; // True if robot is on the zone
	for (int i = 0; i < 100; i++)
	{
		bPresRobot[i] = false;
	}

	for (int j = 0; j < (int)vZones[iZone].vpNodes.size(); j++)
	{
		Node *pNode;
		pNode = vZones[iZone].vpNodes[j];
		if (pNode == NULL)
		{
			return true;
		}
		//ROS_INFO("Zone: %d, Checking node: %d, robot: %d, resrobot: %d, iMaxRobots: %d", iIDZone, pNode->node.id, pNode->iRobot, pNode->iResRobot, iMaxRobots);
		if (iIDRobot >= 0)
		{
			if (pNode->iRobot != iIDRobot)
				bPresRobot[pNode->iRobot] = true;
			if (pNode->iResRobot != iIDRobot)
				bPresRobot[pNode->iResRobot] = true;
		}
		else
		{
			if (pNode->iRobot >= 0 && pNode->iRobot <= 100)
			{
				bPresRobot[pNode->iRobot] = true;
			}
			if (pNode->iResRobot >= 0 && pNode->iResRobot <= 100)
			{
				bPresRobot[pNode->iResRobot] = true;
			}
		}
	}
	int iNR = 0;
	for (int i = 0; i < 100; i++)
	{
		if (bPresRobot[i])
			iNR++;
	}
	//ROS_INFO("Zone:%d,Checking,NR:%d,iMaxRobots:%d",iIDZone,iNR,iMaxRobots);
	if (iNR >= iMaxRobots)
	{
		return false;
		ROS_ERROR("The number of robots (%i) is >= than max (%i)", iNR, iMaxRobots);
	}
	else
	{
		return true;
	}
}

/*! \fn int Dijkstra::CheckNodeFree(int iIDNode,int iIDRobot){
 * 	\brief Checks if Node free for use
 *  \return true if free, false if not
*/
bool Dijkstra::checkNodeFree(int iIDNode, int iIDRobot)
{
	//char cAux[LOG_STRING_LENGTH] = "\0";
	int i = 0;
	int iNR = 0;
	Node *pNodeOr;
	pNodeOr = getNodeFromId(iIDNode);

	//ROS_INFO("Checking Node:%d, for robot:%d",iIDNode,iIDRobot);

	if (pNodeOr == NULL)
	{
		ROS_INFO("Node:%d, Robot %d: NULL", iIDNode, iIDRobot);
		return false;
	}

	//if ((pNodeOr->bBlocked) && (pNodeOr->iRobot!=iIDRobot)) return false;

	for (int i = 0; i < vZones.size(); i++)
	{
		for (int j = 0; j < vZones[i].vpNodes.size(); i++)
		{
			// If node is on this zone
			if (vZones[i].vpNodes[j]->node.id == pNodeOr->node.id)
			{
				if (!checkZoneFree(vZones[i].iIDZone, iIDRobot))
				{
					iNR++;
				}
			}
		}
	}

	if (iIDRobot >= 0)
	{
		if ((pNodeOr->iRobot >= 0) && (pNodeOr->iRobot != iIDRobot))
		{
			//ROS_INFO("Node:%d IRobot %d: not Free",iIDNode,pNodeOr->iRobot);
			iNR++;
		}
		if ((pNodeOr->iResRobot >= 0) && (pNodeOr->iResRobot != iIDRobot))
		{
			//ROS_INFO("Node:%d IResRobot %d: not Free",iIDNode,pNodeOr->iResRobot);
			iNR++;
		}
	}
	else
	{
		if (pNodeOr->iRobot >= 0)
		{
			//ROS_INFO("AAAAA");
			iNR++;
		}
		if (pNodeOr->iResRobot >= 0)
		{
			//ROS_INFO("BBBB");
			iNR++;
		}
	}
	if (iNR > 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

/*! \fn int Dijkstra::addArc(int from_node, graph_msgs::GraphArc *pointerToArc)
 * 	\brief Adds an arc from a node to another with weight
*/
int Dijkstra::addArc(int from_node, graph_msgs::GraphArc new_arc)
{
	int locatedFrom = -1; //Id del nodo

	if (bEdit)
	{ // Edición activa
		ROS_ERROR("Dijkstra::addArc: Error: Graph's edition must be enabled");
		return -2;
	}

	int size = vNodes.size();

	if (size == 0)
	{
		ROS_ERROR("Dijkstra::addArc: Error: No nodes");
		return -1;
	}

	if (from_node == new_arc.node_dest)
	{
		ROS_ERROR("Dijkstra::addArc: Error: graph cycles are not permitted: from %d to %d", from_node, new_arc.node_dest);
		return -1;
	}

	for (int i = 0; i < size; i++)
	{
		if (vNodes[i]->getId() == from_node) //Existe el nodo
			locatedFrom = i;
	}

	if (locatedFrom < 0)
	{
		ROS_ERROR("Dijkstra::addArc: Error: Incorrect node number (%d)", from_node);
		return -1;
	}

	return vNodes[locatedFrom]->addNodeAdjacent(new_arc);
}

/*! \fn int Dijkstra::deleteArc(int from_node, int to_node)
 * 	\brief Deletes selected arc
 *  \return 0 if OK
*/
int Dijkstra::deleteArc(int from_node, int to_node)
{
	if (bEdit)
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

	return vNodes[from_node]->deleteAdjacent(to_node);
}

/*! \fn int Dijkstra::deleteArcs(int from_node)
 * 	\brief Deletes all the arc from the node
 *  \return 0 if OK
*/
int Dijkstra::deleteArcs(int from_node)
{
	if (bEdit)
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

	return vNodes[from_node]->deleteAdjacent();
}

/*! \fn int Dijkstra::resetRoutes()
 * 	\brief Reset the calculated routes
 *  \return 0 if OK
*/
int Dijkstra::resetRoutes()
{
	if (bEdit)
	{
		ROS_ERROR("Dijkstra::resetRoutes: Error: Graph's edition must be enabled");
		return -2;
	}
	rRoutes->reset();
	return 0;
}

/*! \fn int Dijkstra::GetNearestNodeID(double x, double y, std::string frame)
 * 	\brief return Nearest Node ID
*/
int Dijkstra::getNearestNodeID(double x, double y, std::string frame)
{
	double dist = 999999999.9;
	int current_node = -1;
	for (int in = 0; in < vNodes.size(); in++)
	{
		if (vNodes[in]->node.pose.z == 0)
		{
			if (vNodes[in]->node.pose.frame_id == frame)
			{
				double new_dist = std::sqrt(std::pow(vNodes[in]->node.pose.x - x, 2) +
											std::pow(vNodes[in]->node.pose.y - y, 2));
				if (new_dist <= dist)
				{
					dist = new_dist;
					current_node = vNodes[in]->node.id;
				}
			}
		}
	}
	//ROS_INFO("Near Node:%d",current_node);
	return current_node;
}

/*! \fn int Dijkstra::ReserveNode(int iRobot,int iIDNode)
 * 	\brief Reserve Node for Robot
*/
bool Dijkstra::reserveNode(int iRobot, int iIDNode)
{
	for (int in = 0; in < vNodes.size(); in++)
	{
		if (in == iIDNode)
		{
			//if (vNodes[in]->iResRobot!=iRobot) ROS_INFO("Reserve Node:%d for Robot:%d",iIDNode,iRobot);
			vNodes[in]->iResRobot = iRobot;
		}
		else
		{
			if (vNodes[in]->iResRobot == iRobot)
			{
				//ROS_INFO("%d,UnReserve Node:%d for Robot:%d",in,iIDNode,iRobot);
				vNodes[in]->iResRobot = -1;
			}
		}
	}
	return true;
}

/*! \fn int Dijkstra::UnBlockAll(int iRobot)
 * 	\brief Unblock All Nodes for Robot
*/
bool Dijkstra::unBlockAll(int iRobot)
{
	for (int in = 0; in < vNodes.size(); in++)
	{
		if (vNodes[in]->iResRobot == iRobot)
		{
			ROS_INFO("UnBlock Node:%d for Robot:%d", in, iRobot);
			vNodes[in]->iRobot = -1;
			//vNodes[in]->bBlocked=false;
		}
	}
	return true;
}

/*! \fn std::vector<Node *> Dijkstra::GetNodesUsed()
 * 	\brief Gets the list of nodes used or blocked
 *  \return 0 if OK
*/
std::vector<graph_msgs::GraphNode> Dijkstra::getNodesUsed()
{
	std::vector<graph_msgs::GraphNode> route;
	for (int in = 0; in < vNodes.size(); in++)
	{
		// Checking if the node has a robot
		bool bAdd = false;
		if (vNodes[in]->iRobot >= 0)
		{
			bAdd = true;
		}
		if (vNodes[in]->iResRobot >= 0)
		{
			bAdd = true;
		}
		// Checking if the zone is free
		for (int i = 0; i < vZones.size() && !bAdd; i++)
		{
			for (int j = 0; j < vZones[i].vpNodes.size(); j++)
			{
				// If node is on a zone, we check if the zone is free
				if (vZones[i].vpNodes[j]->node.id == vNodes[in]->node.id)
				{
					if (!checkZoneFree(vZones[i].iIDZone, -1))
					{
						bAdd = true;
					}
				}
			}
		}
		if (bAdd)
		{
			route.push_back(vNodes[in]->node);
		}
	}
	return route;
}

/*! \fn std::vector<Node *> Dijkstra::GetNodes()
 * 	\brief Gets the list of nodes used or blocked
 *  \return 0 if OK
*/
std::vector<Node *> Dijkstra::getNodes()
{
	return vNodes;
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
		ROS_ERROR("Dijkstra::getRoute: Error: Graph's edition must be enabled");
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
		if (vNodes[i]->getId() == initial_node) //Existe el nodo
			positionInitialNode = i;
		else if (vNodes[i]->getId() == end_node) //Existe el nodo
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
		int id = vNodes[k]->getId();
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
			id = vNodes[k]->getParent();
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
int Dijkstra::getRoute(int initial_node, int end_node, std::vector<Node *> *route)
{
	int positionInitialNode = -1, positionEndNode = -1;

	if (bEdit)
	{
		ROS_ERROR("Dijkstra::getRoute: Error: Graph's edition must be enabled");
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
		if (vNodes[i]->getId() == initial_node) // Existe el nodo
			positionInitialNode = i;
		else if (vNodes[i]->getId() == end_node) // Existe el nodo
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
		int id = vNodes[k]->getId();
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
			id = vNodes[k]->getParent();
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

/*! \fn void Dijkstra::setEditable(bool edit)
 * 	\brief Controls if the graph is being edited
*/
void Dijkstra::setEditable(bool editable)
{
	bEdit = editable;
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
		vNodes[i]->reset();
	}
	return 0;
}

/*! \fn int Dijkstra::setInitialNode(int node)
 * 	\brief Set the initial node
*/
int Dijkstra::setInitialNode(int node)
{
	vNodes[node]->setInitial();
	return 0;
}

/*! \fn int Dijkstra::pushIntoQueue(int node)
 * 	\brief Push a node into the queue
*/
int Dijkstra::pushIntoQueue(int node)
{
	PointerNode pNode(vNodes[node]);
	pqQueue.push(pNode);

	return 0;
}

/*! \fn PointerNode Dijkstra::popFromQueue()
 * 	\brief Pops the node with more priority
 *  \return -1 if the queue is empty
 *  \return 0 if OK
*/
Dijkstra::PointerNode Dijkstra::popFromQueue()
{
	if (pqQueue.empty())
	{
		ROS_ERROR("Dijkstra::PopFromQueue: the queue is empty");
		return 0;
	}

	PointerNode pn = pqQueue.top(); // Get the node
	pqQueue.pop();					// Extracts the node

	return pn;
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
	PointerNode *pNodeMin;
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
		//Extraemos el nodo con distancia mínima de la cola
		PointerNode pNodeMin = popFromQueue();
		//Para todos los nodos adyacentes al extraido calculamos distancias
		for (int i = 0; i < (int)pNodeMin.node->node.arc_list.size(); i++)
		{
			int nodeAdjacent = pNodeMin.node->node.arc_list[i].node_dest;

			int nodeAdjIndex = getNodeIndex(nodeAdjacent);
			if (nodeAdjIndex < 0)
			{
				ROS_ERROR("Dijkstra::calculateRoute: Error: node adjacent %d does not exist", nodeAdjacent);
				return -1;
			}

			// Añadimos al peso la distancia entre los nodos
			diffX = vNodes[nodeAdjIndex]->node.pose.x - pNodeMin.node->node.pose.x;
			diffY = vNodes[nodeAdjIndex]->node.pose.y - pNodeMin.node->node.pose.y;
			distance = sqrt(diffX * diffX + diffY * diffY) * 1000.0;
			int weight = (int)(pNodeMin.node->node.arc_list[i].distance);

			if (!vNodes[nodeAdjIndex]->isUsed())
			{ // Si no hemos utilizado el nodo
				// if distancia[v] > distancia[u] + weight (u, v)
				if (vNodes[nodeAdjIndex]->getDistance() > (pNodeMin.node->getDistance() + weight))
				{
					// distancia[v] = distancia[u] + peso (u, v)
					vNodes[nodeAdjIndex]->setDistance(pNodeMin.node->getDistance() + weight);
					// parent[v] = u
					vNodes[nodeAdjIndex]->setParent(pNodeMin.node->getId());
				}
			} // else
		}
		pNodeMin.node->setUsed(); // The node is used
		updateQueue();
	}
	// Cuando se vacíe la cola llegaremos al final del algoritmo puesto que habremos recorrido todos los nodos conexos del grafo
	if (vNodes[end_node]->getParent() == NO_PARENT)
	{ // Si el nodo final no tiene nodo antecesor es que el grafo no es conexo
		ROS_ERROR("Dijkstra::calculateRoute: Error: nodes %d and %d are not conex", initial_node, end_node);
		return -1;
	}
	return 0;
}

/*! \fn int Dijkstra::getNodeIndex(int nodeID)
 * 	\brief Gets the index of the node using his ID
 *  \return -1 if id does not exist
*/
// TODO Optimizar para que no haya que usar getNodeFromId!!
int Dijkstra::getNodeIndex(int nodeID)
{
	int ret = -1;
	for (int i = 0; i < (int)vNodes.size(); i++)
	{
		if (vNodes[i]->getId() == nodeID)
		{
			ret = i;
			break;
		}
	}
	return ret;
}

/*! \fn int Dijkstra::getArcBetweenNodes(int from_node, int to_node){
 * 	\brief Gets the arc's values between nodes
 *  \return 0 if OK
*/
int Dijkstra::getArcBetweenNodes(int from_node, int to_node)
{
	// La edición debe estar finalizada
	if (bEdit)
	{
		ROS_ERROR("Dijkstra::GetMagnetsBetweenNodes: Edition must be disabled");
		return -2;
	}
	// Comprobación rango de nodos
	if ((from_node > iMaxNodeId) || (from_node < 0) || (to_node > iMaxNodeId) || (to_node < 0))
	{
		ROS_ERROR("Dijkstra::GetMagnetsBetweenNodes: Bad id of nodes: %d to %d", from_node, to_node);
		return -1;
	}
	return 0;
}

/*! \fn void Graph::printZones()
 * 	\brief Print zones of current graph
*/
void Dijkstra::printZones()
{
	if (vZones.size() > 0)
	{
		ROS_INFO("");
		ROS_INFO("Graph::print %d zones:", vZones.size());
		for (int i = 0; i < vZones.size(); i++)
		{
			ROS_INFO("Zone: %i", vZones[i].iIDZone);
			ROS_INFO("  Max num of robots: %i", vZones[i].iMaxRobots);
			ROS_INFO("  %i nodes on this zone:", vZones[i].vpNodes.size());
			for (int j = 0; j < vZones[i].vpNodes.size(); j++)
			{
				ROS_INFO("  - Node %i", vZones[i].vpNodes[j]->node.id);
			}
		}
	}
	else
		ROS_INFO("The graph has 0 zones");
	ROS_INFO("");
}

/*! \fn void Graph::printNodes()
 * 	\brief Print nodes of current graph
*/
void Dijkstra::printNodes()
{
	if (vNodes.size() > 0)
	{
		ROS_INFO("");
		ROS_INFO("Graph::print %d nodes:", vNodes.size());
		for (int i = 0; i < vNodes.size(); i++)
		{
			ROS_INFO("Node %i:", vNodes[i]->node.id);
			ROS_INFO("  Name: %s", vNodes[i]->node.name.c_str());
			ROS_INFO("  Zone: %i", vNodes[i]->node.zone);
			ROS_INFO("  iRobot %i:", vNodes[i]->iRobot);
			ROS_INFO("  iResRobot %i:", vNodes[i]->iResRobot);
			ROS_INFO("  Pose:");
			ROS_INFO("    x: %f", vNodes[i]->node.pose.x);
			ROS_INFO("    y: %f", vNodes[i]->node.pose.y);
			ROS_INFO("    z: %f", vNodes[i]->node.pose.z);
			ROS_INFO("    theta: %f", vNodes[i]->node.pose.theta);
			ROS_INFO("    frame_id: %s", vNodes[i]->node.pose.frame_id.c_str());
			printArcs(vNodes[i]->node);
		}
	}
	else
		ROS_INFO("The graph has 0 nodes");
	ROS_INFO("");
}

/*! \fn void Graph::printArcs()
 * 	\brief Print arcs of current node
*/
void Dijkstra::printArcs(graph_msgs::GraphNode node)
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