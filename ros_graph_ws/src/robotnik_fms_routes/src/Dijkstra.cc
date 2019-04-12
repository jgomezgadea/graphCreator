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
Dijkstra::Dijkstra(){
	bEdit = true;
	iMaxNodeId = iMaxMagnetId = -1;
}

/*! \fn Dijkstra::~Dijkstra()
 * 	\brief Public Destructor
*/
Dijkstra::~Dijkstra(){
	delete rRoutes;
	delete []pNodes;
	//cout << "Dijkstra::~Dijkstra" << endl;
}

/*! \fn int Dijkstra::FinalizeEdition()
 * 	\brief Disables the edition of nodes, edges. Necesario para poder calcular rutas sin incoherencias
 *  \return 0 if graph's elements are OK
*/
int Dijkstra::FinalizeEdition(std::string *msg){
	int max_id = 0;
	bool bNodeFound = false;

	if(vNodes.size() > 0){	// Si no hay nodos no tiene sentido
		// Comprobamos coherencia de los arcos de cada nodo. No pueden haber arcos que apunten a nodos inexistentes.
		// Esta comprobación no se realiza en el momento en que se inserta un nuevo arco, puesto que necesitamos
		// que sea flexible a la hora de leer el grafo desde un archivo xml.
		//Buscamos el mayor Id y creamos una ruta con todos esos nodos(no es muy eficiente en espacio pero podemos reutilizar rRoute)
		for(int i = 0; i < (int)vNodes.size(); i++){
			//
			//Comprobamos si existen los adyacentes que tiene asignados cada nodo
			for(int j = 0; j< (int)vNodes[i].vAdjacent.size(); j++){
				bNodeFound = false;
				for(int k = 0; k < (int)vNodes.size(); k++){
					if(vNodes[i].vAdjacent[j].GetNode() == vNodes[k].GetId()){
						bNodeFound = true;
						break;
					}
				}
				if(!bNodeFound){	// Nodo adyacente no existe
					//cout << "Dijkstra::FinalizeEdition: Error: Arco entre node " << vNodes[i].GetId() << " y node " << vNodes[i].vAdjacent[j].GetNode() << " no existe." << endl;
                    ROS_ERROR("Dijkstra::FinalizeEdition: Error: Arco entre node %d y node %d", vNodes[i].GetId(), vNodes[i].vAdjacent[j].GetNode());
                    *msg = "Dijkstra::FinalizeEdition: Error: Arco entre node:"+std::to_string(vNodes[i].GetId())+" y node:"+ std::to_string(vNodes[i].vAdjacent[j].GetNode());

					return -1;
				}
			}
			if(vNodes[i].GetId() > max_id)
				max_id = vNodes[i].GetId();
		}
	    //	cout << "Dijkstra::FinalizeEdition: Creamos matriz de " << max_id << "x" << max_id << endl;
		bEdit = false;
		rRoutes = new Route(max_id + 1);	//Creamos matriz de posibles rutas
        pNodes = new Node *[max_id + 1];   //Creamos vector de punteros a cada nodo, refereciados por su ID
		iMaxNodeId = max_id;
		//! Enlazamos punteros
		for(int i = 0; i < (int)vNodes.size(); i++){
			pNodes[vNodes[i].GetId()] = &vNodes[i];
			//cout << "Dijkstra::FinalizeEdition: Nodo enlazado: " << pNodes[vNodes[i].GetId()]->GetId() << endl;
		}
	}

	return 0;
}

/*! \fn void Dijkstra::EnableEdition()
 * 	\brief Disables the edition of nodes, edges. Necesario para poder calcular rutas sin incoherencias
*/
void Dijkstra::EnableEdition(){
	bEdit = true;
	iMaxNodeId = iMaxMagnetId = -1;

	delete rRoutes;
	delete []pNodes;
}

/*! \fn int Dijkstra::DeleteNodes()
 * 	\brief Deletes All the nodes
 *  \return 0 if OL
*/
int Dijkstra::DeleteNodes(){
	if(!bEdit){
		ROS_ERROR("Dijkstra::DeleteNodes: Error: Graph's edition must be disabled");
		return -2;
	}

	for(int i = 0; i < (int)vNodes.size(); i++){
		vNodes[i].DeleteAdjacent();
	}
	vNodes.clear();
	iMaxNodeId = -1;

	return 0;
}

/*! \fn int Dijkstra::DeleteAll()
 * 	\brief Deletes all the components
 *  \return 0 if OK
*/
int Dijkstra::DeleteAll(){
	EnableEdition();

	for(int i = 0; i < (int)vNodes.size(); i++){
		vNodes[i].DeleteAdjacent();
	}

	vNodes.clear();
	vMagnets.clear();
	ROS_INFO("Dijkstra::DelteAll: all the data has beee removed");
	return 0;
}

//! Add Swith Map
int Dijkstra::AddSwitchMap(int iNode,bool bSwitch,string sMap, float x, float y, float theta){
    Node *node = GetNodeFromID(iNode);

    node->bSwitchMap=bSwitch;
    node->sSwitchMapName=sMap;
    node->fSwitchMapX=x;
    node->fSwitchMapY=y;
    node->fSwitchMapTheta=theta;
}

//! Add Move Params
int Dijkstra::AddMoveParams(int iNode, bool bMove,float fXGoal,float fYGoal,float fThetaGoal,float fXVel,float fYVel,float fThetaVel){
     Node *node = GetNodeFromID(iNode);

     node->bMove=bMove;
     node->fMoveGoalX=fXGoal;
     node->fMoveGoalY=fYGoal;
     node->fMoveGoalTheta=fThetaGoal;
     node->fMoveMaxVelX=fXVel;
     node->fMoveMaxVelY=fYVel;
     node->fMoveMaxVelTheta=fThetaVel;

}

//! Add Do Nothing
int Dijkstra::AddDoNothing(int iNode,bool bDoNot){
    Node *node = GetNodeFromID(iNode);

    node->bDoNothing=bDoNot;
}

//! Add ElevatorParamstoNode
int Dijkstra::AddElevatorParams(int iNode,bool bElevator,int iID,bool bGetControl,bool bLeaveControl,bool bOpenDoor,bool bCloseDoor,int iFloor){
    Node *node = GetNodeFromID(iNode);

    node->bElevator=bElevator;
    node->iElevatorID=iID;
    node->bElevatorGetControl=bGetControl;
    node->bElevatorLeaveControl=bLeaveControl;
    node->bElevatorCloseDoor=bCloseDoor;
    node->bElevatorOpenDoor=bOpenDoor;
    node->iElevatorFloor=iFloor;

    //ROS_ERROR("Node:%d,ID:%d,Control:%d,Leave:%d,open:%d,close:%d,floor:%d",iNode,bElevator,iID,bGetControl,bLeaveControl,bOpenDoor,bCloseDoor,iFloor);

}


/*! \fn int Dijkstra::AddNode(int node, int type, int floor, double x, double y, double z, char *name ,bool bAcoustic,bool bCritical,bool bFree, bool bStop, int iPathMode, int zone)
 * 	\brief  Adds a new node
 *  \returns The id of the node if OK
*/
//!
int Dijkstra::AddNode(int node, int type, int floor,int laser, double x, double y, double z, double theta,
                      std::string frame, double xOffset, double yOffset, char *name ,
                      bool bAcoustic, bool bCritical, bool bFree, bool bStop, int iPathMode,
                      bool bRecal,bool bCharge,
					  bool bLoad, bool bUnload, bool bPrePick, bool bPrePlace, bool bPostPick, bool bPostPlace,
					  bool bElevator,bool bDoorOpen,
                      bool bDoorClose,int iDoorID,bool bEnterLift,bool bLeaveLift,string sLiftFl,bool bEnterShower, bool bLeaveShower, std::string sShDoor,std::string sShFrom,

                      bool bMagPick,bool bMagPlace,bool bFindMag,bool bLeaveMag,
                      float fFindMagDistance, std::string sFindMagTurnDirection, std::string sLeaveMagTurn,
                      int iMPickPlaceLane,int iMPickPlaceCartPos,std::string sMPickPlaceTurn,bool bMPickPlaceAllowMarkers, std::string sMPickPlaceCartId

                      ){
    if(!bEdit){
        ROS_ERROR("Dijkstra::AddNode: Error: Graph's edition must be disabled");
        return -2;
    }

    int size = vNodes.size();

    for(int i = 0; i < size; i++){
        if(vNodes[i].GetId() == node)   // Nodo con id repetido
            return -1;
    }

    if(node < 0){
        //cout << "Dijkstra::AddNode: el id del nodo debe ser un numero positivo" << endl;
        ROS_ERROR("Dijkstra::AddNode: el id del nodo debe ser un numero positivo");
        return -1;
    }

    //ROS_INFO("DOpen:%d,DOpen:%d,DID:%d",bDoorOpen,bDoorClose,iDoorID);

    //ROS_INFO("Elevator:%d",bElevator);

    Node *new_node = new Node(node, type, floor,laser, x, y, z, theta,frame ,xOffset ,yOffset ,
                              name, bAcoustic, bCritical, bFree, bStop, iPathMode, bRecal,bCharge,
                              bLoad,bUnload,bPrePick,bPrePlace,bPostPick,bPostPlace,
                              bElevator,bDoorOpen,bDoorClose,iDoorID,
                              bEnterLift,bLeaveLift,sLiftFl,bEnterShower,bLeaveShower,sShDoor,sShFrom,
                              bMagPick,bMagPlace,bFindMag,bLeaveMag,
                              fFindMagDistance,sFindMagTurnDirection,sLeaveMagTurn,
                              iMPickPlaceLane,iMPickPlaceCartPos,sMPickPlaceTurn,
                              bMPickPlaceAllowMarkers,sMPickPlaceCartId);

    vNodes.push_back(*new_node);

    return size + 1;
}

/*! \fn int Dijkstra::AddZone(int iIDZone,iMaxRobots){
 * 	\brief Adds a Zone
 *  \return 0 if OK
*/
int Dijkstra::AddZone(int iIDZone,int iMaxRobots,bool bMan,int iNodeDest,int iComp){
    for (int i=0;i<(int)vZones.size();i++){
        if (vZones[i].iIDZone==iIDZone) return -1;
    }
    vZones.push_back(Zone(iIDZone,iMaxRobots,bMan,iNodeDest,iComp));
    return 0;
}

/*! \fn int Dijkstra::GetNodeFromID(int iIDNode){
 * 	\brief Gets Pointer to a Node
 *  \return NULL if Error
*/
Node *Dijkstra::GetNodeFromID(int iIDNode){
    Node *nod=NULL;

    int iNode=GetNodeIndex(iIDNode);
    nod=&vNodes[iNode];
    return nod;

}

/*! \fn int Dijkstra::AddNodetoZone(int iIDNode,int iIDZone){
 * 	\brief Adds Node to a Zone
 *  \return 0 if OK
*/
int Dijkstra:: AddNodeToZone(int iIDNode,int iIDZone,bool bMan){
    Node *nod = GetNodeFromID(iIDNode);
    // Falta comprobar que existen zona y nodo
    if (nod!=NULL) {
        int res=nod->AddZoneToNode(iIDZone);
        //if (res>=0){
            int s=(int)vZones.size();
            for (int i=0; i<s;i++) {
                if (vZones[i].iIDZone==iIDZone){
                    Node *pNode=GetNodeFromID(iIDNode);
                    res=vZones[i].AddNodeToZone(pNode,bMan);
                    if (res==-1)
                        return -1;
                    else
                        return 0;
                }
            }
        //}
        return 0;
    } else {
        return -1;
    }

}

/*! \fn bool Dijkstra::CheckCompZoneFree(int iIDZone,int iIDRobot){
 * 	\brief Checks if zone Free
 *  \return true if free false other case
*/
bool Dijkstra::CheckCompZoneFree(int iIDZone,int iIDRobot){
    //ROS_INFO("Checking Comp Zone:%d for Robot:%d ",iIDZone,iIDRobot);
    int iZone=-1;
    bool bFound=false;
    int iMaxRobots=0;
    bool bPresRobot[50];
    for (int i=0; i<(int)vZones.size();i++){
        if (vZones[i].iIDZone==iIDZone) {
            iMaxRobots=vZones[i].iMaxRobots;
            bFound=true;
            iZone=i;
            break;
        }
    }
    if (!bFound) {
        return true;
    }
    iMaxRobots=1;
    for (int i=0;i<50;i++) {
        bPresRobot[i]=false;
    }
    for (int j=0; j<(int)vZones[iZone].vpNodes.size();j++){
        Node *pNode;
        pNode=vZones[iZone].vpNodes[j];
        if (pNode==NULL) {
            return true;
        }
        //ROS_INFO("Zone:%d,Checking Node:%d robot:%d resrobot:%d,iMaxRobots:%d",iIDZone,pNode->iNode,pNode->iRobot,pNode->iResRobot,iMaxRobots);
        if (iIDRobot>=0){
            if ((pNode->iRobot>=0) && (pNode->iRobot!=iIDRobot))  bPresRobot[pNode->iRobot]=true;
            if ((pNode->iResRobot>=0) && (pNode->iResRobot!=iIDRobot)) bPresRobot[pNode->iResRobot]=true;
        } else {
            //ROS_INFO("1");
            if (pNode->iRobot>=0) {
                //ROS_INFO("2");
                bPresRobot[pNode->iRobot]=true;
            }
            if (pNode->iResRobot>=0) {
                //ROS_INFO("3");
                bPresRobot[pNode->iResRobot]=true;
            }
        }
    }
    int iNR=0;
    for (int i=0;i<50;i++) {
        if (bPresRobot[i]) iNR++;
    }
    //ROS_INFO("Zone:%d,Checking,NR:%d,iMaxRobots:%d",iIDZone,iNR,iMaxRobots);
    if (iNR>=iMaxRobots) {
        //if (iIDRobot>=0) ROS_ERROR("--Comp Zone Block:%d,robot:%d",iIDZone,iIDRobot);
        return false;
    } else {

        return true;
    }
}

/*! \fn bool Dijkstra::CheckZoneFree(int iIDZone,int iIDRobot){
 * 	\brief Checks if zone Free
 *  \return true if free false other case
*/
bool Dijkstra::CheckZoneFree(int iIDZone,int iIDRobot){

    //ROS_INFO("Checking Zone:%d for Robot:%d ",iIDZone,iIDRobot);
    int iZone=-1;
    bool bFound=false;
    int iMaxRobots=0;
    bool bPresRobot[50];
    int iCompZone=-1;
    bool bCompZoneFree=true;
    for (int i=0; i<(int)vZones.size();i++){
        if (vZones[i].iIDZone==iIDZone) {
            iMaxRobots=vZones[i].iMaxRobots;
            bFound=true;
            iZone=i;
            iCompZone=vZones[i].iComplementary;
            //ROS_ERROR("Comp zone:%d,RRRobot:%d",iCompZone,iIDRobot);
            bCompZoneFree=CheckCompZoneFree(iCompZone,iIDRobot);
            break;
        }
    }
    if (!bFound) {
        return true;
    }
    for (int i=0;i<50;i++) {
        bPresRobot[i]=false;
    }
    for (int j=0; j<(int)vZones[iZone].vpNodes.size();j++){
        Node *pNode;
        pNode=vZones[iZone].vpNodes[j];
        if (pNode==NULL) {
            return true;
        }
        //ROS_INFO("Zone:%d,Checking Node:%d robot:%d resrobot:%d,iMaxRobots:%d",iIDZone,pNode->iNode,pNode->iRobot,pNode->iResRobot,iMaxRobots);
        if (iIDRobot>=0){
            if ((pNode->iRobot>=0) && (pNode->iRobot!=iIDRobot))  bPresRobot[pNode->iRobot]=true;
            if ((pNode->iResRobot>=0) && (pNode->iResRobot!=iIDRobot)) bPresRobot[pNode->iResRobot]=true;
        } else {
            //ROS_INFO("1");
            if (pNode->iRobot>=0) {
                //ROS_INFO("2");
                bPresRobot[pNode->iRobot]=true;
            }
            if (pNode->iResRobot>=0) {
                //ROS_INFO("3");
                bPresRobot[pNode->iResRobot]=true;
            }
        }
    }
    int iNR=0;
    for (int i=0;i<50;i++) {
        if (bPresRobot[i]) iNR++;
    }
    //ROS_INFO("Zone:%d,Checking,NR:%d,iMaxRobots:%d",iIDZone,iNR,iMaxRobots);
    if (iNR>=iMaxRobots) {
        return false;
    } else {

        if (bCompZoneFree) return true;
        else return false;
    }
}

/*! \fn int Dijkstra::CheckNodeFree(int iIDNode,int iIDRobot){
 * 	\brief Checks if Node free for use
 *  \return true if free false other case
*/
bool Dijkstra::CheckNodeFree(int iIDNode,int iIDRobot){
    //char cAux[LOG_STRING_LENGTH] = "\0";
    int i=0;
    int iNR=0;
    Node *pNodeOr;
    pNodeOr=GetNodeFromID(iIDNode);

    //ROS_INFO("Checking Node:%d, for robot:%d",iIDNode,iIDRobot);

    if (pNodeOr==NULL) {
        ROS_INFO("Node:%d, Robot %d: NULL",iIDNode,iIDRobot);
        return false;
    }

    //if ((pNodeOr->bBlocked) && (pNodeOr->iRobot!=iIDRobot)) return false;



    int iNZ=pNodeOr->viZones.size();
    for (i=0; i<(int)iNZ;i++){
        int iZone=pNodeOr->viZones[i];
        //ROS_INFO("Checking Zone:%d, Node:%d, for robot:%d",iZone,iIDNode,iIDRobot);
        if (!CheckZoneFree(iZone,iIDRobot)) {
            iNR++;
            //ROS_INFO("---Node:%d, Zone %d: not Free",iIDNode,iZone);
        } else {
            //ROS_INFO("----------------Node:%d, Zone %d: Free",iIDNode,iZone);
        }
    }

    if (iIDRobot>=0){
        if ((pNodeOr->iRobot>=0) && (pNodeOr->iRobot!=iIDRobot)) {
            //ROS_INFO("Node:%d IRobot %d: not Free",iIDNode,pNodeOr->iRobot);
            iNR++;
        }
        if ((pNodeOr->iResRobot>=0) && (pNodeOr->iResRobot!=iIDRobot)) {
            //ROS_INFO("Node:%d IResRobot %d: not Free",iIDNode,pNodeOr->iResRobot);
            iNR++;
        }
    } else {
        if (pNodeOr->iRobot>=0) {
            //ROS_INFO("AAAAA");
            iNR++;
        }
        if (pNodeOr->iResRobot>=0) {
            //ROS_INFO("BBBB");
            iNR++;
        }
    }
    if (iNR>0) {
        return false;
    } else {
        return true;
    }
}



/*! \fn int Dijkstra::AddNode(int node, int type, int floor, double x, double y, double z, char *name ,bool bAcoustic,bool bCritical,bool bFree, bool bStop, int iPathMode, int zone)
 * 	\brief  Adds a new node
 *  \returns The id of the node if OK
*/
//!
/*
int Dijkstra::AddNode(int node, int type, int floor,int laser, double x, double y, double z, char *name , bool bAcoustic, bool bCritical, bool bFree, bool bStop, int iPathMode, int zone,bool bRecal){
	if(!bEdit){
		ROS_ERROR("Dijkstra::AddNode: Error: Graph's edition must be disabled");
		return -2;
	}

	int size = vNodes.size();

	for(int i = 0; i < size; i++){
		if(vNodes[i].GetId() == node)   // Nodo con id repetido
			return -1;
	}

	if(node < 0){
		//cout << "Dijkstra::AddNode: el id del nodo debe ser un numero positivo" << endl;
		ROS_ERROR("Dijkstra::AddNode: el id del nodo debe ser un numero positivo");
		return -1;
	}

	Node *new_node = new Node(node, type, floor,laser, x, y, z, name, bAcoustic, bCritical, bFree, bStop, iPathMode, zone, bRecal); //

	vNodes.push_back(*new_node);

    return size + 1;
}*/

/*! \fn int Dijkstra::AddArc(int from_node, int to_node, double speed, std::vector<int> magnetsOnArc)
 * 	\brief Adds an arc from a node to another with constant weight
 *  No controlamos que el nodo objetivo esté en la lista de nodos, puesto que puede que se introduzca despues
*/
int Dijkstra::AddArc(int from_node, int to_node, double speed, std::vector<int> magnetsOnArc){
	int locatedFrom = -1;   // Id del nodo
	std::vector<Magnet *> vpMagnets; // Punteros a iman
	bool bExistMagnet = false;

    //ROS_INFO("Adding Arc from Node:%d to Node:%d",from_node,to_node);

	if(!bEdit){ // Edición activa
		ROS_ERROR("Dijkstra::AddArc: Error: Graph's edition must be disabled");
		return -2;
	}

	int size = vNodes.size();

	if(size == 0){  //  No hay nodos, no puede añadirse ningún arco
		//cout << "Dijkstra::AddArc: Error: No nodes" << endl;
		ROS_ERROR("Dijkstra::AddArc: Error: No nodes");
		return -1;
	}

	if(from_node == to_node){   // Los ciclos en un mismo nodo no están permitidos
        //ROS_ERROR("Dijkstra::AddArc: Error: graph cycles are not permitted: from %d to %d", from_node, to_node);
		//cout << "Dijkstra::AddArc: Error: graph cycles are not permitted" << endl;
		return -1;
	}

	for(int i = 0; i < size; i++){   // Comprobamos que el nodo  origen exista
		if(vNodes[i].GetId() == from_node) //Existe el nodo
			locatedFrom = i;
	}
	if( locatedFrom < 0 ){
		//cout << "Dijkstra::AddArc: Error: Incorrect node number "<< from_node << endl;
		ROS_ERROR("Dijkstra::AddArc: Error: Incorrect node number (%d)", from_node);

		return -1;
	}
	//
	// Buscamos que los imanes del arco existan y los enlazamos (puntero) con el arco
	for(int i = 0; i < (int)magnetsOnArc.size(); i++){
		bExistMagnet = false;
		for(int j = 0; j < (int)vMagnets.size(); j++){
			if(magnetsOnArc[i] == vMagnets[j].iNumber){
				bExistMagnet = true;
				vpMagnets.push_back(&vMagnets[j]);
			}
		}
		if(!bExistMagnet){  //El magnet no existe
            //cout << "Dijkstra::AddArc: Magnet " << magnetsOnArc[i] << " from node " <<
            //	from_node << " to " << to_node <<  " does not exist." << endl;
			ROS_ERROR("Dijkstra::AddArc: Magnet %d from node %d to %d does not exist", magnetsOnArc[i], from_node, to_node);
			return -1;
		}
	}

	return vNodes[locatedFrom].AddNodeAdjacent(to_node, speed, magnetsOnArc, vpMagnets);
}

/*! \fn int Dijkstra::AddArc(int from_node, int to_node, double speed, int weight, std::vector<int> magnetsOnArc)
 * 	\brief Adds an arc from a node to another with weight
*/
int Dijkstra::AddArc(int from_node, int to_node, double speed, int weight, std::vector<int> magnetsOnArc){
	int locatedFrom = -1;   //Id del nodo
	std::vector<Magnet *> vpMagnets; // Punteros a iman
	bool bExistMagnet = false;

	if(!bEdit){
		return -2;
	}

	int size = vNodes.size();

	if(size == 0){
		//cout << "Dijkstra::AddArc: Error: No nodes" << endl;
		ROS_ERROR("Dijkstra::AddArc: Error: No nodes");
		return -1;
	}

	if(from_node == to_node){
        //ROS_ERROR("Dijkstra::AddArc: Error: graph cycles are not permitted: from %d to %d", from_node, to_node);
		//cout << "Dijkstra::AddArc: Error: graph cycles are not permitted" << endl;
		return -1;
	}

	for(int i= 0; i < size; i++){
		if(vNodes[i].GetId() == from_node) //Existe el nodo
			locatedFrom = i;
	}

	if( locatedFrom < 0 ){
		ROS_ERROR("Dijkstra::AddArc: Error: Incorrect node number (%d)", from_node);
		//cout << "Dijkstra::AddArc: Error: Incorrect node number "<< from_node << endl;
		return -1;
	}
	//
	// Buscamos que los imanes del arco existan y los enlazamos (puntero) con el arco
	for(int i = 0; i < (int)magnetsOnArc.size(); i++){
		bExistMagnet = false;
		for(int j = 0; j < (int)vMagnets.size(); j++){
			if(magnetsOnArc[i] == vMagnets[j].iNumber){
				bExistMagnet = true;
				vpMagnets.push_back(&vMagnets[j]);
			}
		}
		if(!bExistMagnet){  //El magnet no existe
			//cout << "Dijkstra::AddArc: Magnet " << magnetsOnArc[i] << " from node " <<
			//	from_node << " to " << to_node <<  " does not exist." << endl;
			ROS_ERROR("Dijkstra::AddArc: Magnet %d from node %d to %d does not exist", magnetsOnArc[i], from_node, to_node);
			return -1;
		}
	}

	return vNodes[locatedFrom].AddNodeAdjacent(to_node, speed, weight, magnetsOnArc, vpMagnets);
}

/*! \fn int Dijkstra::DeleteArc(int from_node, int to_node)
 * 	\brief Deletes selected arc
 *  \return 0 if OK
*/
int Dijkstra::DeleteArc(int from_node, int to_node){
	/*if(!bEdit){
		return -2;
	}
	int size = vNodes.size();

	if( size == 0){
		cout << "Dijkstra::AddArc: Error: No nodes" << endl;
		return -1;
	}

	if( (from_node < 0) || (from_node > size - 1) || (to_node < 0) || (to_node > size - 1) ){
		cout << "Dijkstra::AddArc: Error: Incorrect nodes number" << endl;
		return -1;
	}

	return vNodes[from_node].DeleteAdjacent(to_node);*/
	return 0;
}

/*! \fn int Dijkstra::DeleteArcs(int from_node)
 * 	\brief Deletes all the arc from the node
 *  \return 0 if OK
*/
int Dijkstra::DeleteArcs(int from_node){
	/*if(!bEdit){
		return -2;
	}
	int size = vNodes.size();

	if( size == 0){
		cout << "Dijkstra::AddArc: Error: No nodes" << endl;
		return -1;
	}

	if( (from_node < 0) || (from_node > size -1)  ){
		cout << "Dijkstra::AddArc: Error: Incorrect nodes number" << endl;
		return -1;
	}

	return vNodes[from_node].DeleteAdjacent();*/
	return 0;
}

/*! \fn int Dijkstra::ResetRoutes()
 * 	\brief Reset the calculated routes
 *  \return 0 if OK
*/
int Dijkstra::ResetRoutes(){
	if(bEdit){
	//	cout << "Dijkstra::GetRoute: Error: Graph's edition must be disabled" << endl;
		ROS_ERROR("Dijkstra::ResetRoutes: Error: Graph's edition must be disabled");
		return -2;
	}
	rRoutes->Reset();
	return 0;
}


/*! \fn int Dijkstra::GetNearestNodeID(double x, double y)
 * 	\brief return Nearest Node ID
*/
int Dijkstra::GetNearestNodeID(double x, double y,std::string frame){

    double dist=999999999.9;
    int current_node=-1;
    for(int in = 0; in < vNodes.size(); in++){
        if (vNodes[in].dZ==0){
            if (vNodes[in].sFrame_id==frame){
                double new_dist=sqrt(((vNodes[in].dX-x)*(vNodes[in].dX-x))+
                                     ((vNodes[in].dY-y)*(vNodes[in].dY-y)));

                if (new_dist<=dist) {
                    dist=new_dist;
                    current_node=vNodes[in].iNode;
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
bool Dijkstra::ReserveNode(int iRobot, int iIDNode){
    for(int in = 0; in < vNodes.size(); in++){
        if (in==iIDNode){
            //if (vNodes[in].iResRobot!=iRobot) ROS_INFO("Reserve Node:%d for Robot:%d",iIDNode,iRobot);
            vNodes[in].iResRobot=iRobot;
        } else {
            if (vNodes[in].iResRobot==iRobot) {
                //ROS_INFO("%d,UnReserve Node:%d for Robot:%d",in,iIDNode,iRobot);
                vNodes[in].iResRobot=-1;
            }
        }
    }
    return true;
}

/*! \fn int Dijkstra::UnBlockAll(int iRobot)
 * 	\brief Unblock All Nodes for Robot
*/
bool Dijkstra::UnBlockAll(int iRobot){
    for(int in = 0; in < vNodes.size(); in++){
        if (vNodes[in].iResRobot==iRobot) {
            ROS_INFO("UnBlock Node:%d for Robot:%d",in,iRobot);
            vNodes[in].iRobot=-1;
            //vNodes[in].bBlocked=false;
        }
    }
    return true;
}

/*! \fn int Dijkstra::GetNodesUsed(std::vector<int> *route)
 * 	\brief Gets the list of nodes used or blocked
 *  \return 0 if OK
*/
bool Dijkstra::GetNodesUsed(std::vector<Node *> *route){
    for(int in = 0; in < vNodes.size(); in++){
        //ROS_INFO("N:%d->resNode:%d",in,vNodes[in].iResRobot);
        bool bAdd=false;
        if (vNodes[in].iRobot>=0) {
            bAdd=true;
        }
        /*
        if (vNodes[in].bBlocked) {
            bAdd=true;
        }
        if (vNodes[in].bReserved) {
            bAdd=true;
        }*/
        if (vNodes[in].iResRobot>=0) {
            bAdd=true;
        }
        for (int iz=0;iz<vNodes[in].viZones.size();iz++){
            //ROS_INFO(" aaaa Checking Zone:%d Node:%d",vNodes[in].viZones[iz],vNodes[in].iNode);
            if (!CheckZoneFree(vNodes[in].viZones[iz],-1)){
                //ROS_INFO(" aaaa Zone USED:%d",vNodes[in].viZones[iz]);
                bAdd=true;
            } else {
                //ROS_INFO(" aaaa Zone NOT USED:%d",vNodes[in].viZones[iz]);
            }
        }
        if (bAdd) {
            route->push_back(&vNodes[in]);
            //ROS_INFO("Add N:%d",in);
        }
    }

}

/*! \fn int Dijkstra::GetRoute(int initial_node, int end_node, std::vector<int> *route)
 * 	\brief Gets the best calculated route between selected nodes
 *  \return 0 if OK
*/
int Dijkstra::GetRoute(int initial_node, int end_node, std::vector<int> *route){
	int positionInitialNode = -1, positionEndNode = -1;

	if(bEdit){
		//cout << "Dijkstra::GetRoute: Error: Graph's edition must be disabled" << endl;
		ROS_ERROR("Dijkstra::GetRoute: Error: Graph's edition must be disabled");
		return -2;
	}

	if( (initial_node < 0) || (end_node < 0) ){
		//cout << "Dijkstra::GetRoute: Error: node id must be greater than 0" << endl;
		ROS_ERROR("Dijkstra::GetRoute: Error: node id must be greater than 0");
		return -1;
	}
	int size = vNodes.size();

	//Buscamos las posiciones dentro del vector, si es que existen los nodos
	for(int i = 0; i < size; i++){
		if(vNodes[i].GetId() == initial_node) //Existe el nodo
			positionInitialNode = i;
		else if(vNodes[i].GetId() == end_node) //Existe el nodo
			positionEndNode = i;
	}
	if(positionInitialNode < 0){	// El nodo inicial no existe
		ROS_ERROR("Dijkstra::GetRoute: Error: initial  node %d does not exist",  initial_node );
		//cout << "Dijkstra::GetRoute: Error: initial  node "<<  initial_node <<" does not exist." << endl;
		return -1;
	}
	if(positionEndNode < 0){	// El nodo final no existe
		//cout << "Dijkstra::GetRoute: Error: end  node "<<  end_node <<" does not exist." << endl;
		ROS_ERROR("Dijkstra::GetRoute: Error: end node %d does not exist(initial = %d)",  end_node, initial_node);
		return -1;
	}

	//cout << "Dijkstra::GetRoute: Posicion nodo "<< initial_node << " en " <<  positionInitialNode <<
	//	 ", posicion nodo " << end_node <<" en " << positionEndNode << endl;

	if(rRoutes->ExistRoute(initial_node, end_node)){	//Si la ruta existe la devolvemos directamente
		//cout << "Dijkstra::GetRoute: Route exists" << endl;
		//rRoutes->Print(initial_node, end_node);
		*route = rRoutes->GetRoute(initial_node, end_node);
		return 0;
	}

	// si no existe la ruta la calculamos
	//cout << "Dijkstra::GetRoute: Calculamos ruta" << endl;
	if(CalculateRoute(positionInitialNode, positionEndNode) == 0){
		//Vamos añadiendo los nodos a la ruta
		int k = positionEndNode;
		int id = vNodes[k].GetId();
		while( id != NO_PARENT ){ // Recorremos la ruta de manera inversa hasta llegar al nodo inicial, cuyo padre es null
			//cout << "Dijkstra::GetRoute: index = " << k <<  ", ID = " << id << endl;
			if(rRoutes->AddNode(initial_node, end_node, id) != 0){	//Los dos primeros parámetros indentifican la ruta, el último añade los nodos por donde transcurre la ruta
				//cout << "Dijkstra::GetRoute: Error adding the node " << id << " on route (" << initial_node << ", " << end_node << ")" << endl;
				ROS_ERROR("Dijkstra::GetRoute: Error adding the node %d on route (%d, %d)",id, initial_node, end_node);

			}
			if(rRoutes->AddNode(initial_node, end_node, vNodes[k]) != 0){	//Los dos primeros parámetros indentifican la ruta, el último añade los nodos por donde transcurre la ruta
				//cout << "Dijkstra::GetRoute: Error adding the node " << id << " on route (" << initial_node << ", " << end_node << ")" << endl;
				ROS_ERROR("Dijkstra::GetRoute: Error adding the node %d on route (%d, %d)",id, initial_node, end_node);

			}
			id = vNodes[k].GetParent();
			k = GetNodeIndex(id);
		}
		//Ruta añadida
		//rRoutes->Print(initial_node, end_node);
		*route = rRoutes->GetRoute(initial_node, end_node);
		return 0;
	}else {
		//cout << "Dijkstra::GetRoute: Error: CalculateRoute failed" << endl;
		ROS_ERROR("Dijkstra::GetRoute: Error: CalculateRoute failed");
		return -1;
	}
}

/*! \fn int Dijkstra::GetRoute(int initial_node, int end_node, std::vector<Node> *route)
 * 	\brief Gets the best calculated route between selected nodes
 *  \return 0 if OK. And the the array with the nodes of the route
*/
int Dijkstra::GetRoute(int initial_node, int end_node, std::vector<Node> *route){
	int positionInitialNode = -1, positionEndNode = -1;

    //cout << "DIJKSTRA GET ROUTE from " << initial_node << " to " << end_node << endl;

	if(bEdit){
		//cout << "Dijkstra::GetRoute: Error: Graph's edition must be disabled" << endl;
		ROS_ERROR("Dijkstra::GetRoute: Error: Graph's edition must be disabled");
		return -2;
	}

	if( (initial_node < 0) || (end_node < 0) ){
		//cout << "Dijkstra::GetRoute: Error: node id must be greater than 0" << endl;
		ROS_ERROR("Dijkstra::GetRoute: Error: node id must be greater than 0");
		return -1;
	}
	int size = vNodes.size();

	//Buscamos las posiciones dentro del vector, si es que existen los nodos
	for(int i = 0; i < size; i++){
		if(vNodes[i].GetId() == initial_node) //Existe el nodo
			positionInitialNode = i;
		else if(vNodes[i].GetId() == end_node) //Existe el nodo
			positionEndNode = i;
	}
	if(positionInitialNode < 0){	// El nodo inicial no existe
		ROS_ERROR("Dijkstra::GetRoute: Error: initial  node %d does not exist",  initial_node );
		//cout << "Dijkstra::GetRoute: Error: initial  node "<<  initial_node <<" does not exist." << endl;
		return -1;
	}
	if(positionEndNode < 0){	// El nodo inicial no existe
		//cout << "Dijkstra::GetRoute: Error: end  node "<<  end_node <<" does not exist." << endl;
		ROS_ERROR("Dijkstra::GetRoute: Error: end  node %d does not exist",  end_node );
		return -1;
	}

	//cout << "Dijkstra::GetRoute: Posicion nodo "<< initial_node << " en " <<  positionInitialNode <<
	//	 ", posicion nodo " << end_node <<" en " << positionEndNode << endl;

	if(rRoutes->ExistRoute(initial_node, end_node)){	//Si la ruta existe la devolvemos directamente
		*route = rRoutes->GetDetailRoute(initial_node, end_node);
		//cout << "Dijkstra::GetRoute: Route exists. size = " << route->size() << endl;
		//rRoutes->Print(initial_node, end_node);


		return 0;
	}

	// si no existe la ruta la calculamos
	//cout << "Dijkstra::GetRoute: Calculamos ruta" << endl;
	if(CalculateRoute(positionInitialNode, positionEndNode) == 0){
		//Vamos añadiendo los nodos a la ruta
		int k = positionEndNode;
		int id = vNodes[k].GetId();
		while( id != NO_PARENT ){ // Recorremos la ruta de manera inversa hasta llegar al nodo inicial, cuyo padre es null
			//cout << "Dijkstra::GetRoute: index = " << k <<  ", ID = " << id << endl;
			if(rRoutes->AddNode(initial_node, end_node, vNodes[k]) != 0){	//Los dos primeros parámetros indentifican la ruta, el último añade los nodos por donde transcurre la ruta
				//cout << "Dijkstra::GetRoute: Error adding the node " << id << " on route (" << initial_node << ", " << end_node << ")" << endl;
				ROS_ERROR("Dijkstra::GetRoute: Error adding the node %d on route (%d, %d)",id, initial_node, end_node);
			}
			if(rRoutes->AddNode(initial_node, end_node, id) != 0){	//Los dos primeros parámetros indentifican la ruta, el último añade los nodos por donde transcurre la ruta
				//cout << "Dijkstra::GetRoute: Error adding the node " << id << " on route (" << initial_node << ", " << end_node << ")" << endl;
				ROS_ERROR("Dijkstra::GetRoute: Error adding the node %d on route (%d, %d)",id, initial_node, end_node);
			}
			id = vNodes[k].GetParent();
			k = GetNodeIndex(id);
		}
		//Ruta añadida
		//rRoutes->Print(initial_node, end_node);
		*route = rRoutes->GetDetailRoute(initial_node, end_node);
		//cout << "Nodes" << endl; // route->size() <<
		return 0;
	}else {
		//cout << "Dijkstra::GetRoute: Error: CalculateRoute failed" << endl;
		ROS_ERROR("Dijkstra::GetRoute: Error: CalculateRoute failed");
		return -1;
	}
}

/*! \fn int Dijkstra::ResetQueue()
 * 	\brief Resets the priority queue
*/
int Dijkstra::ResetQueue(){
	while (!pqQueue.empty())	 //Mientras la cola no esté vacia, vamos extrayendo elementos
		 pqQueue.pop();
  	return 0;
}

/*! \fn int Dijkstra::ResetNodes()
 * 	\brief Resets the values of every node
*/
int Dijkstra::ResetNodes(){
	for(int i = 0; i < (int)vNodes.size(); i++){
		vNodes[i].Reset();
	}

	return 0;
}

/*! \fn int Dijkstra::SetInitialNode(int node)
 * 	\brief Set the initial node
*/
int Dijkstra::SetInitialNode(int node){
	vNodes[node].SetInitial();
	return 0;
}

/*! \fn int Dijkstra::PushIntoQueue(int node)
 * 	\brief Push a node into the queue
*/
int Dijkstra::PushIntoQueue(int node){
	PointerNode pNode(&vNodes[node]);
	pqQueue.push(pNode);

	return 0;
}

/*! \fn int Dijkstra::PopFromQueue(PointerNode *pn)
 * 	\brief Pops the node with more priority
 *  \return -1 if the queue is empty
 *  \return 0 if OK
*/
int Dijkstra::PopFromQueue(PointerNode *pn){
	if(pqQueue.empty()){
		//cout << "Dijkstra::PopFromQueue: the queue is empty" << endl;
		ROS_ERROR("Dijkstra::PopFromQueue: the queue is empty");
		return -1;
	}

	*pn = pqQueue.top();				// Get the node
	//cout << "Dijkstra::PopFromQueue: top = " << pn->node->GetId() << " dist = " << pn->node->Distance() << endl;
	pqQueue.pop();						// Extracts the node

	return 0;
}

/*! \fn int Dijkstra::QueueNodes()
 * 	\brief Queues all the nodes
*/
int Dijkstra::QueueNodes(){
	for(int i=0; i < (int)vNodes.size(); i++){
		PushIntoQueue(i);
	}
	return 0;
}

/*! \fn void Dijkstra::UpdateQueue()
 * 	\brief Updates the queue after modifiying values of the nodes
*/
void Dijkstra::UpdateQueue(){
	vector <PointerNode> vPN;

	while(!pqQueue.empty()){			//Extraemos todos los nodos
		vPN.push_back(pqQueue.top());
		pqQueue.pop();
	}
	for(int i=0; i < (int)vPN.size(); i++){	//Los volvemos a encolar
		pqQueue.push(vPN[i]);
	}

}

/*! \fn int Dijkstra::CalculateRoute(int initial_node, int end_node)
 * 	\brief Calculates the best route
 *  \return 0 if OK
*/
int Dijkstra::CalculateRoute(int initial_node, int end_node){
	PointerNode pNodeMin;
	double diffX = 0.0;	// Lo utilizaremos para el cálculo de la distancia en X entre dos nodos
	double diffY = 0.0; // Lo utilizaremos para el cálculo de la distancia en Y entre dos nodos
	double distance = 0.0;

	ResetNodes();
	SetInitialNode(initial_node);
	ResetQueue();

	/// Empezamos algoritmo de Dijkstra propiamente dicho
	QueueNodes();	//Encolamos todos los nodos del grafo

	//cout << "Dijkstra::CalculateRoute: Traza: " << pqQueue.size() << " nodos encolados" << endl;
	while(!pqQueue.empty()){	//Mientras la cola no esté vacía
		if( PopFromQueue(&pNodeMin) == 0){	//Extraemos el nodo con distancia mínima de la cola
			//cout << "\t Extraemos nodo " << pNodeMin.node->GetId() << " .Quedan " << pqQueue.size() << endl;
			//Para todos los nodos adyacentes al extraido calculamos distancias
			for(int i= 0; i < (int)pNodeMin.node->vAdjacent.size(); i++){
				int nodeAdjacent = pNodeMin.node->vAdjacent[i].GetNode();

				int nodeAdjIndex = GetNodeIndex(nodeAdjacent);
				if(nodeAdjIndex < 0){
					ROS_ERROR("Dijkstra::CalculateRoute: Error: node adjacent %d does not exist", nodeAdjacent);
					//cout << "Dijkstra::CalculateRoute: Error: node adjacent " << nodeAdjacent << " does not exist" << endl;
					return -1;
				}
				//
				// Añadimos al peso la distancia entre los nodos
				diffX = vNodes[nodeAdjIndex].dX - pNodeMin.node->dX;
				diffY = vNodes[nodeAdjIndex].dY - pNodeMin.node->dY;
				distance = sqrt( diffX * diffX + diffY * diffY )*1000.0;
				//int weight = (int) (pNodeMin.node->vAdjacent[i].GetWeight() + distance);
				int weight = (int) (pNodeMin.node->vAdjacent[i].GetWeight());

			//	cout << "\t\tAdyacente: nodo " <<  nodeAdjacent << ", Peso " << weight << ", Padre " << vNodes[nodeAdjacent].GetParent() << endl;
				if(!vNodes[nodeAdjIndex].IsUsed()){	//Si no hemos utilizado el nodo
					if( vNodes[nodeAdjIndex].Distance() > (pNodeMin.node->Distance() + weight) ){// distancia[v] > distancia[u] + peso (u, v)
						vNodes[nodeAdjIndex].SetDistance(pNodeMin.node->Distance() + weight);	// distancia[v] = distancia[u] + peso (u, v)
						vNodes[nodeAdjIndex].SetParent(pNodeMin.node->GetId());					// padre[v] = u
			//			cout << "\t\t\tNueva distancia " << vNodes[nodeAdjacent].Distance() << " y padre " << vNodes[nodeAdjacent].GetParent() << endl;
					}
				}//else
				//	cout << "\t\t\tNodo Utilizado!!" << endl;
			}
			pNodeMin.node->Used();	// Marcamos el nodo como utilizado
			UpdateQueue();
		}
		//cout << endl;
	}
	//Cuando se vacíe la cola llegaremos al final del algoritmo puesto que habremos recorrido todos los nodos conexos del grafo
	if(vNodes[end_node].GetParent() == NO_PARENT){	//Si el nodo final no tiene nodo antecesor es que el grafo no es conexo
		//cout << "Dijkstra::CalculateRoute: Error: nodes " << initial_node << " and " << end_node << " are not conex" << endl;
		ROS_ERROR("Dijkstra::CalculateRoute: Error: nodes %d and %d are not conex",initial_node, end_node);
		return -1;
	}
	//cout << "Dijkstra::CalculateRoute: Ruta completada" << endl;
	return 0;
}

/*! \fn void Dijkstra::PrintNodes()
 * 	\brief Prints current nodes
*/
void Dijkstra::PrintNodes(){
	double x = 0.0, y = 0.0, z = 0.0;
	int size = vNodes.size();

	if(size > 0){
		cout << "Dijkstra::PrintNodes: " << size << " nodes" << endl;
		for(int i= 0; i<size; i++){
			cout << "\tNode " << vNodes[i].GetId() << ": Dist = " << vNodes[i].iDist << " , parent = " << vNodes[i].iParent << endl;
			cout << "\tName = " << vNodes[i].GetName() << ", Floor = " << vNodes[i].GetFloor() << ", Type = " << vNodes[i].GetType()  << endl;
			cout << "\tAcoustic = " << vNodes[i].bAcoustic << ", Critical = " <<  vNodes[i].bCritical << ", Stop = " <<  vNodes[i].bStop << ", Free = " <<  vNodes[i].bFree << endl;
			vNodes[i].GetPosition(&x, &y, &z);
			cout << "\tPosition: x = " << x << " y = " << y <<  " z = " << z << endl;
 			vNodes[i].PrintArcs();
			cout << endl;
		}
	}else
		cout << "Dijkstra::PrintNodes: No nodes.." << endl;
}

/*! \fn int Dijkstra::GetNodeIndex(int nodeID)
 * 	\brief Gets the index of the node using his ID
 *  \return -1 if id does not exist
*/
int Dijkstra::GetNodeIndex(int nodeID){
	int ret = -1;
	for(int i = 0; i < (int)vNodes.size(); i++){
		if(vNodes[i].GetId() == nodeID){
			ret = i;
			break;
		}
	}
	return ret;
}

/*! \fn int Dijkstra::AddMagnet(int id, double x, double y, double z)
 * 	\brief Adds a new magnet
 *  \return 0 if OK
*/
int Dijkstra::AddMagnet(int id, double x, double y, double z){

	if(!bEdit){
		return -2;
	}

	//En primer lugar comprobamos que el magnet no esté ya en la lista
	for(int i = 0; i < (int)vMagnets.size(); i++){
		if(vMagnets[i].iNumber == id){  //Magnet repetido
			//cout << "Dijkstra::AddMagnet: Magnet " << id << " already exists" << endl;
			ROS_ERROR("Dijkstra::AddMagnet: Magnet %d already exists", id);
			return -1;
		}
	}
	if(id < 0){
		//cout << "Dijkstra::AddMagnet: Error: id tiene que ser positivo: " << id << endl;
		ROS_ERROR("Dijkstra::AddMagnet: Error: id tiene que ser positivo: %d", id );
		return -1;
	}

	if(id > iMaxMagnetId)   // Guardamos el valor del id de valor máximo
		iMaxMagnetId = id;

	vMagnets.push_back(Magnet(id, x, y, z));	//Insertamos iman
	return 0;
}

/*! \fn void Dijkstra::PrintMagnets()
 * 	\brief Prints current magnets
*/
void Dijkstra::PrintMagnets(){
	//double x = 0.0, y = 0.0, z = 0.0;
	int size = vMagnets.size();

	if(size > 0){
		cout << "Dijkstra::Magnets: " << size << endl;
		for(int i= 0; i<size; i++){
			cout << "\tMagnet " << vMagnets[i].iNumber << endl;
			cout << "\t Position: x = " << vMagnets[i].dX << " y = " << vMagnets[i].dY <<  " z = " << vMagnets[i].dZ << endl;
			cout << endl;
		}
	}else
		cout << "Dijkstra::PrintMagnets: No magnets.." << endl;
}

/*! \fn int Dijkstra::GetNodePosition(int node_id, double *x, double *y, double *z)
 * 	\brief Gets the node's position with this id
 *  \return 0 if OK
*/
int Dijkstra::GetNodePosition(int node_id, double *x, double *y, double *z){
	if(bEdit){
		//cout << "Dijkstra::GetNodePosition: Edition must be disabled" << endl;
		ROS_ERROR("Dijkstra::GetNodePosition: Edition must be disabled");
		return -2;
	}

	if( (node_id > iMaxNodeId) || (node_id < 0)){
		ROS_ERROR("Dijkstra::GetNodePosition: node %d does not exist", node_id);
		return -1;
	}

	pNodes[node_id]->GetPosition(x, y, z);
	//cout << "Dijkstra::GetNode: node " << node_id << " ID = " << node->GetId() << endl;
	return 0;
}

/*! \fn int Dijkstra::GetArcBetweenNodes(int from_node, int to_node, std::vector<Magnet> *magnets_on_arc, double *speed){
 * 	\brief Gets the arc's values between nodes
 *  \return 0 if OK
*/
int Dijkstra::GetArcBetweenNodes(int from_node, int to_node, std::vector<Magnet> *magnets_on_arc, double *speed){
	// La edición debe estar finalizada
	if(bEdit){
		//cout << "Dijkstra::GetMagnetsBetweenNodes: Edition must be disabled" << endl;
		ROS_ERROR("Dijkstra::GetMagnetsBetweenNodes: Edition must be disabled");
		return -2;
	}
	// Comprobación rango de nodos
	if( (from_node > iMaxNodeId) || (from_node < 0) ||  (to_node > iMaxNodeId) || (to_node < 0) ){
	//	cout << "Dijkstra::GetMagnetsBetweenNodes: Bad number of nodes" << endl;
		ROS_ERROR("Dijkstra::GetMagnetsBetweenNodes: Bad id of nodes: %d to %d", from_node, to_node );
		return -1;
	}
	// Obtenemos imanes desde un nodo al otro
	if(pNodes[from_node]->GetArcToAdjacent(to_node, magnets_on_arc, speed) != 0){ // Obtenemos los imanes en la ruta al nodo adyacente
		//cout << "Dijkstra::GetMagnetsBetweenNodes: Error getting the magnets from " << from_node << "to " << to_node << endl;
		ROS_ERROR("Dijkstra::GetMagnetsBetweenNodes: Error getting the magnets from %d to %d", from_node, to_node );
		return -1;
	}

	return 0;
}


/*! \fn Node* Dijkstra::GetNode(unsigned int nodeID)
 * 	\brief Gets Node by nodeID
*/
Node* Dijkstra::GetNode(unsigned int node_id){
    //if( (node_id > iMaxNodeId) || (node_id < 0)){
    //	ROS_ERROR("Dijkstra::GetNode: node %d does not exist", node_id);
    //	return NULL;
    //}
    //ROS_ERROR("Dijkstra::GetNode: node %d  ", node_id);

    for (int i=0; i<vNodes.size(); i++){
        if (vNodes[i].iNode==node_id) return &vNodes[i];
        //ROS_ERROR("Dijkstra::GetNode: node %d, pNodes[i]->iNode:%d  ", node_id,vNodes[i].iNode);
    }
    //ROS_ERROR("Dijkstra::GetNode: node %d  NULL ", node_id);
    return NULL;
}
