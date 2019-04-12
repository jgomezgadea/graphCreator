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

#define INFINITE	99999999
#define NO_PARENT	-99999

#define MAX_STRING_LENGTH 300

//! Class used for storing all the magnets in the system
class Magnet {
	public:
	//! Id for the magnet
	int iNumber;
	//! Position of the magnet
	double dX, dY, dZ;
	//! Constructor
	Magnet(int id, double x, double y, double z){
		iNumber = id;
		dX = x;
		dY = y;
		dZ = z;
	}
	//! Destructor
	~Magnet(){

	}
};

//! Class Node utilizada por Dijstra para representar los nodos del grafo
class Node{

//! Class Arc utilizada por Node para indicar el peso y al nodo al que está conectado
	class Arc {
	public:
		//! Peso de la arista
		int iWeight;
		//! Nodo con el conecta la arista
		int iNextNode;
		//! velocidad máxima en ese arco (m/s)
		double dSpeed;
		//! Vector con la lista de imanes (ID) sobre los que pasa el arco
		std::vector<int> vMagnets;
		//! Vector de punteros a iman, con el mismo orden que vMagnets
		std::vector<Magnet *> vpMagnets;
	public:
		//! Constructor
		Arc(int next_node, double speed){
			iWeight = 1;		//Peso por defecto
			iNextNode = next_node;
			dSpeed = speed;	   //Velocidad lenta por defecto
		};
		//! Constructor
		Arc(int next_node, int weight, double speed){
			iWeight = weight;	//Peso asignado
			iNextNode = next_node;
			dSpeed = speed;
		};
		//! Destructor
		~Arc(){
			//std::cout << "Arc::~Arc:" << std::endl;
		};
		//! Establece la velocidad en ese arco
		int SetSpeed(double value){
			dSpeed = value;
			return 0;
		}
		//! Gets the node connected
		int GetNode(){
			return iNextNode;
		}
		//! Gets the weight of the edge
		int GetWeight(){
			return iWeight;
		}
		//! Adds a new magnet
		//! return -1 if the magnet already exists
		//! return 0 if OK
		int AddMagnet(int magnet_id, Magnet * magnet_pointer){
			for(int i= 0; i < (int)vMagnets.size(); i++){
				if(vMagnets[i] == magnet_id)
					return -1;
			}
			vMagnets.push_back(magnet_id);
			vpMagnets.push_back(magnet_pointer);
			return 0;
		}
		//! Adds the magnet from a vector
		int AddMagnets(std::vector <int> magnetList, std::vector <Magnet *> magnetPointerList){
			if(magnetList.size()!= magnetPointerList.size())	//Tienen que ser de la misma talla
				return -1;
			for(int i= 0; i < (int)magnetList.size(); i++){
				vMagnets.push_back(magnetList[i]);			// Añadimos número de imán
				vpMagnets.push_back(magnetPointerList[i]);  // Añadimos puntero a dicho imán
			}
			return 0;
        }
	};

public:
	//! Identificador/Número de nodo
	int iNode;
	//! Distancia al nodo
	int iDist;
	//! Nodo antecesor en la ruta
	int iParent;
	//! Flag para marcar cuando se utiliza el nodo para el cálculo de distancias (solo se utiliza una vez), para no volverlo a encolar
	bool bUsed;
	//! vector de arcos a nodos adyacentes
	std::vector<Arc> vAdjacent;
    //! vector de Zonas
    std::vector<int> viZones;
	//! Coordenadas del nodo
	double dX, dY, dZ;
    //! Theta
    double dTheta;
    //! Frame
    std::string sFrame_id;
    //! Offset
    double dXOffset,dYOffset;
	//! Tipo de nodo
	int iType;
	//! Piso del nodo
	int iFloor;
	//! Laser Range
	int iLaser_Range;
	//! Nombre del nodo
	char cName[MAX_STRING_LENGTH];
	//! Nodo Acustico
	bool bAcoustic;
	//! Nodo Libre
	bool bFree;
	//! Nodo Critico
	bool bCritical;
	//! Nodo Para Permitida
	bool bStop;
	//! Modo de avance
	int iPathMode;
	//! Habilita/Deshabilita recalibración ms20
	bool bRecalibration;

    //! bLoad Node
    bool bLoad;
    //! bUnload Node
    bool bUnload;
	//! bPrePick Node
	bool bPrePick;
	//! bPrePlace Node
	bool bPrePlace;
	//! bPostPick Node
	bool bPostPick;
	//! bPostPlace Node
	bool bPostPlace;
    //! bCharge Node
    bool bCharge;


    //! bElevator
    bool bElevator;
    //! bElevatorGetControl
    bool bElevatorGetControl;
    //! bElevatorLeaveControl
    bool bElevatorLeaveControl;
    //! bElevatorOpenDoor
    bool bElevatorOpenDoor;
    //! bElevatorCloseDoor
    bool bElevatorCloseDoor;
    //! iElevatorID
    bool iElevatorID;
    //! iElevatorFloor
    int iElevatorFloor;


    //! bMagneticLoad Node
    bool bMagneticLoad;
    //! bMagneticUnLoad Node
    bool bMagneticUnload;
    //! bFindMagnetic Node
    bool bFindMagnetic;
    //! bLeaveMagnetic Node
    bool bLeaveMagnetic;

    //! DoNothingNode
    bool bDoNothing;

    //! Switch Map
    bool bSwitchMap;

    //! Switch Map Name
    string sSwitchMapName;

    //! Switch Map Point Coordinates
    float fSwitchMapX;
    float fSwitchMapY;
    float fSwitchMapTheta;

    //! Move node
    bool bMove;
    float fMoveGoalX;
    float fMoveGoalY;
    float fMoveGoalTheta;
    float fMoveMaxVelX;
    float fMoveMaxVelY;
    float fMoveMaxVelTheta;

    std::string sLeaveMagneticTurn;
    float fFindMagneticDistance;
    std::string sFindMagneticTurnDirection;

    int iMagneticLoadUnloadLane;
    int iMagneticLoadUnloadCartPosition;
    std::string sMagneticLoadUnloadTurn;
    bool bMagneticLoadUnloadAllowMarkers;
    std::string sMagneticLoadUnloadCartID;

    //! bBlocked
    //bool bBlocked;

    //! iRobot
    int iRobot;

    //! bReserved
    //bool bReserved;

    //! iRobot Reserved
    int iResRobot;

    //! Door

    bool bDoorOpen;
    bool bDoorClose;
    bool bEnterShower;
    bool bLeaveShower;
    int iDoorID;

    std::string sShowerDoor;
    std::string sShowerFrom;

    bool bEnter_Lift;
    bool bLeave_Lift;
    string sLift_Floor;

public:
	//! Constructor
    /*Node(int node){
		iNode = node;
		iDist = INFINITE;
		iParent = NO_PARENT;
		bUsed = false;
		iType = -1;
		iFloor = -1;
        iLaser_Range = 1;
		dX = 0.0;
		dY = 0.0;
		dZ = 0.0;
		bAcoustic=false;
		bCritical=false;
		bFree=false;
		bStop=false;
		bRecalibration=true;

        bLoad=false;
        bUnload=false;
        bCharge=false;
        bElevator=false;
        bBlocked=false;
    }*/
    /*
	//! Constructor
	Node(int node, int type, int floor, double x, double y, double z, char *name){
		iNode = node;
		iDist = INFINITE;
		iParent = NO_PARENT;
		bUsed = false;
		iType = type;
		iFloor = floor;
        iLaser_Range=1;
		dX = x;
		dY = y;
		dZ = z;
		//Cuidado que esto va a petar si metes un cName mas grande que el tamaño asignado strcpy(cName, name);
		bAcoustic=false;
		bCritical=false;
		bFree=false;
		bStop=false;
		bRecalibration=true;
        bLoad=false;
        bUnload=false;
        bCharge=false;
        bElevator=false;
        bBlocked=false;
    }*/
    /*
	//! Constructor
	Node(int node, int type, int floor,int laser, double x, double y, double z, char *name, bool bAcst, bool bCritic, bool bFr, bool bStp, int iPathMod, int zone, bool bRecal){
		iNode = node;
		iDist = INFINITE;
		iParent = NO_PARENT;
		bUsed = false;
		iType = type;
		iFloor = floor;
		iLaser_Range = laser;
        iLaser_Range = 1;
		dX = x;
		dY = y;
		dZ = z;
		//Cuidado que esto va a petar si metes un cName mas grande que el tamaño asignado strcpy(cName, name);
		bAcoustic=bAcst;
		bCritical=bCritic;
		bFree=bFr;
		bStop=bStp;
		iPathMode = iPathMod;
		iZone = zone;
		bRecalibration=bRecal;
        bLoad=false;
        bUnload=false;
        bCharge=false;
        bElevator=false;
        bBlocked=false;
    }*/
    //! Constructor
    Node(int node, int type, int floor,int laser, double x, double y, double z, double dThet, std::string sFram ,
         double xOffs,double yOffs, char *name, bool bAcst, bool bCritic, bool bFr, bool bStp, int iPathMod,
         bool bRecal,bool bChar, bool bL,bool bUL, bool bPrepick,bool bPreplace, bool bPostpick,bool bPostplace,bool bElevat,bool bDOpen,bool bDClose,int iDID,
         bool bEnterLif,bool bLeaveLif,string sLiftFl,bool bEnterShow,bool bLeaveShow,std::string sShSide,std::string sShFrom,

         bool bMagLoad,bool bMagUnload,bool bFindMag,bool bLeaveMag,
         float fFindMagDistance, std::string sFindMagTurnDirection, std::string sLeaveMagTurn,
         int iMLoadUnloadLane,int iMLoadUnloadCartPos,std::string sMLoadUnloadTurn,bool bMLoadUnloadAllowMarkers, std::string sMLoadUnloadCartId){


        bMagneticLoad=bMagLoad;
        bMagneticUnload=bMagUnload;
        bFindMagnetic=bFindMag;
        bLeaveMagnetic=bLeaveMag;

        fFindMagneticDistance=fFindMagDistance;
        sFindMagneticTurnDirection=sFindMagTurnDirection;
        sLeaveMagneticTurn=sLeaveMagTurn;

        iMagneticLoadUnloadLane=iMLoadUnloadLane;
        iMagneticLoadUnloadCartPosition=iMLoadUnloadCartPos;
        sMagneticLoadUnloadTurn=sMLoadUnloadTurn;
        bMagneticLoadUnloadAllowMarkers=bMLoadUnloadAllowMarkers;
        sMagneticLoadUnloadCartID=sMLoadUnloadCartId;



        iNode = node;
        iDist = INFINITE;
        iParent = NO_PARENT;
        bUsed = false;
        iType = type;
        iFloor = floor;
        iLaser_Range = laser;

        //iLaser_Range = 22;
        dX = x;
        dY = y;
        dZ = z;

        dTheta = dThet;

        sFrame_id = sFram;

        dXOffset=xOffs;
        dYOffset=yOffs;


        //strcpy(cName, name); Esto puede tener un overflow tremendo
        strncpy(cName, name, MAX_STRING_LENGTH); // MAX_STRING_LENGTH es el tamaño reservado para cName, asi que se copian como mucho esos caracteres
        cName[MAX_STRING_LENGTH-1] = 0; //y ademas, por si acaso, asignamos NULL al final

        bAcoustic=bAcst;
        bCritical=bCritic;
        bFree=bFr;
        bStop=bStp;
        iPathMode = iPathMod;
        bRecalibration=bRecal;
        bLoad=bL;
        bUnload=bUL;
		bPrePick=bPrepick;
		bPrePlace=bPreplace;
		bPostPick=bPostpick;
		bPostPlace=bPostplace;
        bCharge=bChar;
        bElevator=bElevat;
        //bBlocked=false;
        //bReserved=false;
        iRobot=-1;
        iResRobot=-1;
        bDoorOpen=bDOpen;
        bDoorClose=bDClose;
        iDoorID=iDID;
        bEnterShower=bEnterShow;
        bLeaveShower=bLeaveShow;

        bEnter_Lift=bEnterLif;
        bLeave_Lift=bLeaveLif;
        sLift_Floor=sLiftFl;

        sShowerFrom=sShFrom;
        sShowerDoor=sShSide;
    }
	//! Destructor
	~Node(){
		//delete log;
		//std::cout << "Node::~Node:" << std::endl;
	}
	//! Sets the node's coordinates
    void SetPosition(double x, double y, double z, double theta, std::string sFram){
		dX = x;
		dY = y;
		dZ = z;
        dTheta = theta;
        sFrame_id=sFram;
	}
	//! Sets the node's floor
	void SetFloor(int floor){
		iFloor = floor;
	}
	//! Sets the nodes's type
	void SetType(int type){
		iType = type;
	}
	//! Sets the node's name
	void SetName(char *name){
        //strcpy(cName, name); Esto puede tener un overflow tremendo
        strncpy(cName, name, MAX_STRING_LENGTH); // MAX_STRING_LENGTH es el tamaño reservado para cName, asi que se copian como mucho esos caracteres
        cName[MAX_STRING_LENGTH-1] = 0; //y ademas, por si acaso, asignamos NULL al final
	}
	//! Adds new node adjacent with default weight and a list of magnets in the way
	//!	\returns 0 if OK
	int AddNodeAdjacent(int node_id, double speed, std::vector<int> magnetsOnArc, std::vector<Magnet*> magnetsPointerOnArc){
		int size = vAdjacent.size();

		if(size > 0){
			for(int i = 0; i< size; i++){	//Comprobamos que no esté repetido
				if(vAdjacent[i].iNextNode == node_id){	//Si está repetido no lo insertamos

                    //std::cout << "Node::AddNodeAdjacent: Error: node " << node_id << " already adjacent" << std::endl;
					return -1;
				}
			}
		}
		//
		// Añadimos el arco
		Arc new_arc(node_id, speed);
		new_arc.AddMagnets(magnetsOnArc, magnetsPointerOnArc);
		vAdjacent.push_back(new_arc);
		return 0;
	}
	//! Adds new node adjacent with selected weight and a list of magnets in the way
	//!	\returns 0 if OK
	int AddNodeAdjacent(int node_id, double speed, int weight, std::vector<int> magnetsOnArc, std::vector<Magnet*> magnetsPointerOnArc){
		int size = vAdjacent.size();

		if(size > 0){
			for(int i = 0; i< size; i++){	//Comprobamos que no esté repetido
				if( vAdjacent[i].iNextNode == node_id ){	//Si está repetido no lo insertamos
                    //std::cout << "Node::AddNodeAdjacent: Error: node " << node_id << " already adjacent" << std::endl;

					return -1;
				}
			}
		}
		Arc new_arc(node_id, abs(weight), speed); //Le pasamos el peso en valor absoluto, para evitarnos comprobaciones
		new_arc.AddMagnets(magnetsOnArc, magnetsPointerOnArc);
		vAdjacent.push_back(new_arc);
		return 0;
	}
	//! Resets values for new routes
	void Reset(){
		iDist = INFINITE;
		iParent = NO_PARENT;
		bUsed = false;
	}
	//! Format the node as initial
	void SetInitial(){
		iDist = 0;
		iParent = NO_PARENT;
		bUsed = false;
	}
	//! Sets the parent's node in the route
	void SetParent(int parent){
		iParent = parent;
	}
	//! Deletes selected adjacent node
	//!	\returns 0 if OK
	int DeleteAdjacent(int node_id){
		int size = vAdjacent.size();
		if(size > 0){
			for(int i = 0; i < size; i++){
				if( vAdjacent[i].iNextNode == node_id ){	//Encontrado
					vAdjacent.erase(vAdjacent.begin() + i);
					return 0;
				}
			}
		}

		//std::cout << "Node::AddNodeAdjacent: Error: node " << node_id << " not adjacent" << std::endl;
		return -1;
	}
	//! Deletes all adjacent nodes
	int DeleteAdjacent(){
		vAdjacent.clear();

		return 0;
	}
	//! Prints current edges
	void PrintArcs(){
		int size = vAdjacent.size();

		if(size > 0){
			std::cout << "\t Node::PrintArcs: " << size << " arcs" << std::endl;
			for(int i = 0; i < size; i++){
				std::cout << "\t\tArc " << i << ": Weight = " << vAdjacent[i].iWeight << ", Point to " << vAdjacent[i].iNextNode << std::endl;
				std::cout << "\t\tMagnets: " << std::endl << "\t\t\t";
				for(int j = 0; j < (int)vAdjacent[i].vpMagnets.size(); j++){
					std::cout << vAdjacent[i].vpMagnets[j]->iNumber << " ";
				}
				std::cout << std::endl;
			}
		}else
			std::cout << "\tNode::PrintArcs: No arcs.." << std::endl;

	}


	//! returns if the node has been used in the algorithm
	bool IsUsed(){
		return bUsed;
	}
	//! Sets the node as used
	void Used(){
		bUsed = true;
	}
	//! Returns the current distance stored in the node
	int Distance(){
		return iDist;
	}
	//! Sets the value of the distance
	void SetDistance(int distance){
		iDist = distance;
	}
	//! returns the value of the id
	int GetId(){
		return iNode;
	}
	//! returns the value of the parent
	int GetParent(){
		return iParent;
	}
	//! returns the type of the node
	int GetType(){
		return iType;
	}
	//! returns the floor where the node is located
	int GetFloor(){
		return iFloor;
	}
	//! returns the position of the node
	int GetPosition(double *x, double *y, double *z){
		*x = dX;
		*y = dY;
		*z = dZ;

		return 0;
	}
	//! returns the name of the node
	char *GetName(){
		return cName;
	}
	//! Gets the vector of magnets until the node adjacent
	//!	\returns 0 if OK
	int GetArcToAdjacent(int node_adjacent, std::vector<Magnet> *magnets, double *speed){
		int size = vAdjacent.size();
		if(size > 0){
			//
			// Buscamos el nodo adyacente y extraemos la lista de imanes
			for(int i = 0; i < size; i ++){
				if(vAdjacent[i].iNextNode == node_adjacent){	//nodo encontrado
					//magnets = &vAdjacent[i].vpMagnets;			//Devolvemos la lista de imanes
					*speed = vAdjacent[i].dSpeed;
					for(int j = 0; j < (int)vAdjacent[i].vpMagnets.size(); j++){
						magnets->push_back(*vAdjacent[i].vpMagnets[j]);
					}
					//std::cout << "Node::GetMagnetsToAdjacent: Added " << magnets->size() << " magnets from " << iNode << " to " << node_adjacent << std::endl;
					return 0;
				}
			}
		}

		// No existe el arco a ese nodo
		return -1;
	}

    //! Adds a new Zone
    //! return -1 if the node already exists
    //! return 0 if OK
    int AddZoneToNode(int iZone){
        for(int i= 0; i < (int)viZones.size(); i++){
            if(viZones[i] == iZone)
                return -1;
        }
        viZones.push_back(iZone);
        return 0;
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
class Dijkstra{

	//! Class Route, contiene (puede llegar a contener, puesto que se calcula conforme va haciendo falta) la ruta óptima
	//!	entre todos los nodos
	class Route {
		//! Class NodesRoute
		//! Clase que contiene la lista de nodos de una ruta calculada
		class NodesRoute{
			public:
				//! Array con los nodos de esa ruta
				std::vector <int> vListNodes;
				//! Vector con los atributos detallados de cada nodo de la ruta
				std::vector <Node> vListPointerNode;
			public:
				//! Public constructor
				NodesRoute(){ }
				//! Public destructor
				~NodesRoute(){
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
			Route(int num_nodes){

				nodes = num_nodes;
			//	std::cout << "Route::Route: nodes = " << nodes << std::endl;

                iRoute = new NodesRoute *[nodes];
				for(int i = 0; i < nodes; i++){
                    iRoute[i] = new NodesRoute [nodes];
				}

			}
			//! destructor
			~Route(){

				for(int i= 0; i < nodes; i++)
					delete [] iRoute[i];

				delete [] iRoute;
				//std::cout << "Route::~Route:" << std::endl;
			}
			//! Añade un nodo en la ruta para llegar al nodo objetivo "to_node"
			//! La ruta se añade de manera inversa, por lo que insertamos elementos siempre al principio del vector
			int AddNode(int from_node, int to_node, int node){
				if( (from_node < 0)  || (from_node > nodes-1) || (to_node < 0) || (to_node > nodes-1) ){
					return -1;
				}
				std::vector<int>::iterator it;

				it = iRoute[from_node][to_node].vListNodes.begin();
				it = iRoute[from_node][to_node].vListNodes.insert ( it , node );
				//std::cout << "Route::AddNode: Added from " << from_node << " to " << to_node << ": " << node << std::endl;
				return 0;
			}
			//! Añade un nodo en la ruta para llegar al nodo objetivo "to_node"
			//! La ruta se añade de manera inversa, por lo que insertamos elementos siempre al principio del vector
			int AddNode(int from_node, int to_node, Node node){
				if( (from_node < 0)  || (from_node > nodes-1) || (to_node < 0) || (to_node > nodes-1) ){
					return -1;
				}
				std::vector<Node>::iterator it;

				it = iRoute[from_node][to_node].vListPointerNode.begin();
				it = iRoute[from_node][to_node].vListPointerNode.insert ( it , node );
				//std::cout << "Route::AddNode: Added from " << from_node << " to " << to_node << ": " << node << std::endl;
				return 0;
			}
			//! Devuelve la ruta deseada
			std::vector<int> GetRoute(int from_node, int to_node){
				if( (from_node < 0) || (from_node > nodes-1) || (to_node < 0) || (to_node > nodes-1) ){
					std::vector <int> aux;
					return aux;
				}

				return iRoute[from_node][to_node].vListNodes;
			}
			std::vector<Node> GetDetailRoute(int from_node, int to_node){
				if( (from_node < 0) || (from_node > nodes-1) || (to_node < 0) || (to_node > nodes-1) ){
					std::vector <Node> aux;
					return aux;
				}
                //cout << "GetDetailedRoute. Size = " << iRoute[from_node][to_node].vListPointerNode.size() << endl;
				return iRoute[from_node][to_node].vListPointerNode;
			}

			//! Consulta si existe la ruta
			bool ExistRoute(int from_node, int to_node){
				if( (from_node < 0) || (from_node > nodes-1) || (to_node < 0) || (to_node > nodes-1) ){
					return false;
				}
				if(iRoute[from_node][to_node].vListNodes.size() > 0)
					return true;
				else
					return false;
			}
			//! Imprime la ruta
			void Print(int from_node, int to_node){
				if(ExistRoute(from_node, to_node)){
					int size = iRoute[from_node][to_node].vListNodes.size();
					int i = 0;
					std::cout << "Route::Print: Route from " << from_node << " to " << to_node << ": " << size << " nodes" << std::endl << "\t";
					for(i = 0; i < size-1; i++){
						std::cout << iRoute[from_node][to_node].vListNodes[i] << " -> ";
					}
					std::cout << iRoute[from_node][to_node].vListNodes[i] << std::endl;
				}else
					std::cout << "Route::Print: This route doesn't exist" << std::endl;
			}

			//! Devuelve el número de rutas
			int GetNumOfRoutes(){
				return nodes;
			}

			//! Deletes the selected route
			int DeleteRoute(int from_node, int to_node){
				if( (from_node < 0) || (from_node > nodes - 1) || (to_node < 0) || (to_node > nodes - 1) ){
					//std::cout << "Route::DeleteRoute: This route doesn't exist: " << from_node << " -> " << to_node << std::endl;
					return -1;
				}
				iRoute[from_node][to_node].vListNodes.clear();	//Limpiamos la ruta
				iRoute[from_node][to_node].vListPointerNode.clear();	//Limpiamos la ruta
				return 0;

			}

			//! Deletes all the routes
			void Reset(){
				for(int i = 0; i < nodes; i++){
					for(int j = 0; j < nodes; j++){
						DeleteRoute(i,j);
					}
				}
			}

	};

    //! Class Zone
    class Zone{
        public:
        //! ID Zone
        int iIDZone;
        //! Max Robots in Zone
        int iMaxRobots;
        //! Zone Type
        //! =0 o 1 Zona Exclusion Normal
        //! =2 No Maniobra, Movimientos con origen y destino dentro de la misma zona generaran una ruta con un paso
        //!     intermedio por el nodo destino zona.
        //! =3 ambos casos
        bool bMan;
        //! Node Dest Zone cuando de devuelva un carro a la zona primero se envia aqui, tambien sera el nodo intermedio
        //! para los movimientos con origen y destino dentro de la misma zona de no maniobra
        int iNodeDest;

        //! complementary Zone
        //!
        int iComplementary;
        //! Vector Lista Nodos que contiene la zona
        std::vector<Node *> vpNodes;

        Zone(int iZ){
            iIDZone=iZ;
            iMaxRobots=1;
            bMan=false;
            iNodeDest=-1;
        }

        Zone(int iZ,int iMaxR){
            iIDZone=iZ;
            iMaxRobots=iMaxR;
            bMan=false;
            iNodeDest=-1;
        }

        Zone(int iZ,int iMaxR,bool bMan,int iNodeD,int iComp){
            iIDZone=iZ;
            iMaxRobots=iMaxR;
            bMan=false;
            iNodeDest=iNodeD;
            iComplementary=iComp;
        }

        ~Zone(){
        }

        //! Adds a new Zone
        //! return -1 if the node already exists
        //! return 0 if OK
        int AddNodeToZone(Node *pNode,bool bMan){
            for(int i=0; i<(int)vpNodes.size(); i++){
                if(vpNodes[i]->iNode == pNode->iNode)
                    return -1;
            }
            vpNodes.push_back(pNode);
            pNode->AddZoneToNode(this->iIDZone);
            return 0;
        }


    };



	//! Class PointerNode se utiliza para poder meter las referencias de los nodos en una cola con prioridad
	class PointerNode {
	public:
		Node *node;
	public:
		PointerNode(Node *new_node){
			node = new_node;
		}
		PointerNode(){
			node = NULL;
		}
	};

	//! Clase utilizada para hacer la comparación entre nodos, cada vez que insertamos uno en la cola de prioridad
	class NodeComparison {

	public:
		NodeComparison() {}
		bool operator() (const PointerNode &lhs, const PointerNode &rhs) const	//Utiliza punteros a nodo, puesto que la distancia del nodo puede ser modificada una vez introducida en la cola
		{
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
		std::priority_queue<PointerNode , std::deque<PointerNode>, NodeComparison> pqQueue;
		//! Array de punteros a nodo, para acceder a los nodos directamente. Referenciados por su ID
		Node **pNodes;
		//! max value of a node id
		int iMaxNodeId;
		//! max value of a magnet id
		int iMaxMagnetId;
		//! Object for save and print DeviceLog messages
	public:
        //! Vector de imanes
		std::vector<Magnet> vMagnets;
		//! Vector con los nodos del grafo
		std::vector<Node> vNodes;
        //! Zones Vector
        std::vector<Zone> vZones;

	public:
		//! Public constructor
		Dijkstra();
		//! Public Destructor
		~Dijkstra();
		//! Disables the edition of nodes, edges. Necesario para poder calcular rutas sin incoherencias
		//! Return -1 if exist errors (Nodos con arcos a nodos que no existen)
        int FinalizeEdition(string *msg);
		//! Enable the edition of nodes, edges. Necesario para poder añadir nodos y aristas. Todos los cálculos realizados se perderán
		void EnableEdition();
		//! Reset the calculated route
		int ResetRoutes();
		//! Deletes All the nodes
		int DeleteNodes();
		//! Deletes selected edge
		int DeleteArc(int from_node, int to_node);
		//! Deletes all the edge from the node
		int DeleteArcs(int from_node);
		//! Gets the optimum calculated route between selected nodes
		int GetRoute(int inital_node, int end_node, std::vector<int> *route);
		int GetRoute(int inital_node, int end_node, std::vector<Node> *route);

        //! Get list of nodes used or blocked
        bool GetNodesUsed(std::vector<Node *> *route);

        //! reserve Node
        bool ReserveNode(int iRobot,int iIDNode);

        //! GetNearestNode
        int  GetNearestNodeID(double x, double y, string frame);

        //! bool UnBlockAll(int iRobot)
        bool UnBlockAll(int iRobot);

		//! Adds a new node
        //int AddNode(int node, int type, int floor, int laser, double x, double y, double z, char *name , bool bAcoustic, bool bCritical, bool bFree, bool bStop, int iPathMode, int zone, bool bRecal);

        //! Adds a new node
        int AddNode(int node, int type, int floor, int laser, double x, double y, double z, double theta, std::string frame,
                    double xOffset, double yOffset, char *name , bool bAcoustic, bool bCritical, bool bFree, bool bStop, int iPathMode,
                    bool bRecal, bool bCharge, bool bLoad, bool bUnload, bool bPrePick, bool bPrePlace, bool bPostPick, bool bPostPlace,
                    bool bElevator, bool bDoorOpen, bool bDoorClose, int iDoorID, bool bEnterLift, bool bLeaveLift, string sLiftFloor,
                    bool bEnterShower,
                    bool bLeaveShower, std::string sShDoor, std::string sShFrom, bool bMagPick, bool bMagPlace, bool bFindMag, bool bLeaveMag,
                    float fFindMagDistance, string sFindMagTurnDirection, string sLeaveMagTurn, int iMPickPlaceLane, int iMPickPlaceCartPos,
                    string sMPickPlaceTurn, bool bMPickPlaceAllowMarkers, string sMPickPlaceCartId);

        //! Add ElevatorParamstoNode
        int AddElevatorParams(int iNode, bool bElev, int iID, bool bGetControl, bool bLeaveControl, bool bOpenDoor, bool bCloseDoor, int iFloor);

        //! Add DoNothing
        int AddDoNothing(int iNode, bool bDoNot);

        //! Add Switch Map
        int AddSwitchMap(int iNode, bool bSwitch, string sMap, float x, float y, float theta);

        //! Add Move Params
        int AddMoveParams(int iNode, bool bMove,float fXMoveGoal,float fYMoveGoal,float fThetaMoveGoal,float fXMaxVel,float fYMaxVel,float fThetaMaxVel);


		//! Adds edge from a node to another with constant weight
		int AddArc(int from_node, int to_node, double speed, std::vector<int> magnetsOnArc);
		//! Adds edge from a node to another with weight
		int AddArc(int from_node, int to_node, double speed, int weight, std::vector<int> magnetsOnArc);
		//! Get the node with this id
		int GetNodePosition(int node_id, double *x, double *y, double *z);
		//! Gets the list of Magnets on the arc between both nodes
		int GetArcBetweenNodes(int from_node, int to_node, std::vector<Magnet> *magnets_on_arc, double *speed);
		//! Prints current nodes
		void PrintNodes();
		//! Adds a new magnet
		int AddMagnet(int id, double x, double y, double z);
		//! Prints all the magnets
		void PrintMagnets();
		//! Deletes all the nodes
		int DeleteMagnets();
		//! Deletes all
		int DeleteAll();
		//! Gets the index of the node using his ID
		int GetNodeIndex(int nodeID);
		//! Gets Node by nodeID
		Node* GetNode(unsigned int node_id);
        //! Add Node to Zone
        int AddNodeToZone(int iIDNode,int iIDZone,bool bMan);
        //! Add Zone to Graph
        int AddZone(int iIDZone, int iMaxRobots, bool bMan, int iNodeDest, int iComp);

        bool CheckNodeFree(int iIDNode,int iIDRobot);
        bool CheckZoneFree(int iIDZone,int iIDRobot);
        bool CheckCompZoneFree(int iIDZone,int iIDRobot);

        //! Get Node From ID
        Node *GetNodeFromID(int iIDNode);
	private:
		//! Set the initial node
		int SetInitialNode(int node);
		//! Reset the value of the nodes
		int ResetNodes();
		//! Calculates the optimum route
		int CalculateRoute(int initial_node, int end_node);
		//! Resets the priority queue
		int ResetQueue();
		//! Push a node into the queue
		int PushIntoQueue(int node);
		//! Pops the node with more priority
		int PopFromQueue(PointerNode *pn);
		//! Queues all the nodes
		int QueueNodes();
		//! Updates the queue after modifiying values of the nodes.
		void UpdateQueue();

};


#endif
