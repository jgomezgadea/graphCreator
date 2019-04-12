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

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>
#include <xercesc/parsers/AbstractDOMParser.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMImplementationRegistry.hpp>
#include <xercesc/dom/DOMLSParser.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMError.hpp>
#include <xercesc/dom/DOMLocator.hpp>
#include <xercesc/dom/DOMNamedNodeMap.hpp>
#include <xercesc/dom/DOMAttr.hpp>

#include <robotnik_fms_routes/Dijkstra.h>
#include <robotnik_fms_routes/DOMPrintFilter.hpp>
#include <robotnik_fms_routes/DOMPrintErrorHandler.hpp>
#include <robotnik_fms_routes/DOMTreeErrorReporter.hpp>
#include <robotnik_fms_routes/DOMCount.hpp>
#include <geometry_msgs/Pose2D.h>

//XERCES_CPP_NAMESPACE_USE
using namespace std;

//! Defines return values for methods and functions
enum ReturnValue{
	OK = 0,
	INITIALIZED,
	THREAD_RUNNING,
	ERROR = -1,
	NOT_INITIALIZED = -2,
	THREAD_NOT_RUNNING = -3,
	COM_ERROR = -4,
	NOT_ERROR = -5
};

//!  This is a simple class that lets us do easy (though not terribly efficient)
//!  trancoding of char* data to XMLCh data.
class XStr {

public :

    //! Constructor
    XStr(const char* const toTranscode)
    {
        // Call the private transcoding method
        fUnicodeForm = XMLString::transcode(toTranscode);
    }
	//! Destructor
    ~XStr()
    {
        XMLString::release(&fUnicodeForm);
    }


    // -----------------------------------------------------------------------
    //  Getter methods
    // -----------------------------------------------------------------------
    const XMLCh* unicodeForm() const
    {
        return fUnicodeForm;
    }



private :
    //! This is the Unicode XMLCh format of the string.
    XMLCh*   fUnicodeForm;
};

#define X(str) XStr(str).unicodeForm()

class Graph{

private:
	//! Nombre del fichero xml
	char cXMLFile[128];
	//! Controls if has been initialized succesfully
	bool bInitialized;
	int magnets;
	int nodes;
    int zones;
public:
	//! Grafo con los nodos y arcos del sistema
	Dijkstra *dijkstraGraph;

public:
	//! Constructor
	Graph(const char *xmlFile);
	//! Destructor
	~Graph();
	//!
    int Setup(string *msg);
	//!
	int ShutDown();
	//! Prints the list of nodes and magnets read from xml
	void Print();
    //! Obtiene las coordenadas de los nodos y de los imanes ordenadas para esa trayectoria, además de las velocidades entre dichos nodos
	int GetRoute(int from, int to, vector<geometry_msgs::Pose2D> *nodes, vector<geometry_msgs::Pose2D> *magnets, vector<double> *speed_between_nodes);
	//! Misma función salvo que obtiene los nodos de la ruta en detalle, no solamente su posición
	int GetRoute(int from, int to, vector<Node> *detailed_nodes, vector<geometry_msgs::Pose2D> *nodes, vector<geometry_msgs::Pose2D> *magnets, vector<double> *speed_between_nodes);
	//! Obtiene los nodos por los que pasa la ruta que va desde el nodo "from" al nodo "to"
	int GetRoute(int from, int to, vector<int> *route);
	//! Obtiene los nodos e imanes de la ruta de forma detallada
	int GetRoute(int from, int to, vector<Node> *detailed_nodes, vector<Magnet> *detailed_magnets, vector<double> *speed_between_nodes);
	//! Obtiene la posición del nodo indicado
	int GetNodePosition(int num_node, geometry_msgs::Pose2D  *pos);

    //! Get List of Used Nodes
    bool GetNodesUsed(std::vector<Node *> *route);

    bool ReserveNode(int iRobot, int iIDNode);

    bool UnBlockAll(int iRobot);
	//! 
	int GetMagnets();
	int GetNodes();
	//! Gets Node by nodeID
	Node* GetNode(unsigned int node_id);

    bool CheckNodeFree(int idNode,int idRobot);
    bool CheckNodesFree(std::vector<int> vNodesId,int idRobot);
    bool CheckZoneFree(int idZone,int idRobot);
private:
	//! Deserializes a xml file, extracting the nodes and magnets
    int Deserialize(string *msg);
	//! Process nodes from xml document
	int ProcessNodesFromXML(DOMDocument *doc);
	//! Process magnets from xml document
	int ProcessMagnetsFromXML(DOMDocument *doc);
    //! Process Zones From XML
    int ProcessZonesFromXML(DOMDocument *doc);

};


#endif