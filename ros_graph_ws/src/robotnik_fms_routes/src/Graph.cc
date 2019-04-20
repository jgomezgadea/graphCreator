/** \file Graph.cc
 * \author Robotnik Automation s.L.L.
 * \version 1.1
 * \date    2011
 * \brief Class for managing the nodes, magnets and routes of the system
 * (C) 2011 Robotnik Automation, SLL. All rights reserved
*/

#include <robotnik_fms_routes/Graph.h>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>

DOMCountErrorHandler::DOMCountErrorHandler() :

                                               fSawErrors(false)
{
}

DOMCountErrorHandler::~DOMCountErrorHandler()
{
}

// ---------------------------------------------------------------------------
//  DOMCountHandlers: Overrides of the DOM ErrorHandler interface
// ---------------------------------------------------------------------------
bool DOMCountErrorHandler::handleError(const DOMError &domError)
{
    fSawErrors = true;
    if (domError.getSeverity() == DOMError::DOM_SEVERITY_WARNING)
        // XERCES_STD_QUALIFIER cerr << "\nWarning at file ";
        cout << "Warning at file " << endl;
    else if (domError.getSeverity() == DOMError::DOM_SEVERITY_ERROR)
        //XERCES_STD_QUALIFIER cerr << "\nError at file ";
        cout << "Error at file " << endl;
    else
        XERCES_STD_QUALIFIER cerr << "\nFatal Error at file ";

    cout << StrX(domError.getLocation()->getURI())
         << ", line " << domError.getLocation()->getLineNumber()
         << ", char " << domError.getLocation()->getColumnNumber()
         << "\n  Message: " << StrX(domError.getMessage()) << endl;

    return true;
}

void DOMCountErrorHandler::resetErrors()
{
    fSawErrors = false;
}

/*! \fn Graph::Graph(char *cXMLFile)
 * 	\brief Constructor
*/
Graph::Graph(const char *xmlFile)
{
    strcpy(cXMLFile, xmlFile);
    dijkstraGraph = new Dijkstra();
    bInitialized = false;
    nodes = zones = 0;
}

/*! \fn Graph::~Graph()
 * 	\brief Destructor
*/
Graph::~Graph()
{
    delete dijkstraGraph;
}

/*! \fn int Graph::setup()
 * 	\brief Configures and initializes the graph
 *  @return OK
 *  @return ERROR
 *  @return INITIALIZED
*/
int Graph::setup(string *msg)
{
    ROS_INFO("Graph::start Setup");

    if (bInitialized)
    {
        ROS_ERROR("Graph::setup: Already initialized");
        *msg = "Graph::setup: Already initialized";
        return INITIALIZED;
    }
    std::string desmsg;
    if (deserialize(&desmsg) == ERROR)
    {
        ROS_ERROR("Graph::setup: Error deserializing the graph");
        *msg = "Graph::setup: Error deserializing the graph:" + desmsg;
        return ERROR;
    }
    desmsg = "";
    if (dijkstraGraph->finalizeEdition(&desmsg) == -1)
    {
        ROS_ERROR("Graph::setup: Error graph");
        *msg = "Graph::setup: Error graph:" + desmsg;
        return ERROR;
    }

    ROS_INFO("Graph::end Setup OK");
    bInitialized = true;
    return OK;
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
 * 	\brief Process the data reading previouly from a xml document
*/
[[deprecated]] int Graph::deserialize(std::string *msg) {
    // Initialize the XML4C2 system.
    try
    {
        XMLPlatformUtils::Initialize();
    }
    catch (const XMLException &toCatch)
    {
        char *pMsg = XMLString::transcode(toCatch.getMessage());
        ROS_ERROR("Graph::deserialize: Error during Xerces-c Initialization: %s", pMsg);
        *msg = "Graph::deserialize: Error during Xerces-c Initialization:" + std::string(pMsg);
        XMLString::release(&pMsg);
        return ERROR;
    }

    AbstractDOMParser::ValSchemes valScheme = AbstractDOMParser::Val_Auto;
    bool doNamespaces = false;
    bool doSchema = false;
    bool schemaFullChecking = false;
    //	bool doList = false;
    bool errorOccurred = false;
    //	bool recognizeNEL = false;
    //  bool printOutEncounteredEles = true;
    char localeStr[64];
    memset(localeStr, 0, sizeof localeStr);
    //
    // Instantiate the DOM parser
    static const XMLCh gLS[] = {chLatin_L, chLatin_S, chNull};
    DOMImplementation *impl = DOMImplementationRegistry::getDOMImplementation(gLS);
    DOMLSParser *parser = ((DOMImplementationLS *)impl)->createLSParser(DOMImplementationLS::MODE_SYNCHRONOUS, 0);
    DOMConfiguration *config = parser->getDomConfig();

    config->setParameter(XMLUni::fgDOMNamespaces, doNamespaces);
    config->setParameter(XMLUni::fgXercesSchema, doSchema);
    config->setParameter(XMLUni::fgXercesSchemaFullChecking, schemaFullChecking);

    if (valScheme == AbstractDOMParser::Val_Auto)
    {
        config->setParameter(XMLUni::fgDOMValidateIfSchema, true);
    }
    else if (valScheme == AbstractDOMParser::Val_Never)
    {
        config->setParameter(XMLUni::fgDOMValidate, false);
    }
    else if (valScheme == AbstractDOMParser::Val_Always)
    {
        config->setParameter(XMLUni::fgDOMValidate, true);
    }

    // enable datatype normalization - default is off
    config->setParameter(XMLUni::fgDOMDatatypeNormalization, true);

    // And create our error handler and install it
    DOMCountErrorHandler errorHandler;
    config->setParameter(XMLUni::fgDOMErrorHandler, &errorHandler);

    //
    //  Get the starting time and kick off the parse of the indicated
    //  file. Catch any exceptions that might propogate out of it.
    unsigned long duration;

    ROS_INFO("Graph::deserialize File: %s", cXMLFile);
    //reset error count first
    errorHandler.resetErrors();

    XERCES_CPP_NAMESPACE_QUALIFIER DOMDocument *doc = 0;

    try
    {
        // reset document pool
        parser->resetDocumentPool();

        const unsigned long startMillis = XMLPlatformUtils::getCurrentMillis();
        doc = parser->parseURI(cXMLFile);
        const unsigned long endMillis = XMLPlatformUtils::getCurrentMillis();
        duration = endMillis - startMillis;
    }
    catch (const XMLException &toCatch)
    {
        ROS_ERROR("Graph::deserialize: Error during parsing %s file", cXMLFile); //, StrX(toCatch.getMessage()));

        // XERCES_STD_QUALIFIER cerr << "\nError during parsing: '" << cXMLFile << "'\n"
        //      << "Exception message is:  \n"
        //      << StrX(toCatch.getMessage()) << "\n" << XERCES_STD_QUALIFIER endl;

        *msg = "Graph::deserialize: Error during parsing " + std::string(cXMLFile) + " file";
        errorOccurred = true;
    }
    catch (const DOMException &toCatch)
    {
        const unsigned int maxChars = 2047;
        XMLCh errText[maxChars + 1];
        ROS_ERROR("Graph::deserialize: DOM Error during parsing %s file. DOMException code is: %d", cXMLFile, toCatch.code);
        *msg = "Graph::deserialize: DOM Error during parsing " + std::string(cXMLFile) + " file. DOMException code is:" + std::to_string(toCatch.code);

        if (DOMImplementation::loadDOMExceptionMsg(toCatch.code, errText, maxChars))
        {
            //XERCES_STD_QUALIFIER cerr << "Message is: " << StrX(errText) << XERCES_STD_QUALIFIER endl;
            ROS_ERROR("Graph::deserialize: Error");
        }
        errorOccurred = true;
    }
    catch (...)
    {
        ROS_ERROR("Graph::deserialize: Unexpected exception during parsing %s file", cXMLFile);
        *msg = "Graph::deserialize: Unexpected exception during parsing " + std::string(cXMLFile) + " file";
        errorOccurred = true;
    }

    //
    // Extract the DOM tree, get the list of all the elements and report the
    // length as the count of elements.
    if (errorHandler.getSawErrors())
    {
        // XERCES_STD_QUALIFIER cout << "\nErrors occurred, no output available\n" << XERCES_STD_QUALIFIER endl;
        ROS_ERROR("Graph::deserialize:Errors occurred, no output available");
        *msg = "Graph::deserialize:Errors occurred, no output available";
        errorOccurred = true;
    }
    else
    {
        if (doc)
        {
            //
            // Importante procesar antes magnets que nodos, puesto que los arcos de los nodos, referencian magnets

            nodes = processNodesFromXML(doc);
            zones = processZonesFromXML(doc);
            ROS_INFO("Graph::Deserialize: Processing XML Found %d Nodes, %d Zones", nodes, zones);
            *msg = "Graph::Deserialize: Processing XML Found " + std::to_string(nodes) + " Nodes," + std::to_string(zones) + " Zones";
        }
    }

    //
    //  Delete the parser itself.  Must be done prior to calling Terminate, below.
    parser->release();

    XMLPlatformUtils::Terminate();

    if (errorOccurred)
        return ERROR;
    else
        return OK;
}

/*! \fn int Graph::processZonesFromXML(DOMDocument *doc)
 * 	\brief
*/
int Graph::processZonesFromXML(DOMDocument *doc)
{
    DOMNode *zone, *zoneElement;
    XMLCh *tmpstr;
    int ilen = 0, ilenZoneElements = 0;
    int iZoneNumber = 0;
    int iZoneMaxRobots = 0;
    int iComplementary = -1;

    int Nodes[200];

    DOMNodeList *listZoneElements, *list;

    tmpstr = XMLString::transcode("CZone");
    list = doc->getElementsByTagName(tmpstr);
    ilen = list->getLength();

    //CMAGNETS
    for (int i = 0; i < ilen; i++)
    {
        int iNumNodes = 0;
        bool bMan = false;
        int iNodeDest = -1;
        iComplementary = -1;

        zone = list->item(i);
        listZoneElements = zone->getChildNodes();
        ilenZoneElements = listZoneElements->getLength();
        //Elements of CMagnet
        for (int j = 0; j < ilenZoneElements; j++)
        {
            zoneElement = listZoneElements->item(j);
            //
            // NUMBER
            if (XMLString::equals(XMLString::transcode(zoneElement->getNodeName()), "m_iIDZone"))
            {
                iZoneNumber = XMLString::parseInt(zoneElement->getTextContent());
            }
            else if (XMLString::equals(XMLString::transcode(zoneElement->getNodeName()), "m_iMaxRobots"))
            {

                iZoneMaxRobots = atof(XMLString::transcode(zoneElement->getTextContent()));
            }
            else if (XMLString::equals(XMLString::transcode(zoneElement->getNodeName()), "m_iCompZone"))
            {
                iComplementary = atoi(XMLString::transcode(zoneElement->getTextContent()));
            }
            else if (XMLString::equals(XMLString::transcode(zoneElement->getNodeName()), "m_bManeuver"))
            {
                bMan = atof(XMLString::transcode(zoneElement->getTextContent()));
            }
            else if (XMLString::equals(XMLString::transcode(zoneElement->getNodeName()), "m_iNodeDest"))
            {

                iNodeDest = atof(XMLString::transcode(zoneElement->getTextContent()));
            }
            else if (XMLString::equals(XMLString::transcode(zoneElement->getNodeName()), "m_iNode"))
            {

                Nodes[iNumNodes] = atof(XMLString::transcode(zoneElement->getTextContent()));

                iNumNodes++;
            }
        }

        dijkstraGraph->addZone(iZoneNumber, iZoneMaxRobots, bMan, iNodeDest, iComplementary);
        for (int i = 0; i < iNumNodes; i++)
        {
            ROS_INFO(" Adding Node:%d to Zone:%d,Compl:%d", Nodes[i], iZoneNumber, iComplementary);
            dijkstraGraph->addNodeToZone(Nodes[i], iZoneNumber, bMan);
            ROS_INFO(" Added Node:%d to Zone:%d,Compl:%d", Nodes[i], iZoneNumber, iComplementary);
        }
    }

    return ilen;
}

/*! \fn int Graph::processNodesFromXML(DOMDocument *doc)
 * 	\brief
*/
int Graph::processNodesFromXML(DOMDocument *doc)
{
    DOMNode *node, *nodeElement, *nodePosition, *arc, *arcElement;
    XMLCh *tmpstr;
    int len = 0, lenNodeElements = 0, lenArcs = 0, lenArcElements = 0;
    int i, j, k, l, m;
    double x = 0.0, y = 0.0, z = 0.0, speed = 0.0;
    int nodeNumber = 0, nodeDst = 0, nodeZone = 0;
    char *nodeName; //[50]= "\0";
    DOMNodeList *listNodeElements, *list, *listArcs, *listArcElements;

    float fTheta = 0.0;
    std::string sFrame = "map";

    tmpstr = XMLString::transcode("CNode");
    list = doc->getElementsByTagName(tmpstr);
    len = list->getLength();

    ROS_INFO("Graph::proccesNodesFromXML: Processing %d Nodes", len);

    for (i = 0; i < len; i++)
    {
        //Returns DOMnode object
        node = list->item(i);
        listNodeElements = node->getChildNodes();
        lenNodeElements = listNodeElements->getLength();

        //Elements of the node
        for (j = 0; j < lenNodeElements; j++)
        {
            nodeElement = listNodeElements->item(j);

            if (XMLString::equals(XMLString::transcode(nodeElement->getNodeName()), "m_iNumber"))
            { // NUMBER
                nodeNumber = XMLString::parseInt(nodeElement->getTextContent());
            }
            else if (XMLString::equals(XMLString::transcode(nodeElement->getNodeName()), "m_Position"))
            { // POSITION
                for (nodePosition = nodeElement->getFirstChild(); nodePosition != 0; nodePosition = nodePosition->getNextSibling())
                {
                    if (XMLString::equals(XMLString::transcode(nodePosition->getNodeName()), "X"))
                    {
                        x = atof(XMLString::transcode(nodePosition->getTextContent()));
                    }
                    else if (XMLString::equals(XMLString::transcode(nodePosition->getNodeName()), "Y"))
                    {
                        y = atof(XMLString::transcode(nodePosition->getTextContent()));
                    }
                    else if (XMLString::equals(XMLString::transcode(nodePosition->getNodeName()), "Z"))
                    {
                        z = atof(XMLString::transcode(nodePosition->getTextContent()));
                    }
                    else if (XMLString::equals(XMLString::transcode(nodePosition->getNodeName()), "Theta"))
                    {
                        std::string tht = XMLString::transcode(nodePosition->getTextContent());
                        if ((tht == std::string("NAN")) ||
                            (tht == std::string("nan")))
                        {
                            fTheta = NAN;
                            //ROS_INFO("Theta NAN FOUND");
                        }
                        else
                        {
                            fTheta = atof(XMLString::transcode(nodePosition->getTextContent()));
                            //ROS_INFO("Theta string:%s",tht.c_str());
                        }
                    }
                    else if (XMLString::equals(XMLString::transcode(nodePosition->getNodeName()), "Frame"))
                    {
                        sFrame = XMLString::transcode(nodePosition->getTextContent());
                        //ROS_INFO("Frame = %s", sFrame.c_str());
                    }
                }
            }
            else if (XMLString::equals(XMLString::transcode(nodeElement->getNodeName()), "m_sName"))
            { // NODE NAME
                nodeName = XMLString::transcode(nodeElement->getTextContent());
            }
            else if (XMLString::equals(XMLString::transcode(nodeElement->getNodeName()), "m_iZone"))
            { // ZONE
                nodeZone = XMLString::parseInt(nodeElement->getTextContent());
            }

            else if (XMLString::equals(XMLString::transcode(nodeElement->getNodeName()), "m_ArcList"))
            {
                // Añadimos nodo al grafo
                dijkstraGraph->addNode(nodeNumber, x, y, z, fTheta, sFrame, nodeName);

                listArcs = nodeElement->getChildNodes(); // Lista de arcos
                lenArcs = listArcs->getLength();

                // CARC
                for (k = 0; k < lenArcs; k++)
                {
                    arc = listArcs->item(k);
                    listArcElements = arc->getChildNodes();
                    lenArcElements = listArcElements->getLength();
                    if (lenArcElements > 0)
                        for (l = 0; l < lenArcElements; l++)
                        {
                            arcElement = listArcElements->item(l);
                            // NODE DEST
                            nodeDst = -1;
                            if (XMLString::equals(XMLString::transcode(arcElement->getNodeName()), "m_iNodeDst"))
                            {
                                nodeDst = XMLString::parseInt(arcElement->getTextContent());
                            }
                            else if (XMLString::equals(XMLString::transcode(arcElement->getNodeName()), "m_fArcSpeed"))
                            {
                                speed = atof(XMLString::transcode(arcElement->getTextContent()));

                                //dijkstraGraph->addArc(nodeNumber, nodeDst, speed);
                            }
                            //ROS_INFO("0 Adding Arc from %d to %d",nodeNumber, nodeDst);
                            if (nodeNumber != nodeDst)
                            {
                                //ROS_INFO("1 Adding Arc from %d to %d",nodeNumber, nodeDst);
                                if (nodeDst >= 0)
                                {
                                    dijkstraGraph->addArc(nodeNumber, nodeDst, speed);
                                    ROS_INFO("2 Adding Arc from %d to %d", nodeNumber, nodeDst);
                                }
                            }
                        }
                }
            }
            /*
            for(int i=0;i<dijkstraGraph->vNodes.size();i++){
                Node nod=dijkstraGraph->vNodes[i];
                ROS_INFO("Node:%d arcs:%d",nod.iNode,(int)nod.vAdjacent.size());

            }*/
        }
    }

    return len;
}

/*! \fn void Graph::print ()
 * 	\brief
*/
void Graph::print()
{
    dijkstraGraph->printNodes();
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
        if (!dijkstraGraph->getNodePosition(route[i], &pos.x, &pos.y, &pos.theta))
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
    if (!dijkstraGraph->getNodePosition(route[i], &pos.x, &pos.y, &pos.theta))
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
        if (!dijkstraGraph->getNodePosition((*detailed_nodes)[i].iNode, &pos.x, &pos.y, &pos.theta))
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
    if (!dijkstraGraph->getNodePosition(num_node, &x, &y, &a))
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