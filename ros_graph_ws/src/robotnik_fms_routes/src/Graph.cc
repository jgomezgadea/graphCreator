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
bool DOMCountErrorHandler::handleError(const DOMError& domError)
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
Graph::Graph(const char *xmlFile){
	strcpy(cXMLFile, xmlFile);
	dijkstraGraph = new Dijkstra ();
	bInitialized = false;
    magnets = nodes = zones = 0;
}

/*! \fn Graph::~Graph()
 * 	\brief Destructor
*/
Graph::~Graph(){
	delete dijkstraGraph;
}



/*! \fn int Graph::Setup()
 * 	\brief Configures and initializes the graph
 *  @return OK
 *  @return ERROR
 *  @return INITIALIZED
*/
int Graph::Setup(string *msg){
    ROS_INFO("Graph::Start Setup");

	if(bInitialized){
		ROS_ERROR("Graph::Setup: Already initialized");
		//printf("Graph::Setup: Already initialized\n");
        *msg="Graph::Setup: Already initialized";
		return INITIALIZED;
	}
    std::string desmsg;
    if(Deserialize (&desmsg) == ERROR){
		ROS_ERROR("Graph::Setup: Error deserializing the graph");
        *msg="Graph::Setup: Error deserializing the graph:"+desmsg;
		return ERROR;
	}
    desmsg="";
    if(dijkstraGraph->FinalizeEdition(&desmsg) == -1){
		ROS_ERROR("Graph::Setup: Error graph");
        *msg="Graph::Setup: Error graph:"+desmsg;
		return ERROR;
	}

    ROS_INFO("Graph::End Setup OK");
	bInitialized = true;
	return OK;
}

/*! \fn int Graph::ShutDown()
 * 	\brief Closes and frees the reserved resources
 *  @return OK
 *  @return NOT_INITIALIZED
*/
int Graph::ShutDown(){
	if(!bInitialized){
		ROS_ERROR("Graph::ShutDown: Impossible because of it's not initialized");
		return NOT_INITIALIZED;
	}

	dijkstraGraph->DeleteAll();

	bInitialized = false;

	return OK;
}

/*! \fn int Graph::Deserialize()
 * 	\brief Process the data read previouly from a xml document
*/
int Graph::Deserialize(std::string *msg){
	//
	// Initialize the XML4C2 system.
	try  {
        XMLPlatformUtils::Initialize();

    }catch(const XMLException& toCatch) {
        char *pMsg = XMLString::transcode(toCatch.getMessage());
		ROS_ERROR("Graph::Deserialize: Error during Xerces-c Initialization: %s", pMsg);
        *msg="Graph::Deserialize: Error during Xerces-c Initialization:"+std::string(pMsg);
		 //XERCES_STD_QUALIFIER cerr << "XMLParser::InitializeXML: Error during Xerces-c Initialization.\n"
         //    << "  Exception message:"
        //     << pMsg;
        XMLString::release(&pMsg);
        return ERROR;
    }

    AbstractDOMParser::ValSchemes valScheme = AbstractDOMParser::Val_Auto;
	bool doNamespaces       = false;
	bool doSchema           = false;
	bool schemaFullChecking = false;
//	bool doList = false;
	bool errorOccurred = false;
//	bool recognizeNEL = false;
//  bool printOutEncounteredEles = true;
	char localeStr[64];
    memset(localeStr, 0, sizeof localeStr);
	//
    // Instantiate the DOM parser
    static const XMLCh gLS[] = { chLatin_L, chLatin_S, chNull };
    DOMImplementation *impl = DOMImplementationRegistry::getDOMImplementation(gLS);
    DOMLSParser       *parser = ((DOMImplementationLS*)impl)->createLSParser(DOMImplementationLS::MODE_SYNCHRONOUS, 0);
    DOMConfiguration  *config = parser->getDomConfig();

    config->setParameter(XMLUni::fgDOMNamespaces, doNamespaces);
    config->setParameter(XMLUni::fgXercesSchema, doSchema);
    config->setParameter(XMLUni::fgXercesSchemaFullChecking, schemaFullChecking);

    if (valScheme == AbstractDOMParser::Val_Auto) {
        config->setParameter(XMLUni::fgDOMValidateIfSchema, true);

    }else if (valScheme == AbstractDOMParser::Val_Never)  {
        config->setParameter(XMLUni::fgDOMValidate, false);

    }else if (valScheme == AbstractDOMParser::Val_Always)  {
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

    ROS_INFO("Graph::Deserialize File: %s",cXMLFile);
    //reset error count first
    errorHandler.resetErrors();

    XERCES_CPP_NAMESPACE_QUALIFIER DOMDocument *doc = 0;

    try {
        // reset document pool
        parser->resetDocumentPool();

        const unsigned long startMillis = XMLPlatformUtils::getCurrentMillis();
        doc = parser->parseURI(cXMLFile);
        const unsigned long endMillis = XMLPlatformUtils::getCurrentMillis();
        duration = endMillis - startMillis;

    } catch (const XMLException& toCatch)    {

		ROS_ERROR("Graph::Deserialize: Error during parsing %s file",cXMLFile);//, StrX(toCatch.getMessage()));

       // XERCES_STD_QUALIFIER cerr << "\nError during parsing: '" << cXMLFile << "'\n"
       //      << "Exception message is:  \n"
       //      << StrX(toCatch.getMessage()) << "\n" << XERCES_STD_QUALIFIER endl;

        *msg="Graph::Deserialize: Error during parsing "+std::string(cXMLFile)+" file";
        errorOccurred = true;

    } catch (const DOMException& toCatch)    {

        const unsigned int maxChars = 2047;
        XMLCh errText[maxChars + 1];
		ROS_ERROR("Graph::Deserialize: DOM Error during parsing %s file. DOMException code is: %d",cXMLFile, toCatch.code);
        *msg = "Graph::Deserialize: DOM Error during parsing "+std::string(cXMLFile)+" file. DOMException code is:"+std::to_string(toCatch.code);
        //XERCES_STD_QUALIFIER cerr << "\nDOM Error during parsing: '" << cXMLFile << "'\n"
        //     << "DOMException code is:  " << toCatch.code << XERCES_STD_QUALIFIER endl;

        if (DOMImplementation::loadDOMExceptionMsg(toCatch.code, errText, maxChars)){
            //XERCES_STD_QUALIFIER cerr << "Message is: " << StrX(errText) << XERCES_STD_QUALIFIER endl;
			ROS_ERROR("Graph::Deserialize: Error");
		}
        errorOccurred = true;

    }    catch (...)    {
       // XERCES_STD_QUALIFIER cerr << "\nUnexpected exception during parsing: '" << cXMLFile << "'\n";
		ROS_ERROR("Graph::Deserialize: Unexpected exception during parsing %s file", cXMLFile);
        *msg = "Graph::Deserialize: Unexpected exception during parsing "+std::string(cXMLFile)+" file";
		errorOccurred = true;
    }

    //
    //  Extract the DOM tree, get the list of all the elements and report the
    //  length as the count of elements.
    if (errorHandler.getSawErrors())   {
       // XERCES_STD_QUALIFIER cout << "\nErrors occurred, no output available\n" << XERCES_STD_QUALIFIER endl;
        ROS_ERROR("Graph::Deserialize:Errors occurred, no output available");
        *msg = "Graph::Deserialize:Errors occurred, no output available";
		errorOccurred = true;

    }  else   {
        if (doc) {
			//
			// Importante procesar antes magnets que nodos, puesto que los arcos de los nodos, referencian magnets

			magnets = ProcessMagnetsFromXML(doc);
            nodes   = ProcessNodesFromXML(doc);
            zones   = ProcessZonesFromXML(doc);
            ROS_INFO("Graph::Deserialize: Processing XML Found %d Nodes, %d Zones, %d Magnets",nodes,zones,magnets);
            *msg="Graph::Deserialize: Processing XML Found "+std::to_string(nodes)+" Nodes,"+std::to_string(zones)+" Zones"+std::to_string(magnets)+" Magnets";
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

/*! \fn int Graph::ProcessZonesFromXML(DOMDocument *doc)
 * 	\brief
*/
int Graph::ProcessZonesFromXML(DOMDocument *doc){
    DOMNode *zone, *zoneElement;
    XMLCh* tmpstr;
    int ilen = 0, ilenZoneElements = 0;
    int iZoneNumber = 0;
    int iZoneMaxRobots = 0;
    int iComplementary = -1;

    int Nodes[200];

    DOMNodeList *listZoneElements,*list;

    tmpstr = XMLString::transcode("CZone");
    list = doc->getElementsByTagName(tmpstr);
    ilen = list->getLength();

    //CMAGNETS
    for(int i=0; i< ilen ;i++){
        int iNumNodes=0;
        bool bMan=false;
        int iNodeDest=-1;
        iComplementary = -1;

        zone = list->item(i);
        listZoneElements = zone->getChildNodes();
        ilenZoneElements = listZoneElements->getLength();
        //Elements of CMagnet
        for(int j = 0; j < ilenZoneElements; j++){
            zoneElement = listZoneElements->item(j);
            //
            // NUMBER
            if(XMLString::equals(XMLString::transcode(zoneElement->getNodeName()),"m_iIDZone")){
                iZoneNumber = XMLString::parseInt(zoneElement->getTextContent());
            } else if(XMLString::equals(XMLString::transcode(zoneElement->getNodeName()),"m_iMaxRobots")){

                iZoneMaxRobots = atof(XMLString::transcode(zoneElement->getTextContent()));

            } else if(XMLString::equals(XMLString::transcode(zoneElement->getNodeName()),"m_iCompZone")){

                iComplementary = atoi(XMLString::transcode(zoneElement->getTextContent()));

            } else if(XMLString::equals(XMLString::transcode(zoneElement->getNodeName()),"m_bManeuver")){

                bMan = atof(XMLString::transcode(zoneElement->getTextContent()));

            } else if(XMLString::equals(XMLString::transcode(zoneElement->getNodeName()),"m_iNodeDest")){

                iNodeDest = atof(XMLString::transcode(zoneElement->getTextContent()));

            } else if(XMLString::equals(XMLString::transcode(zoneElement->getNodeName()),"m_iNode")){

                Nodes[iNumNodes] = atof(XMLString::transcode(zoneElement->getTextContent()));

                iNumNodes++;
            }
        }

        dijkstraGraph->AddZone(iZoneNumber,iZoneMaxRobots,bMan,iNodeDest,iComplementary);
        for (int i=0; i<iNumNodes;i++){
            ROS_INFO(" Adding Node:%d to Zone:%d,Compl:%d",Nodes[i],iZoneNumber,iComplementary);
            dijkstraGraph->AddNodeToZone(Nodes[i],iZoneNumber,bMan);
            ROS_INFO(" Added Node:%d to Zone:%d,Compl:%d",Nodes[i],iZoneNumber,iComplementary);
        }
    }

    return ilen;
}


/*! \fn int Graph::ProcessNodesFromXML(DOMDocument *doc)
 * 	\brief
*/
int Graph::ProcessNodesFromXML(DOMDocument *doc){
	DOMNode *node, *nodeElement, *nodePosition, *arc, *arcElement, *magnet;
	XMLCh* tmpstr;
	int len = 0, lenNodeElements = 0, lenArcs = 0, lenArcElements = 0, lenMagnets = 0;
	int i,j,k,l,m;
	double x=0.0, y= 0.0, z= 0.0, speed = 0.0;
	int nodeNumber = 0, nodeType = 0, nodeFloor = 0, nodeLaser_Range, nodeDst = 0, magnetNumber = 0, nodePathMode = 0, nodeZone = 0;
    char *nodeName;//[50]= "\0";
    std::string showerDoor;
    std::string showerFrom;
	DOMNodeList *listNodeElements,*list, *listArcs, *listArcElements, *listMagnets;
	vector<int> vMagnetsOnArc;  // Vector con la lista de imanes sobre los que pasa el arco
	bool nodeAcoustic=false;
	bool nodeCritical=false;
    bool nodeStop=true;
	bool nodeFree=true;
	bool nodeRecalibration=true;
    bool nodeLoad=false;
    bool nodeUnload=false;
    bool nodePrePick=false;
    bool nodePrePlace=false;
    bool nodePostPick=false;
    bool nodePostPlace=false;

    bool nodeMagneticLoad=false;
    bool nodeMagneticUnload=false;

    bool nodeDoNothing=false;
    bool nodeSwitchMap=false;


    string sSwitchMapName="";
    float fSwitchMapX=0.0;
    float fSwitchMapY=0.0;
    float fSwitchMapTheta=0.0;

    bool nodeCharge=false;    
    float fXOffset=0.0;
    float fYOffset=0.0;
    float fTheta=0.0;
    bool bDoorOpen=false;
    bool bDoorClose=false;
    bool bEnterShower=false;
    bool bLeaveShower=false;
    bool bEnterLift=false;
    bool bLeaveLift=false;
    string sLiftFloor="";
    int iDoorId=-1;


    bool nodeElevator=false;
    bool bElevatorGetControl=false;
    bool bElevatorLeaveControl=false;
    bool bElevatorOpenDoor=false;
    bool bElevatorCloseDoor=false;
    int iElevatorID=-1;
    int iElevatorFloor=-1;

    bool nodeMove=false;
    float fXMoveGoal=0.0;
    float fYMoveGoal=0.0;
    float fThetaMoveGoal=0.0;
    float fXMaxVel=0.0;
    float fYMaxVel=0.0;
    float fThetaMaxVel=0.0;


    bool nodeFindMagnetic=false;
    float fFindMagneticDistance=0.0;
    std::string sFindMagneticTurnDirection="right";

    int iMLoadUnloadLane=0;
    int iMLoadUnloadCartPos=0;
    std::string sMLoadUnloadTurn="";
    bool bMLoadUnloadAllowMarkers=false;
    std::string sMLoadUnloadCartId="";

    bool nodeLeaveMagnetic=false;
    std::string sLeaveMagneticTurn;

    std::string sFrame="map";

	tmpstr = XMLString::transcode("CNode");
	list = doc->getElementsByTagName(tmpstr);
	len = list->getLength();

	ROS_INFO("Graph::ProccesNodesFromXML: Processing %d Nodes", len);


	for(i=0; i< len ;i++){
		nodeRecalibration = nodeFree = true;
        nodeAcoustic = nodeCritical = false;
        nodeStop=true;
        bDoorOpen=false;
        bDoorClose=false;
        bEnterShower=false;
        bLeaveShower=false;
        bEnterLift=false;
        bLeaveLift=false;
        sLiftFloor="";
        iDoorId=-1;
        nodeMagneticUnload=nodeMagneticLoad=nodeUnload=nodeLoad=nodeCharge=false;

        nodeDoNothing=false;
        nodeSwitchMap=false;
        sSwitchMapName="";
        fSwitchMapX=0.0;
        fSwitchMapY=0.0;
        fSwitchMapTheta=0.0;

        nodeElevator=false;
        bElevatorGetControl=false;
        bElevatorLeaveControl=false;
        bElevatorOpenDoor=false;
        bElevatorCloseDoor=false;
        iElevatorID=-1;
        iElevatorFloor=-1;


        nodePrePick=false;
        nodePrePlace=false;
        nodePostPick=false;
        nodePostPlace=false;
        nodeFindMagnetic=nodeLeaveMagnetic=false;
        sLeaveMagneticTurn="";

        fFindMagneticDistance=0.0;
        sFindMagneticTurnDirection="right";


        iMLoadUnloadLane=0;
        iMLoadUnloadCartPos=0;
        sMLoadUnloadTurn="";
        bMLoadUnloadAllowMarkers=false;
        sMLoadUnloadCartId="";

        nodeMove=false;
        fXMoveGoal=0.0;
        fYMoveGoal=0.0;
        fThetaMoveGoal=0.0;
        fXMaxVel=0.0;
        fYMaxVel=0.0;
        fThetaMaxVel=0.0;

		//Returns DOMnode object
		node = list->item(i);
		listNodeElements = node->getChildNodes();
		lenNodeElements = listNodeElements->getLength();
		//
		//Elements of the node
		for(j = 0; j < lenNodeElements; j++){
			nodeElement = listNodeElements->item(j);
			// NUMBER
			if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iNumber")){
				nodeNumber = XMLString::parseInt(nodeElement->getTextContent());
                //cout << "Node:" << nodeNumber  << endl;

			// POSITION
			}else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_Position")){
				//cout << "Position:" << endl;
				for (nodePosition = nodeElement->getFirstChild(); nodePosition != 0; nodePosition=nodePosition->getNextSibling()){
					if(XMLString::equals(XMLString::transcode(nodePosition->getNodeName()),"X")){
                        x = atof(XMLString::transcode(nodePosition->getTextContent()));
					}else if(XMLString::equals(XMLString::transcode(nodePosition->getNodeName()),"Y")){
						y = atof(XMLString::transcode(nodePosition->getTextContent()));
					}else if(XMLString::equals(XMLString::transcode(nodePosition->getNodeName()),"Z")){
                        z = atof(XMLString::transcode(nodePosition->getTextContent()));
                    }else if(XMLString::equals(XMLString::transcode(nodePosition->getNodeName()),"Theta")){
                        std::string tht=XMLString::transcode(nodePosition->getTextContent());
                        if ((tht==std::string("NAN")) ||
                            (tht==std::string("nan"))){
                            fTheta = NAN;
                            //ROS_INFO("Theta NAN FOUND");
                        } else {
                            fTheta = atof(XMLString::transcode(nodePosition->getTextContent()));
                            //ROS_INFO("Theta string:%s",tht.c_str());
                        }
                    }else if(XMLString::equals(XMLString::transcode(nodePosition->getNodeName()),"Frame")){
                        sFrame = XMLString::transcode(nodePosition->getTextContent());
                        //ROS_INFO("Frame = %s", sFrame.c_str());
                    }
				}
            //X Offset
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_XOffset")){
                fXOffset = atof(XMLString::transcode(nodeElement->getTextContent()));

            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_YOffset")){
                fYOffset = atof(XMLString::transcode(nodeElement->getTextContent()));

			}else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iType")){
				nodeType = XMLString::parseInt(nodeElement->getTextContent());
                ROS_WARN("Graph::ProccesNodesFromXML: Processing Node Found m_iType Deprecated");
            //SHOWERSIDE
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_sName")){
                nodeName = XMLString::transcode(nodeElement->getTextContent());
            //SHOWERFROM
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_sShowerFrom")){
                showerFrom = XMLString::transcode(nodeElement->getTextContent());
			//NAME
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_sShowerDoor")){
                showerDoor = XMLString::transcode(nodeElement->getTextContent());
			//FLOOR
			}else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iFloor")){
				nodeFloor = XMLString::parseInt(nodeElement->getTextContent());
            //LASER_RANGE
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iLaser_Range")){
				nodeLaser_Range = XMLString::parseInt(nodeElement->getTextContent());
                //ROS_INFO("Graph::Get Laser Range:%d",nodeLaser_Range);
            //ACOUSTIC
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bAcoustic")){
				nodeAcoustic = XMLString::parseInt(nodeElement->getTextContent());

            //CRITICAL
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bCritical")){
				nodeCritical = XMLString::parseInt(nodeElement->getTextContent());

            //Load
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bLoad")){
                nodeLoad = XMLString::parseInt(nodeElement->getTextContent());

            //Unload
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bUnload")){
                nodeUnload = XMLString::parseInt(nodeElement->getTextContent());

            //PrePick
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bPrePick")){
                nodePrePick = XMLString::parseInt(nodeElement->getTextContent());

            //PrePlace
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bPrePlace")){
                nodePrePlace = XMLString::parseInt(nodeElement->getTextContent());

            //PostPick
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bPostPick")){
                nodePostPick = XMLString::parseInt(nodeElement->getTextContent());

            //PostPlace
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bPostPlace")){
                nodePostPlace = XMLString::parseInt(nodeElement->getTextContent());

            //Magnetic Load
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bMagneticLoad")){
                nodeMagneticLoad = XMLString::parseInt(nodeElement->getTextContent());

            //Magnetic Load
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bMagneticUnload")){
                nodeMagneticUnload = XMLString::parseInt(nodeElement->getTextContent());

            //Charge
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bCharge")){
                nodeCharge = XMLString::parseInt(nodeElement->getTextContent());

            //STOP
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bStop")){
				nodeStop = XMLString::parseInt(nodeElement->getTextContent());

            //DoNothing
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bDoNothing")){
                nodeDoNothing = XMLString::parseInt(nodeElement->getTextContent());

            //SwitchMap
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bSwitchMap")){
                nodeSwitchMap = XMLString::parseInt(nodeElement->getTextContent());

            //SwitchMapName
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_sSwitchMapName")){
                sSwitchMapName = XMLString::transcode(nodeElement->getTextContent());

            //SwitchMapX
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fSwitchMapX")){
                fSwitchMapX = atof(XMLString::transcode(nodeElement->getTextContent()));

            //SwitchMapY
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fSwitchMapY")){
                fSwitchMapY = atof(XMLString::transcode(nodeElement->getTextContent()));

            //SwitchMapY
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fSwitchMapTheta")){
                fSwitchMapTheta = atof(XMLString::transcode(nodeElement->getTextContent()));

            // Enter Lift
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bEnterLift")){
                bEnterLift = XMLString::parseInt(nodeElement->getTextContent());

            // Leave Lift
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bLeaveLift")){
                bLeaveLift = XMLString::parseInt(nodeElement->getTextContent());
                //ROS_INFO("Graph::ProccesNodesFromXML:  Node: %d, DOOR CLOSE= %d", nodeNumber, bDoorClose);

            // Leave Lift
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_sLiftFloor")){
                sLiftFloor = XMLString::transcode(nodeElement->getTextContent());

            // Enter Shower
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bEnterShower")){
                bEnterShower = XMLString::parseInt(nodeElement->getTextContent());
                //ROS_INFO("Graph::ProccesNodesFromXML:  Node: %d, DOOR OPEN= %d", nodeNumber, bDoorOpen);

            // Leave Shower
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bLeaveShower")){
                bLeaveShower = XMLString::parseInt(nodeElement->getTextContent());
                //ROS_INFO("Graph::ProccesNodesFromXML:  Node: %d, DOOR CLOSE= %d", nodeNumber, bDoorClose);

            // DOOR OPEN
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bDoorOpen")){
                bDoorOpen = XMLString::parseInt(nodeElement->getTextContent());
                //ROS_INFO("Graph::ProccesNodesFromXML:  Node: %d, DOOR OPEN= %d", nodeNumber, bDoorOpen);

            // DOOR CLOSE
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bDoorClose")){
                bDoorClose = XMLString::parseInt(nodeElement->getTextContent());
                //ROS_INFO("Graph::ProccesNodesFromXML:  Node: %d, DOOR CLOSE= %d", nodeNumber, bDoorClose);

            // DOOR ID
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iDoorID")){
                iDoorId = XMLString::parseInt(nodeElement->getTextContent());
                //ROS_INFO("Graph::ProccesNodesFromXML:  Node: %d, DOOR ID= %d", nodeNumber, iDoorId);

            // FREE
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bFree")){
				nodeFree = XMLString::parseInt(nodeElement->getTextContent());

            // PATH MODE
			}else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iPathMode")){
				nodePathMode = XMLString::parseInt(nodeElement->getTextContent());

			// ZONE
			}else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iZone")){
				nodeZone = XMLString::parseInt(nodeElement->getTextContent());

            // FindMagnetic
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bFindMagnetic")){
                nodeFindMagnetic = XMLString::parseInt(nodeElement->getTextContent());

            // FindMagneticDistance
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fFindMagneticDistance")){
                fFindMagneticDistance = atof(XMLString::transcode(nodeElement->getTextContent()));

	    // FindMagneticTurnDirection
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_sFindMagneticTurnDirection")){
                sFindMagneticTurnDirection = XMLString::transcode(nodeElement->getTextContent());

            // LeaveMagnetic
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bLeaveMagnetic")){
                nodeLeaveMagnetic = XMLString::parseInt(nodeElement->getTextContent());

            // LeaveMagneticTurnDirection
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_sLeaveMagneticTurnDirection")){
                sLeaveMagneticTurn = XMLString::transcode(nodeElement->getTextContent());

            // LoadUnloadLane
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iLoadUnloadLane")){
                iMLoadUnloadLane = XMLString::parseInt(nodeElement->getTextContent());

            // LoadUnloadCartPosition
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iLoadUnloadCartPosition")){
                iMLoadUnloadCartPos = XMLString::parseInt(nodeElement->getTextContent());

            // LoadUnloadTurnDirection
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_sLoadUnloadTurnDirection")){
                sMLoadUnloadTurn = XMLString::transcode(nodeElement->getTextContent());

            // LoadUnloadAllowMarkers
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bLoadUnloadAllowMarkers")){
                bMLoadUnloadAllowMarkers = XMLString::parseInt(nodeElement->getTextContent());

            // LoadUnloadCartId
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_sLoadUnloadCartId")){
                sMLoadUnloadCartId = XMLString::transcode(nodeElement->getTextContent());

            // RECALIBRATION
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bRecalibration")){
				nodeRecalibration = XMLString::parseInt(nodeElement->getTextContent());
				//ROS_INFO("Graph::ProccesNodesFromXML:  Node: %d, recalibration = %d", nodeNumber, nodeRecalibration);

            // LoadUnloadLane
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iLoadUnloadLane")){
                iMLoadUnloadLane = XMLString::parseInt(nodeElement->getTextContent());

            // Elevator
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bElevator")){
                nodeElevator = XMLString::parseInt(nodeElement->getTextContent());

            // ElevatorID
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iElevatorID")){
                iElevatorID = XMLString::parseInt(nodeElement->getTextContent());

            // ElevatorGetControl
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bElevatorGetControl")){
                bElevatorGetControl = XMLString::parseInt(nodeElement->getTextContent());
                ROS_INFO("Elevator_Get:%d",bElevatorGetControl);

            // ElevatorFloor
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iElevatorFloor")){
                iElevatorFloor = XMLString::parseInt(nodeElement->getTextContent());
                //ROS_INFO("Elevator_Floor:%d",iElevatorFloor);

            // ElevatorLeaveControl
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bElevatorLeaveControl")){
                bElevatorLeaveControl = XMLString::parseInt(nodeElement->getTextContent());
                ROS_INFO("Elevator_Leave:%d",bElevatorLeaveControl);

            // ElevatorOpenDoor
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bElevatorOpenDoor")){
                bElevatorOpenDoor = XMLString::parseInt(nodeElement->getTextContent());

            // ElevatorCloseDoor
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bElevatorCloseDoor")){
                bElevatorCloseDoor = XMLString::parseInt(nodeElement->getTextContent());




            // Move Node
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_bMove")){
                nodeMove = XMLString::parseInt(nodeElement->getTextContent());

            // XMoveGoal
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fMoveGoalX")){
                fXMoveGoal = atof(XMLString::transcode(nodeElement->getTextContent()));

            // YMoveGoal
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fMoveGoalY")){
                fYMoveGoal = atof(XMLString::transcode(nodeElement->getTextContent()));

            // ThetaMoveGoal
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fMoveGoalTheta")){
                fThetaMoveGoal = atof(XMLString::transcode(nodeElement->getTextContent()));

            // XMoveGoal
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fMoveVelMaxX")){
                fXMaxVel = atof(XMLString::transcode(nodeElement->getTextContent()));

            // YMoveGoal
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fMoveVelMaxY")){
                fXMaxVel = atof(XMLString::transcode(nodeElement->getTextContent()));

            // ThetaMoveGoal
            }else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_fMoveVelMaxTheta")){
                fThetaMaxVel = atof(XMLString::transcode(nodeElement->getTextContent()));

			//ARCLIST
			}else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_ArcList")){
				// Añadimos nodo al grafo

                dijkstraGraph->AddNode(nodeNumber,nodeType,nodeFloor,nodeLaser_Range,x,y,z,fTheta,sFrame,fXOffset,fYOffset,
                                       nodeName,nodeAcoustic,nodeCritical,nodeFree,nodeStop,nodePathMode,nodeRecalibration,
                                       nodeCharge,nodeLoad,nodeUnload,nodePrePick,nodePrePlace,nodePostPick,nodePostPlace,
                                       nodeElevator,bDoorOpen,bDoorClose,iDoorId,
                                       bEnterLift,bLeaveLift,sLiftFloor,bEnterShower,bLeaveShower,showerDoor,showerFrom,
                                       nodeMagneticLoad,nodeMagneticUnload,nodeFindMagnetic,nodeLeaveMagnetic,
                                       fFindMagneticDistance,sFindMagneticTurnDirection,sLeaveMagneticTurn,
                                       iMLoadUnloadLane,iMLoadUnloadCartPos,sMLoadUnloadTurn,bMLoadUnloadAllowMarkers,
                                       sMLoadUnloadCartId);


                dijkstraGraph->AddElevatorParams(nodeNumber,nodeElevator,iElevatorID,bElevatorGetControl,bElevatorLeaveControl,bElevatorOpenDoor,bElevatorCloseDoor,iElevatorFloor);

                dijkstraGraph->AddDoNothing(nodeNumber,nodeDoNothing);

                dijkstraGraph->AddSwitchMap(nodeNumber,nodeSwitchMap,sSwitchMapName,fSwitchMapX,fSwitchMapY,fSwitchMapTheta);

                dijkstraGraph->AddMoveParams(nodeNumber,nodeMove,fXMoveGoal,fYMoveGoal,fThetaMoveGoal,fXMaxVel,fYMaxVel,fThetaMaxVel);

				listArcs = nodeElement->getChildNodes();	//Lista de arcos
				lenArcs = listArcs->getLength();

				// CARC
				for( k = 0; k < lenArcs; k++ ){
					arc = listArcs->item(k);
					listArcElements = arc->getChildNodes();
					lenArcElements = listArcElements->getLength();
					if(lenArcElements > 0)
                        //cout << "Arc " << endl;

					for(l=0; l < lenArcElements; l++){
						arcElement = listArcElements->item(l);
						// NODE DEST
                        nodeDst=-1;
						if(XMLString::equals(XMLString::transcode(arcElement->getNodeName()),"m_iNodeDst")){
							nodeDst = XMLString::parseInt(arcElement->getTextContent());

                            //cout << " Node dest:" << nodeDst << endl;
						// MAGNETLIST
						}else if(XMLString::equals(XMLString::transcode(arcElement->getNodeName()),"m_MagnetList")){
							//cout << " Magnets:" << endl;
							vMagnetsOnArc.clear();  //Limpiamos vector
							listMagnets = arcElement->getChildNodes();
							lenMagnets = listMagnets->getLength();
							for(m = 0; m < lenMagnets; m++){
								magnet = listMagnets->item(m);
								if(XMLString::equals(XMLString::transcode(magnet->getNodeName()),"int")){
									magnetNumber = XMLString::parseInt(magnet->getTextContent());
									vMagnetsOnArc.push_back(magnetNumber);

								}
							}
							//
							// Añadimos arco
							//dijkstraGraph->AddArc(nodeNumber, nodeDst, vMagnetsOnArc);
						// SPEED
						}else if(XMLString::equals(XMLString::transcode(arcElement->getNodeName()),"m_fArcSpeed")){
							speed = atof(XMLString::transcode(arcElement->getTextContent()));

                            //dijkstraGraph->AddArc(nodeNumber, nodeDst, speed, vMagnetsOnArc);
						}
                        //ROS_INFO("0 Adding Arc from %d to %d",nodeNumber, nodeDst);
                        if (nodeNumber!=nodeDst) {
                            //ROS_INFO("1 Adding Arc from %d to %d",nodeNumber, nodeDst);
                            if (nodeDst>=0) {
                                dijkstraGraph->AddArc(nodeNumber, nodeDst, speed, vMagnetsOnArc);
                                ROS_INFO("2 Adding Arc from %d to %d",nodeNumber, nodeDst);
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

/*! \fn int Graph::ProcessMagnetsFromXML(DOMDocument *doc)
 * 	\brief
*/
int Graph::ProcessMagnetsFromXML(DOMDocument *doc){
	DOMNode *node, *nodeElement, *nodePosition;
	XMLCh* tmpstr;
	int len = 0, lenNodeElements = 0;
	int i,j;
	double x=0.0, y= 0.0, z= 0.0;
	int magnetNumber = 0;


	DOMNodeList *listNodeElements,*list;

	tmpstr = XMLString::transcode("CMagnet");
	list = doc->getElementsByTagName(tmpstr);
	len = list->getLength();

	ROS_INFO("Graph::ProcessMagnetsFromXML: Processing %d Magnets", len);

	//CMAGNETS
	for(i=0; i< len ;i++){

		//Returns DOMnode object
		node = list->item(i);
		listNodeElements = node->getChildNodes();
		lenNodeElements = listNodeElements->getLength();
		//Elements of CMagnet
		for(j = 0; j < lenNodeElements; j++){
			nodeElement = listNodeElements->item(j);
			//
			// NUMBER
			if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_iNumber")){
				magnetNumber = XMLString::parseInt(nodeElement->getTextContent());

				//cout << "Magnet number:" << magnetNumber << endl;
			//
			// POSITION
			}else if(XMLString::equals(XMLString::transcode(nodeElement->getNodeName()),"m_Position")){
				//cout << "Position:" << endl;
				for (nodePosition = nodeElement->getFirstChild(); nodePosition != 0; nodePosition=nodePosition->getNextSibling()){
					if(XMLString::equals(XMLString::transcode(nodePosition->getNodeName()),"X")){
						x = atof(XMLString::transcode(nodePosition->getTextContent()));

						//cout << "  X =" << x << endl;
					}else if(XMLString::equals(XMLString::transcode(nodePosition->getNodeName()),"Y")){
						y = atof(XMLString::transcode(nodePosition->getTextContent()));

						//cout << "  Y =" << y << endl;
					}else if(XMLString::equals(XMLString::transcode(nodePosition->getNodeName()),"Z")){
						z = atof(XMLString::transcode(nodePosition->getTextContent()));

						//cout << "  Z =" << z << endl;
					}
				}
				//
				// Añadimos magnet
				dijkstraGraph->AddMagnet(magnetNumber, x, y, z);
			}
		}
		//cout << endl;
	}


	return len;
}

/*! \fn void Graph::Print ()
 * 	\brief
*/
void Graph::Print (){
	dijkstraGraph->PrintNodes();
	dijkstraGraph->PrintMagnets();

}

/*! \fn int Graph::GetMagnets()
 * 	\brief
*/
int Graph::GetMagnets(){
	return magnets;
}

/*! \fn int Graph::GetNodes()
 * 	\brief
*/
int Graph::GetNodes(){
	return nodes;
}

/*! \fn int Graph::GetRoute(int from, int to, vector<int> *route)
 * 	\brief Gets the list of nodes from initial node "from" to the end node "to"
*/
int Graph::GetRoute(int from, int to, vector<int> *route){
	return  dijkstraGraph->GetRoute(from, to, route);
}

/*! \fn int Graph::GetNodesUsed(vector<Node *> *route)
 * 	\brief Gets the list of nodes used
*/
bool Graph::GetNodesUsed(std::vector<Node *> *route){
    return dijkstraGraph->GetNodesUsed(route);
}

/*! \fn int Graph::ReserveNode(int iRobot,int iIDNode)
 * 	\brief Reserve Node
*/
bool Graph::ReserveNode(int iRobot,int iIDNode){
    return dijkstraGraph->ReserveNode(iRobot,iIDNode);
}

/*! \fn int Graph::UnBlockAll(int iRobot)
 * 	\brief UnBlock all nodes
*/
bool Graph::UnBlockAll(int iRobot){
    return dijkstraGraph->UnBlockAll(iRobot);
}


/*! \fn int Graph::GetRoute(int from, int to, vector<geometry_msgs::Pose2D> *nodes, vector<geometry_msgs::Pose2D> *magnets){
 * 	\brief Obtiene las coordenadas de los nodos y de los imanes ordenadas para esa trayectoria, además de las velocidades entre dichos nodos
*/
int Graph::GetRoute(int from, int to, vector<geometry_msgs::Pose2D> *nodes, vector<geometry_msgs::Pose2D > *magnets, vector<double> *speed_between_nodes){
	vector <int> route;
	int i = 0;
	geometry_msgs::Pose2D  pos;
	vector<Magnet> vpMagnets;
	double speed = 0.0;

	if(dijkstraGraph->GetRoute(from, to, &route) !=  0) // Ruta incorrecta
		return -1;

	for(i = 0; i < (int)(route.size() - 1); i++){  // Recorremos los nodos de la ruta hasta el penultimo
		if(!dijkstraGraph->GetNodePosition(route[i], &pos.x, &pos.y, &pos.theta)){
			nodes->push_back(pos);
			vpMagnets.clear();
			if(dijkstraGraph->GetArcBetweenNodes(route[i], route[i+1], &vpMagnets, &speed) != 0){ // Obtenemos los imanes en la ruta al nodo adyacente
				//cout << "Graph::GetRoute: Error getting the magnets from " << route[i] << "to " << route[i+1] << endl;
				ROS_ERROR("Graph::GetRoute: Error getting the magnets from %d to %d", route[i],route[i+1]);

				return -1;
			}else{
				//cout << "Graph::GetRoute:speed = " << speed << endl;
				speed_between_nodes->push_back(speed);
				for(int j = 0; j < (int)vpMagnets.size(); j++){
					//Magnet *aux = vpMagnets[j];
					pos.x = vpMagnets[j].dX;
					pos.y = vpMagnets[j].dY;
					magnets->push_back(pos);
				}
			}
		}else{
			//cout << "Graph::GetRoute: Error getting node " << route[i] << endl;
			ROS_ERROR("Graph::GetRoute: Error getting node %d", route[i]);

			return -1;
		}
	}
	// El último nodo
	if(!dijkstraGraph->GetNodePosition(route[i], &pos.x, &pos.y, &pos.theta)){
		nodes->push_back(pos);
	} else {
		//cout << "Graph::GetRoute: Error getting node " << route[i] << endl;
		ROS_ERROR("Graph::GetRoute: Last node: Error getting node %d", route[i]);

		return -1;
	}

	return 0;

}

/*! \fn int Graph::GetRoute(int from, int to, vector<Node> *detailed_nodes, vector<geometry_msgs::Pose2D> *nodes, vector<geometry_msgs::Pose2D> *magnets){
 * 	\brief Misma función salvo que también obtiene los nodos de la ruta en detalle, no solamente su posición
*/
int Graph::GetRoute(int from, int to, vector<Node> *detailed_nodes, vector<geometry_msgs::Pose2D> *nodes, vector<geometry_msgs::Pose2D> *magnets, vector<double> *speed_between_nodes){

	if(dijkstraGraph->GetRoute(from, to, detailed_nodes) !=  0) // Ruta incorrecta
		return -1;

    if(GetRoute(from, to, nodes, magnets, speed_between_nodes) != 0)    // Llamamos a la función de obtener ruta normal
        return -1;

	return 0;

}

/*! \fn int Graph::GetRoute(int from, int to, vector<Node> *detailed_nodes, vector<Magnet> *detailed_magnets, vector<double> *speed_between_nodes){
 * 	\brief Misma función salvo que también obtiene los nodos de la ruta en detalle, no solamente su posición
*/
int Graph::GetRoute(int from, int to, vector<Node> *detailed_nodes, vector<Magnet> *detailed_magnets, vector<double> *speed_between_nodes){
	vector <int> route;
	int i = 0;
	geometry_msgs::Pose2D  pos;
	vector<Magnet> vpMagnets;
	double speed = 0.0;


	if(dijkstraGraph->GetRoute(from, to, detailed_nodes) !=  0) // Ruta incorrecta
		return -1;

	for(i = 0; i < (int)(detailed_nodes->size() - 1); i++){  // Recorremos los nodos de la ruta hasta el penultimo
		if(!dijkstraGraph->GetNodePosition((*detailed_nodes)[i].iNode, &pos.x, &pos.y, &pos.theta)){
			vpMagnets.clear();
			if(dijkstraGraph->GetArcBetweenNodes((*detailed_nodes)[i].iNode, (*detailed_nodes)[i+1].iNode, &vpMagnets, &speed) != 0){ // Obtenemos los imanes en la ruta al nodo adyacente
				//cout << "Graph::GetRoute: Error getting the magnets from " << route[i] << "to " << route[i+1] << endl;
				ROS_ERROR("Graph::GetRoute: Error getting the magnets from %d to %d", (*detailed_nodes)[i].iNode,(*detailed_nodes)[i+1].iNode);
				return -1;
			}else{
				//cout << "Graph::GetRoute:speed = " << speed << endl;
				speed_between_nodes->push_back(speed);
				for(int j = 0; j < (int)vpMagnets.size(); j++){
					//cout << "Adding magnet " << vpMagnets[j].iNumber << endl;

					detailed_magnets->push_back(vpMagnets[j]);
				}
			}
		}else{
			//cout << "Graph::GetRoute: Error getting node " << route[i] << endl;
			ROS_ERROR("Graph::GetRoute: Error getting node %d", route[i]);
			return -1;
		}
	}

	return 0;

};

/*! \fn int Graph::GetNodePosition(int num_node, geometry_msgs::Pose2D *pos)
 * 	\brief Obtiene la posición del nodo
 *  \return 0 if OK
 *  \return -1 si el nodo no existe
*/
int Graph::GetNodePosition(int num_node, geometry_msgs::Pose2D  *pos){
	double x = 0.0, y = 0.0, a = 0.0;
	if(!dijkstraGraph->GetNodePosition(num_node, &x, &y, &a)){
		pos->x = x;
		pos->y = y;
		pos->theta = a;
		return 0;
	}
	return -1;
}

/*! \fn Node* Dijkstra::GetNode(unsigned int nodeID)
 * 	\brief Gets Node by nodeID
*/
Node* Graph::GetNode(unsigned int node_id){

	return dijkstraGraph->GetNode(node_id);
}

/*! \fn Node* Dijkstra::CheckNodeFree(int idNode, int idRobot)
 * true if node Free, false if used or reserved by a robot
*/
bool Graph::CheckNodeFree(int idNode, int idRobot){
    return dijkstraGraph->CheckNodeFree(idNode,idRobot);
}

/*! \fn Node* Dijkstra::CheckNodesFree(std::vector<int> idNode, int idRobot)
 * true if node Free, false if used or reserved by a robot
*/
bool Graph::CheckNodesFree(std::vector<int> vNodesId, int idRobot){
    bool bfree=true;
    for (int i=0;i<vNodesId.size();i++){
        if (!dijkstraGraph->CheckNodeFree(vNodesId[i],idRobot)) bfree=false;
    }
    return bfree;
}

/*! \fn Node* Dijkstra::CheckZoneFree(int idZone, int idRobot)
 * true if Zone Free, false if used or reserved by a robot
*/
bool Graph::CheckZoneFree(int idZone, int idRobot){
    return dijkstraGraph->CheckZoneFree(idZone,idRobot);
}
