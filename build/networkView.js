/**
 * @author Jose Gómez - jgomez@robotnik.es
 */

var NETWORKVIEW = NETWORKVIEW || {
  REVISION: '0.1.0'
};

NETWORKVIEW.NetworkView = function (options) {
  options = options || {};
  this.divID = options.divID || 'network';

  this.message = undefined;
  this.nodes = new vis.DataSet();
  this.edges = new vis.DataSet();

  var self = this;

  // SERVICES

  // add_node service
  this.addNodeService = new ROSLIB.Service({
    ros: options.ros,
    name: options.rosAddNodeService || '/robotnik_fms_routes_node/add_node',
    serviceType: options.rosAddNodeMsg || 'graph_msgs/Node'
  })

  // set_node service
  this.setNodeService = new ROSLIB.Service({
    ros: options.ros,
    name: options.rosSetNodeService || '/robotnik_fms_routes_node/set_node',
    serviceType: options.rosSetNodeMsg || 'graph_msgs/Node'
  })

  // delete_node service
  this.deleteNodeService = new ROSLIB.Service({
    ros: options.ros,
    name: options.rosDeleteNodeService || '/robotnik_fms_routes_node/delete_node',
    serviceType: options.rosDeleteNodeMsg || 'graph_msgs/NodeId'
  })

  // add_arc service
  this.addArcService = new ROSLIB.Service({
    ros: options.ros,
    name: options.rosAddArcService || '/robotnik_fms_routes_node/add_arc',
    serviceType: options.rosAddArcMsg || 'graph_msgs/ArcId'
  })

  // set_arc_pos service
  this.setArcPosService = new ROSLIB.Service({
    ros: options.ros,
    name: options.rosSetArcPosService || '/robotnik_fms_routes_node/set_arc_pos',
    serviceType: options.rosSetArcPosMsg || 'graph_msgs/ArcId'
  })

  // delete_arc service
  this.deleteArcService = new ROSLIB.Service({
    ros: options.ros,
    name: options.rosDeleteArcService || '/robotnik_fms_routes_node/delete_arc',
    serviceType: options.rosDeleteArcMsg || 'graph_msgs/ArcId'
  })

  // TOPICS

  // graph topic
  this.rosNodesTopic = new ROSLIB.Topic({
    ros: options.ros,
    name: options.rosNodesTopic || '/robotnik_fms_routes_node/graph',
    messageType: options.rosNodesMsg || 'graph_msgs/GraphNodeArray'
  })

  // graph callback
  this.rosNodesTopic.subscribe(function (message) {

    if (self.hasChanged(message)) {

      // Delete actual network
      self.deleteNetwork();

      // Fill network with received nodes
      // TODO Puede que no nos lleguen ordenados los id's (ni que estén todos)
      message.nodes.forEach(node => {
        self.addNode(node);
        node.arc_list.forEach(arc => {
          self.addEdge(node.id, arc);
        })
      });

      self.stabilize();
      //self.rosNodesTopic.unsubscribe();
    }
    self.message = message;

  });

  // Create the network
  this.container = document.getElementById(this.divID);
  this.data = {
    nodes: this.nodes,
    edges: this.edges
  };

  // Add buttons callback
  this.options = {
    interaction: { hover: true },
    manipulation: {
      enabled: true,
      initiallyActive: true,

      // ADD NODE function
      addNode: function (data, callback) {
        addNode(data, cancelNodeEdit, callback);
      },

      // ADD EDGE function
      addEdge: function (data, callback) {
        // Check is not a cycle
        if (data.from !== data.to) {
          // Check edge not exist
          if (self.edges.get(data.from + " " + data.to) === null) {
            var request = new ROSLIB.ServiceRequest({
              from_id: data.from,
              to_id: data.to
            });
            self.addArcService.callService(request, function (result) {
              console.log(result.message);
            });
            data.arrows = 'to';
            data.id = data.from + " " + data.to;
            callback(data);
          } else {
            console.log("Error: The edge already exists");
            callback();
          }
        }
        else {
          console.log("Error: Graph cycles are not allowed");
          callback();
        }
      },

      // EDIT NODE function
      editNode: function (data, callback) {
        editNode(data, cancelNodeEdit, callback);
      },

      // EDIT EDGE function
      editEdge: function (data, callback) {
        var old = data.id.split(" ");
        // Checking if a modification has been realized
        if (old[0] === data.from && old[1] === data.to) {
          callback(data);
        } else {
          // Check is not a cycle
          if (data.from !== data.to) {
            // Check edge not exist
            if (self.edges.get(data.from + " " + data.to) === null) {
              var request = new ROSLIB.ServiceRequest({
                from_id_old: old[0],
                to_id_old: old[1],
                from_id: data.from,
                to_id: data.to
              });
              self.setArcPosService.callService(request, function (result) {
                console.log(result.message);
              });
              data.id = data.from + " " + data.to;
              data.arrows = 'to';
              callback(data);
              self.edges.remove(old[0] + " " + old[1]);
            } else {
              console.log("Error: The edge already exists");
              callback();
            }
          }
          else {
            console.log("Error: Graph cycles are not allowed");
            callback();
          }
        }
      },

      // DELETE NODE function
      deleteNode: function (data, callback) {
        var request = new ROSLIB.ServiceRequest({
          node_id: data.nodes[0]
        });
        self.deleteNodeService.callService(request, function (result) {
          console.log(result.message);
        });
        callback(data);
      },

      // DELETE EDGE function
      deleteEdge: function (data, callback) {
        var ids = data.edges[0].split(" ");
        var request = new ROSLIB.ServiceRequest({
          from_id: ids[0],
          to_id: ids[1]
        });
        self.deleteArcService.callService(request, function (result) {
          console.log(result.message);
        });
        callback(data);
      },

      // NODE STYLE
      controlNodeStyle: {
        // all node options are valid.
      }
    }
  };

  // FUNCTIONS

  function addNode(data, cancelAction, callback) {
    // Load actual values
    document.getElementById('node-label').value = data.label;
    document.getElementById('node-zone').value = 0;
    data.zone = 0;
    document.getElementById('node-x').value = 0;
    data.x = 0;
    document.getElementById('node-y').value = 0;
    data.y = 0;
    document.getElementById('node-theta').value = 0;
    data.theta = 0;
    document.getElementById('node-frame').value = "map";
    data.frame = "map";
    // Save new values
    document.getElementById('node-saveButton').onclick = saveNodeData.bind(this, data, callback);
    // Cancel
    document.getElementById('node-cancelButton').onclick = cancelAction.bind(this, callback);
    document.getElementById('node-popUp').style.display = 'block';
  }

  function editNode(data, cancelAction, callback) {
    // Load actual values
    var node;
    self.message.nodes.forEach(n => {
      if (n.id === data.id) {
        node = n;
      }
    });
    document.getElementById('node-label').value = node.name;
    document.getElementById('node-zone').value = node.zone;
    data.zone = node.zone;
    document.getElementById('node-x').value = node.pose.x;
    data.x = node.pose.x;
    document.getElementById('node-y').value = node.pose.y;
    data.y = node.pose.y;
    document.getElementById('node-theta').value = node.pose.theta;
    data.theta = node.pose.theta;
    document.getElementById('node-frame').value = node.pose.frame_id;
    data.frame = node.pose.frame_id;
    // Save new values
    document.getElementById('node-saveButton').onclick = editNodeData.bind(this, data, callback);
    // Cancel
    document.getElementById('node-cancelButton').onclick = cancelAction.bind(this, callback);
    document.getElementById('node-popUp').style.display = 'block';
  }

  // Callback passed as parameter is ignored
  function clearNodePopUp() {
    document.getElementById('node-saveButton').onclick = null;
    document.getElementById('node-cancelButton').onclick = null;
    document.getElementById('node-popUp').style.display = 'none';
  }

  function cancelNodeEdit(callback) {
    clearNodePopUp();
    callback(null);
  }

  // Save to the new node the info
  function saveNodeData(data, callback) {
    data.label = document.getElementById('node-label').value;
    data.zone = document.getElementById('node-zone').value;
    data.x = document.getElementById('node-x').value;
    data.y = document.getElementById('node-y').value;
    data.theta = document.getElementById('node-theta').value;
    data.frame = document.getElementById('node-frame').value;
    var request = new ROSLIB.ServiceRequest({
      node: {
        id: data.id,
        name: data.label,
        zone: parseInt(data.zone),
        pose: {
          x: parseFloat(data.x),
          y: parseFloat(data.y),
          theta: parseFloat(data.theta),
          frame_id: data.frame
        }
      }
    });
    self.addNodeService.callService(request, function (result) {
      console.log(result.message);
    });
    clearNodePopUp();
    callback(data);
  }

  // Edit the node the info
  function editNodeData(data, callback) {
    data.label = document.getElementById('node-label').value;
    data.zone = document.getElementById('node-zone').value;
    data.x = document.getElementById('node-x').value;
    data.y = document.getElementById('node-y').value;
    data.theta = document.getElementById('node-theta').value;
    data.frame = document.getElementById('node-frame').value;
    var request = new ROSLIB.ServiceRequest({
      node: {
        id: data.id,
        name: data.label,
        zone: parseInt(data.zone),
        pose: {
          x: parseFloat(data.x),
          y: parseFloat(data.y),
          theta: parseFloat(data.theta),
          frame_id: data.frame
        }
      }
    });
    self.setNodeService.callService(request, function (result) {
      console.log(result.message);
    });
    clearNodePopUp();
    callback(data);
  }

  function editEdgeWithoutDrag(data, callback) {
    // filling in the popup DOM elements
    document.getElementById('edge-label').value = data.label;
    document.getElementById('edge-saveButton').onclick = saveEdgeData.bind(this, data, callback);
    document.getElementById('edge-cancelButton').onclick = cancelEdgeEdit.bind(this, callback);
    document.getElementById('edge-popUp').style.display = 'block';
  }

  function clearEdgePopUp() {
    document.getElementById('edge-saveButton').onclick = null;
    document.getElementById('edge-cancelButton').onclick = null;
    document.getElementById('edge-popUp').style.display = 'none';
  }

  function cancelEdgeEdit(callback) {
    clearEdgePopUp();
    callback(null);
  }

  function saveEdgeData(data, callback) {
    if (typeof data.to === 'object')
      data.to = data.to.id
    if (typeof data.from === 'object')
      data.from = data.from.id
    data.label = document.getElementById('edge-label').value;
    clearEdgePopUp();
    callback(data);
  }

  this.network = new vis.Network(this.container, this.data, this.options);



  /*this.network.on("click", function (params) {
    params.event = "[original event]";
    document.getElementById('eventSpan').innerHTML = '<h2>Click event:</h2>' + JSON.stringify(params, null, 4);
    console.log('click event, getNodeAt returns: ' + this.getNodeAt(params.pointer.DOM));
  });
  this.network.on("doubleClick", function (params) {
      params.event = "[original event]";
      document.getElementById('eventSpan').innerHTML = '<h2>doubleClick event:</h2>' + JSON.stringify(params, null, 4);
  });
  this.network.on("oncontext", function (params) {
      params.event = "[original event]";
      document.getElementById('eventSpan').innerHTML = '<h2>oncontext (right click) event:</h2>' + JSON.stringify(params, null, 4);
  });
  this.network.on("dragStart", function (params) {
      // There's no point in displaying this event on screen, it gets immediately overwritten
      params.event = "[original event]";
      console.log('dragStart Event:', params);
      console.log('dragStart event, getNodeAt returns: ' + this.getNodeAt(params.pointer.DOM));
  });
  this.network.on("dragging", function (params) {
      params.event = "[original event]";
      document.getElementById('eventSpan').innerHTML = '<h2>dragging event:</h2>' + JSON.stringify(params, null, 4);
  });
  this.network.on("dragEnd", function (params) {
      params.event = "[original event]";
      document.getElementById('eventSpan').innerHTML = '<h2>dragEnd event:</h2>' + JSON.stringify(params, null, 4);
      console.log('dragEnd Event:', params);
      console.log('dragEnd event, getNodeAt returns: ' + this.getNodeAt(params.pointer.DOM));
  });
  this.network.on("zoom", function (params) {
      document.getElementById('eventSpan').innerHTML = '<h2>zoom event:</h2>' + JSON.stringify(params, null, 4);
  });
  this.network.on("showPopup", function (params) {
      document.getElementById('eventSpan').innerHTML = '<h2>showPopup event: </h2>' + JSON.stringify(params, null, 4);
  });
  this.network.on("hidePopup", function () {
      console.log('hidePopup Event');
  });
  this.network.on("select", function (params) {
      console.log('select Event:', params);
  });
  this.network.on("selectNode", function (params) {
      console.log('selectNode Event:', params);
  });
  this.network.on("selectEdge", function (params) {
      console.log('selectEdge Event:', params);
  });
  this.network.on("deselectNode", function (params) {
      console.log('deselectNode Event:', params);
  });
  this.network.on("deselectEdge", function (params) {
      console.log('deselectEdge Event:', params);
  });
  this.network.on("hoverNode", function (params) {
      console.log('hoverNode Event:', params);
  });
  this.network.on("hoverEdge", function (params) {
      console.log('hoverEdge Event:', params);
  });
  this.network.on("blurNode", function (params) {
      console.log('blurNode Event:', params);
  });
  this.network.on("blurEdge", function (params) {
      console.log('blurEdge Event:', params);
  });*/
};


// Definition of methods

NETWORKVIEW.NetworkView.prototype.addNode = function (node) {
  this.nodes.add({ id: node.id, label: node.name || "Node " + node.id })
};

NETWORKVIEW.NetworkView.prototype.addEdge = function (from_id, arc) {
  this.edges.add({ id: from_id + " " + arc.node_dest, from: from_id, to: arc.node_dest, arrows: 'to' })
};

NETWORKVIEW.NetworkView.prototype.deleteNetwork = function () {
  this.nodes.clear();
  this.edges.clear();
};

NETWORKVIEW.NetworkView.prototype.stabilize = function () {
  this.network.stabilize();
}

/**
 * Compare between network and graph message
 * 
 * @param callback - function with the following params:
 *     * message - the published message with the graph
 * @returns bool - true if the graph has changed
 */
NETWORKVIEW.NetworkView.prototype.hasChanged = function (message) {
  var changed = false;
  var node_edges;
  // If has different lengths
  if (message.nodes.length !== this.nodes.length) {
    changed = true;
  }
  // If any node is different
  message.nodes.forEach(node => {
    if (this.nodes.getDataSet().get(node.id) == null) {
      changed = true;
    } else if (node.name !== this.nodes.getDataSet().get(node.id).label) {
      changed = true;
    } else {
      // List of edges from this node
      node_edges = this.edges.getDataSet().get({
        filter: function (item) {
          return item.from == node.id;
        }
      })
      // If the node has different arc_list lengths
      if (node.arc_list.length !== node_edges.length) {
        changed = true
      } else {
        // If any arc is different
        node.arc_list.forEach(arc => {
          node_edges = this.edges.getDataSet().get({
            filter: function (item) {
              return item.from == node.id && item.to == arc.node_dest;
            }
          })
          if (node_edges.length === 0) {
            changed = true;
          }
        })
      }
    }
  });
  // Else, if the graph and the network are equal
  return changed;
}

NETWORKVIEW.NetworkView.prototype.resetAllNodes = function () {
  var nodes_aux = this.nodes.get();
  this.nodes.clear();
  var edges_aux = this.edges.get();
  this.edges.clear();
  this.nodes.add(nodes_aux);
  this.edges.add(edges_aux);
};

NETWORKVIEW.NetworkView.prototype.resetAllNodesStabilize = function () {
  this.resetAllNodes();
  this.stabilize();
};