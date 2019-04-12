/**
 * @author Jose Gómez - jgomez@robotnik.es
 */

var GRAPHVIEW = GRAPHVIEW || {
  REVISION: '0.1.0'
};

GRAPHVIEW.GraphView = function (options) {
  options = options || {}
  this.divID = options.divID || 'graph';

  if (options.ros !== undefined) { // If we use ROS

    this.message = undefined;
    this.nodes = new vis.DataSet();
    this.edges = new vis.DataSet();

    this.rosNodes = new ROSLIB.Topic({
      ros: options.ros,
      name: options.rosNodesTopic || '/robotnik_fms_routes_node/graph',
      messageType: options.rosNodesMsg || 'robotnik_fms_msgs/NodesInfo'
    })

    var self = this;
    this.rosNodes.subscribe(function (message) {

      if (self.hasChanged(message)) {

        // Delete actual graph
        self.deleteGraph();

        // Fill graph with received nodes
        message.Nodes.forEach(node => {
          self.addNode(node.name);
        });

        self.stabilize();
        //self.rosNodes.unsubscribe();
      }

    });

  } else { // Get data if is not getted from ros

    this.nodes = options.nodes || new vis.DataSet();
    this.edges = options.edges || new vis.DataSet();

  }

  this.nextID = 0 + this.nodes.length;

  // Create the graph
  this.container = document.getElementById(this.divID);
  this.data = {
    nodes: this.nodes,
    edges: this.edges
  };
  this.options = {};
  this.graph = new vis.Network(this.container, this.data, this.options);
};


// Definition of methods

GRAPHVIEW.GraphView.prototype.addNode = function (name) {
  this.nodes.add({ id: this.nextID, label: name || "Node " + this.nextID })
  this.nextID++;
};

GRAPHVIEW.GraphView.prototype.deleteGraph = function () {
  this.nodes.clear();
  this.edges.clear();
  this.nextID = 0;
};

GRAPHVIEW.GraphView.prototype.stabilize = function () {
  this.graph.stabilize();
}

/**
 * Compare between graph and message
 * 
 * @param callback - function with the following params:
 *   * message - the published message with the graph
 * @returns bool - true if the graph has changed
 */
GRAPHVIEW.GraphView.prototype.hasChanged = function (message) {
  var changed = false;
  // If has different lengths
  if (message.Nodes.length !== this.nodes.length) {
    changed = true;
  }
  // If any node is different
  message.Nodes.forEach(node => {

    if (this.nodes.getDataSet().get(node.id) == null) {
      changed = true;
    } else if (node.name !== this.nodes.getDataSet().get(node.id).label) {
      changed = true;
    }
  });
  // Else, if the two graphs are equal
  return changed;
}

GRAPHVIEW.GraphView.prototype.resetAllNodes = function () {
  var nodes_aux = this.nodes.get();
  this.nodes.clear();
  var edges_aux = this.edges.get();
  this.edges.clear();
  this.nodes.add(nodes_aux);
  this.edges.add(edges_aux);
};

GRAPHVIEW.GraphView.prototype.resetAllNodesStabilize = function () {
  this.resetAllNodes();
  this.stabilize();
};

GRAPHVIEW.GraphView.prototype.stabilize = function () {
  this.graph.stabilize();
};