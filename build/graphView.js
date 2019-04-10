/**
 * @author Jose Gómez - jgomez@robotnik.es
 */

var GRAPHVIEW = GRAPHVIEW || {
  REVISION: '0.1.0'
};

GRAPHVIEW.GraphView = function (options) {
  options = options || {}
  this.divID = options.divID || 'graph';

  // If we use ROS, 
  if (options.ros_url !== undefined) {

    // TODO llenar options.nodes/edges por ROS
    this.rosTopic = new ROSLIB.Topic({
      ros: options.ros_url || 'ws://localhost:9090',
      name: options.topic || '/robotnik_fms_routes_node/graph_marker_array',
      messageType: options.messageType || 'visualization_msgs/MarkerArray'
    })
  }

  this.nodes = options.nodes || new vis.DataSet();
  this.edges = options.edges || new vis.DataSet();
  this.nextID = 1 + this.nodes.length;

  // Create a graph
  this.container = document.getElementById(this.divID);
  this.data = {
    nodes: this.nodes,
    edges: this.edges
  };
  this.options = {};
  this.graph = new vis.Network(this.container, this.data, this.options);
};

GRAPHVIEW.GraphView.prototype.addNode = function (name) {
  this.nodes.add({ id: this.nextID, label: name || "Node " + this.nextID })
  this.nextID++;
};

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
  this.graph.stabilize();
};

GRAPHVIEW.GraphView.prototype.stabilize = function () {
  this.graph.stabilize();
};