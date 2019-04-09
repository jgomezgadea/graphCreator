/**
 * @author Jose Gómez - jgomez@robotnik.es
 */

var GRAPHVIEW = GRAPHVIEW || {
  REVISION: '0.1.0'
};

GRAPHVIEW.GraphView = function (options) {
  options = options || {}
  this.divID = options.divID || 'graph';
  this.nodes = options.nodes || new vis.DataSet();
  this.edges = options.edges || new vis.DataSet();

  // Create a graph
  this.container = document.getElementById(this.divID);
  this.data = {
    nodes: this.nodes,
    edges: this.edges
  };
  this.options = {};
  this.graph = new vis.Network(this.container, this.data, this.options);
};

//GRAPHVIEW.GraphView.prototype.resetAllNodes = function () {
//  console.log("hola");
//  this.stabilize();
//};