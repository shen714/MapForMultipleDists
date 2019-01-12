package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

/**
 * @author Me
 * 
 * A class which represents a intersection in the graph
 *
 */
public class MapNode {
	private GeographicPoint nodeLocation;
	private HashSet<MapEdge> edges;
	
	/** 
	 * Create a new node
	 */
	public MapNode(GeographicPoint location) {
		nodeLocation = location;
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of edges connected to this node
	 * @return The number of edges.
	 */
	public int getNumEdges() {
		return this.edges.size();
	}
	
	/**
	 * Add a edge to the node
	 */
	public void addEdge(MapEdge edge) {
		this.edges.add(edge);
	}
	
	/**
	 * Get all the neighbors that are directly connect to this node
	 * @return a set of node that are neighbors to this node
	 */
	public Set<MapNode> getNeighbors() {
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for(MapEdge edges : this.edges) {
			neighbors.add(edges.getOtherNode(this));
		}
		return neighbors;
	}
	
	/** Get the node's location
	 * 
	 * @return the location as GeographicPoint
	 */
	public GeographicPoint getLocation(){
		return nodeLocation;
	}
	
	/** Get all the edges connected to this nodes
	 * 
	 * @return a set of edges
	 */
	public Set<MapEdge> getEdges(){
		return edges;
	}
	
	/** Get the distance given the other end of the edge
	 * 
	 * @return the distance
	 */
	public double getDistance(MapNode theOtherNode) {
		for(MapEdge edge : getEdges()) {
			if(edge.getOtherNode(this) == theOtherNode) {
				return edge.getDistance();
			}
		}
		return 0.0;
	}
	
	@Override
	public boolean equals(Object o){
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.nodeLocation.equals(this.nodeLocation);
	}

	@Override
	public String toString(){
		String toReturn = "[NODE at location (" + nodeLocation + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}
}
