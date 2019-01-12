package roadgraph;

import geography.GeographicPoint;

/**
 * @author Me
 * 
 * A class which represents a road segment
 *
 */
public class MapEdge {
	private MapNode start;
	private MapNode end;
	private String roadType;
	private String roadName;
	private double distance;
	
	/** 
	 * Create a new edge
	 */
	public MapEdge(MapNode start, MapNode end, String roadName, String roadType, 
			double distance) {
		this.start = start;
		this.end = end;
		this.roadType = roadType;
		this.roadName = roadName;
		this.distance = distance;
	}
	
	public MapNode getEndNode() {
		return this.end;
	}
	
	public GeographicPoint getStartPoint(){
		return start.getLocation();
	}
	
	public GeographicPoint getEndPoint(){
		return end.getLocation();
	}
	
	public double getDistance(){
		return distance;
	}
	
	public String getRoadName(){
		return roadName;
	}

	/** Given a node, get the other node connected by the edge 
	 * 
	 * @param node the given node
	 * @return the other node
	 */
	public MapNode getOtherNode(MapNode node){
		if (node.equals(start)) 
			return end;
		else if (node.equals(end))
			return start;
		throw new IllegalArgumentException("Looking for " +
			"a point that is not in the edge");
	}
	
	@Override
	public String toString(){
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + start.getLocation();
		toReturn += "\n\t" + end.getLocation();
		toReturn += "\nRoad name: " + roadName + " Road type: " + roadType +
				" Segment length: " + String.format("%.3g", distance) + "km";
		
		return toReturn;
	}

}
