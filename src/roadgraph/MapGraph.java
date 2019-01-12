/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
//import util.GraphLoader;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private HashMap<GeographicPoint, MapNode> vertices;
	private HashSet<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph, which contains a hashmap to store
	 * the nodes and a hashset to store the edges
	 */
	public MapGraph()
	{
		vertices = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return vertices.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(location != null && !vertices.containsKey(location)) {
			MapNode addedMapNode = new MapNode(location); 
		    this.vertices.put(location, addedMapNode);
			return true;
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		MapNode start = this.vertices.get(from);
		MapNode end = this.vertices.get(to);
		
		if(!this.vertices.containsKey(from) || !this.vertices.containsKey(to) ||
				roadName == null || roadType == null || length < 0) {
			throw new IllegalArgumentException();
		}
		MapEdge addedEdge = new MapEdge(start, end, roadName, roadType, length);
		edges.add(addedEdge);
		start.addEdge(addedEdge);
	}
	
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		//initialize the queue, the visited set and the parent hashap
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		Queue<MapNode> nodeQ = new LinkedList<MapNode>();
		Set<MapNode> visited = new HashSet<MapNode>();
		Map<MapNode,MapNode> parent = new HashMap<MapNode,MapNode>();
		
		//add the start node to all the collections
		nodeQ.add(startNode);
		visited.add(startNode);
		parent.put(startNode, null);
		
		//looking for the shortest path
		while(!nodeQ.isEmpty()) {
			MapNode curr = nodeQ.remove();
			//nodeSearched.accept(curr.getLocation());
			if (curr.equals(endNode)) { //the path is found
				List<GeographicPoint> path = reconstructPath(parent, startNode, endNode);
				return path;
			}
			for(MapNode neighbor : this.getNeighbors(curr)) {
				if(!visited.contains(neighbor)) {
					nodeQ.add(neighbor);
					visited.add(neighbor);
					parent.put(neighbor, curr);
				}
			}	
		}
		return null;
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		
		Comparator<Entry<MapNode, Double>> distanceComparator = new 
				Comparator<Entry<MapNode, Double>>() {
			@Override
			public int compare(Entry<MapNode, Double> entry1, Entry<MapNode, Double> entry2) {
				Double distance1 = entry1.getValue();
				Double distance2 = entry2.getValue();
				return distance1.compareTo(distance2);
			}
		};
		
		PriorityQueue<Entry<MapNode, Double>> nodeQ = new 
				PriorityQueue<>(this.getNumVertices(), distanceComparator);
		
		Set<MapNode> visited = new HashSet<MapNode>();
		Map<MapNode, Double> distances = new HashMap<MapNode, Double>();
		for(GeographicPoint node : this.getVertices()) {
			distances.put(this.vertices.get(node), Double.MAX_VALUE);
		}
		distances.put(startNode, 0.0);
		Map<MapNode,MapNode> parent = new HashMap<MapNode,MapNode>();
		nodeQ.add(new AbstractMap.SimpleEntry<MapNode, Double>(startNode, 0.0));
		
		while(!nodeQ.isEmpty()) {
			MapNode curr = nodeQ.remove().getKey();
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)) {
				visited.add(curr);
				if (curr.equals(endNode)) {
					List<GeographicPoint> path = reconstructPath(parent, startNode, endNode);
					System.out.println("There are" + visited.size() + "notes in visited");
					return path;
				}
				for(MapNode neighbor : this.getNeighbors(curr)) {
					double distanceFromStart = 0;
					if(!visited.contains(neighbor)) {
						distanceFromStart = distances.get(curr) + curr.getDistance(neighbor);
						double preDist = distances.get(neighbor);
						if(distanceFromStart < preDist) {
							distances.put(neighbor, distanceFromStart);
							parent.put(neighbor, curr);
							nodeQ.add(new AbstractMap.SimpleEntry<MapNode, Double>(
									neighbor, distanceFromStart));
						}
					}
				}	
			}					
		}		
		return null;
	}
	
	/** Generate the shortest path
	 * 
	 * @param parent The hashmap to store the parent node for each node visited
	 * @param start The start location
	 * @param goal The goal location
	 * @return the shortest path
	 */
	private List<GeographicPoint> reconstructPath(Map<MapNode,MapNode> parent,
			MapNode start, MapNode goal) {
			List<GeographicPoint> path = new ArrayList<GeographicPoint>();
			MapNode backtrackingNode = goal;
			while(!backtrackingNode.equals(start)) {
				path.add(backtrackingNode.getLocation());
				backtrackingNode = parent.get(backtrackingNode);
			}
			path.add(start.getLocation());
			Collections.reverse(path);
			return path;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		//nodeSearched.accept(next.getLocation());
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		
		Comparator<Entry<MapNode, Double>> distanceComparator = new 
				Comparator<Entry<MapNode, Double>>() {
			@Override
			public int compare(
					Entry<MapNode, Double> entry1, Entry<MapNode,Double> entry2) {
				Double distance1 = entry1.getValue();
				Double distance2 = entry2.getValue();
				return distance1.compareTo(distance2);
			}
		};
		
		PriorityQueue<Entry<MapNode, Double>> nodeQ = new 
				PriorityQueue<>(this.getNumVertices(), distanceComparator);
		
		Set<MapNode> visited = new HashSet<MapNode>();
		Map<MapNode, Distances> distances = new HashMap<MapNode, Distances>();
		for(GeographicPoint node : this.getVertices()) {
			distances.put(this.vertices.get(node), new Distances(Double.MAX_VALUE, Double.MAX_VALUE));
		}
		distances.put(startNode, new Distances(0.0, 0.0));
		Map<MapNode,MapNode> parent = new HashMap<MapNode,MapNode>();
		nodeQ.add(new AbstractMap.SimpleEntry<MapNode, Double>(startNode, new Distances(0.0, 0.0).getPredicted()));
		
		while(!nodeQ.isEmpty()) {
			MapNode curr = nodeQ.remove().getKey(); 
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)) {
				visited.add(curr);
				if (curr.equals(endNode)) {
					List<GeographicPoint> path = reconstructPath(parent, startNode, endNode);
					System.out.println("There are " + visited.size() + " notes in visited");
					for(GeographicPoint point : path) {
						System.out.println("aStar:" + point);
					}
					
					return path;
				}
				for(MapNode neighbor : this.getNeighbors(curr)) {
					double distanceFromStart = 0;
					if(!visited.contains(neighbor)) {
						distanceFromStart = distances.get(curr).distFromStart + curr.getDistance(neighbor);
						double prevDistFromStart = distances.get(neighbor).distFromStart;
						if(distanceFromStart < prevDistFromStart) {
							distances.put(neighbor, new Distances(distanceFromStart, 
									neighbor.getLocation().distance(goal)));
							parent.put(neighbor, curr);
							nodeQ.add(new AbstractMap.SimpleEntry<MapNode, Double>(
									neighbor, distances.get(neighbor).getPredicted()));
						}
					}
				}	
			}					
		}		
		return null;
	}
	
	public List<GeographicPoint> shortestPathForMultipleDists(GeographicPoint start, List<GeographicPoint> goals){
		List<GeographicPoint> shortestPath  = new LinkedList<GeographicPoint> ();
		GeographicPoint from = start;
		shortestPath.add(start);
//		goals.stream().forEach((g1) -> {
//			
//		});
		while(shortestPath.size() - 1 < goals.size()) {
			Double localMinDistance = Double.MAX_VALUE;
			GeographicPoint localGoal = null;
			for(int j = 0; j < goals.size(); j++ ) {
				GeographicPoint to = goals.get(j);
				if( from != to && !shortestPath.contains(goals.get(j))) {
					List<GeographicPoint> thisShortestPath = this.aStarSearch(from, to);
					//calculate the distance of this path
					double distance = this.getDistanceForPath(thisShortestPath);
					if(distance < localMinDistance) {
						localMinDistance = distance;
						localGoal = to;
					}
				}
			}	
			shortestPath.add(localGoal);
			from = localGoal;
		}
		shortestPath.add(start);
		return shortestPath;
	}
	
	private double getDistanceForPath(List<GeographicPoint> path) {
		double distance = 0;
		for(int i = 0; i < path.size() - 1;  i++) {
			distance += this.vertices.get(path.get(i)).getDistance(this.vertices.get(path.get(i+1)));
		}
		return distance;
	}

	class Distances{
		public double distFromStart;
		public double distToEnd;
		public double predictedDist;
		
		public Distances(double distFromStart, double distToEnd) {
			this.distFromStart = distFromStart;
			this.distToEnd = distToEnd;
		}
		
		public Double getPredicted() {
			return this.distFromStart + this.distToEnd;
		}
	}
	
	public static void main(String[] args) {
		MapGraph testShortestPath = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", testShortestPath);
		//***********************test1***************************
		GeographicPoint start1 = new GeographicPoint (1.0, 1.0);
		List<GeographicPoint> goals1 = new LinkedList<>();
		goals1.addAll(Arrays.asList(new GeographicPoint(8, -1), new GeographicPoint(7, 3),
				new GeographicPoint(4, 2), new GeographicPoint(4, 1), new GeographicPoint(4, 0), 
				new GeographicPoint(4, -1), new GeographicPoint(5, 1), new GeographicPoint(6.5, 0)));
		//***********************test2***************************
		GeographicPoint start2 = new GeographicPoint (4.0, -1.0);
		List<GeographicPoint> goals2 = new LinkedList<>();
		goals2.addAll(Arrays.asList(new GeographicPoint(6.5, 0)));
		//***********************test3***************************
		GeographicPoint start3 = new GeographicPoint (4.0, -1.0);
		List<GeographicPoint> goals3 = new LinkedList<>();
		
		List<GeographicPoint> shortestPath = testShortestPath.shortestPathForMultipleDists(start1, goals1);
		for(GeographicPoint point : shortestPath) {
			System.out.println(point);
		}
		
	}
}

