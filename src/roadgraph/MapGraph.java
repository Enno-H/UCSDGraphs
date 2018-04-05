/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
    //TODO: Add your member variables here in WEEK 3
    private HashMap<GeographicPoint,MapNode> hashMap ;
    private HashSet<MapEdge> edges;
    private Map<MapNode, ArrayList<MapNode>> adjListsMap;
    private HashMap<MapNode,Double> distance;

    int diNum = 0;
    int aNum = 0;


    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        // TODO: Implement in this constructor in WEEK 3
        this.hashMap = new HashMap<>();
        this.edges = new HashSet<>();
        this.adjListsMap = new HashMap<>();
        this.distance = new HashMap<>();
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        //TODO: Implement this method in WEEK 3
        return hashMap.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        //TODO: Implement this method in WEEK 3
        return hashMap.keySet();
    }

    /**
     * Get the number of road segments in the graph
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        //TODO: Implement this method in WEEK 3
        int NumEdges = 0;
        for(MapNode node : hashMap.values()){
            NumEdges = NumEdges + node.getDegree();
        }
        return NumEdges;
    }


    /** Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     * @param location  The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        // TODO: Implement this method in WEEK 3
        if(hashMap.keySet().contains(location) || location == null){
            return false;
        }
        MapNode newNode = new MapNode(location);
        hashMap.put(location,newNode);
        adjListsMap.put(newNode, new ArrayList<>());
        return true;
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

        //TODO: Implement this method in WEEK 3
        if(hashMap.keySet().contains(from) && hashMap.keySet().contains(to) && roadName != null
                && roadType != null && length >= 0){
            MapEdge newEdge = new MapEdge(from,to,roadName,roadType,length);
            hashMap.get(from).addEdge(newEdge);
            adjListsMap.get(hashMap.get(from)).add(hashMap.get(to));
            edges.add(newEdge);
        }
        else{
            throw new IllegalArgumentException("Illegal arguments");
        }
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
        Consumer<GeographicPoint> temp = (x) -> {
        };
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
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        MapNode startNode = hashMap.get(start);
        MapNode goalNode = hashMap.get(goal);
        HashSet visited = new HashSet();
        Queue<MapNode> queue = new LinkedList<MapNode>();
        MapNode curr = startNode;
        HashMap<MapNode,MapNode> parent = new HashMap<>();
        List<GeographicPoint> route = new LinkedList<>();

        queue.add(startNode);
        visited.add(startNode);

        while(!queue.isEmpty()){
            curr = queue.poll();
            nodeSearched.accept(curr.getGeograpicPoint());
            if(curr == goalNode){
                while(parent.get(curr)!= null){
                    ((LinkedList<GeographicPoint>) route).addFirst(curr.getGeograpicPoint());
                    curr = parent.get(curr);
                }
                ((LinkedList<GeographicPoint>) route).addFirst(curr.getGeograpicPoint());
                return route;
            }
            for(MapNode node : adjListsMap.get(curr)){
                if(!visited.contains(node)){
                    visited.add(node);
                    parent.put(node,curr);
                    queue.add(node);
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
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
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
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 4

        MapNode startNode = hashMap.get(start);
        MapNode goalNode = hashMap.get(goal);
        HashSet<MapNode> visited = new HashSet<>();
        PriorityQueue<MapNode> queue = new PriorityQueue<>(getNumVertices(),dijkstraOrder);
        MapNode curr = startNode;
        HashMap<MapNode,MapNode> parent = new HashMap<>();
        List<GeographicPoint> route = new LinkedList<>();

        //initialize the distances of all nodes
        for(MapNode node : hashMap.values()){
            distance.put(node,10000000.0);
        }

        queue.add(startNode);
        distance.put(startNode,0.0);

        while(!queue.isEmpty()){
            curr = queue.poll();
            diNum++;
            System.out.println(diNum+": dijkstra visiting:"+curr.getGeograpicPoint().toString());
            nodeSearched.accept(curr.getGeograpicPoint());
            if(!visited.contains(curr)){
                visited.add(curr);
                if(curr == goalNode){
                    while(parent.get(curr)!= null){
                        ((LinkedList<GeographicPoint>) route).addFirst(curr.getGeograpicPoint());
                        curr = parent.get(curr);
                    }
                    ((LinkedList<GeographicPoint>) route).addFirst(curr.getGeograpicPoint());
                    return route;
                }
                for(MapNode node : adjListsMap.get(curr)){
                    if(!visited.contains(node)){
                        double addedDis = curr.getEdgeTo(node).getDistance() + distance.get(curr);
                        if(addedDis < distance.get(node)){
                            distance.replace(node,addedDis);
                            parent.put(node,curr);
                            queue.add(node);
                        }
                    }
                }
            }






        }

        return null;
    }

    Comparator<MapNode> dijkstraOrder = new Comparator<MapNode>() {
        @Override
        public int compare(MapNode o1, MapNode o2) {
            if(distance.get(o1) < distance.get(o2)){
                return -1;
            }
            else if(distance.get(o1) > distance.get(o2)){
                return 1;
            }
            else{
                return 0;
            }
        }
    };




    /** Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal The goal location
     * @return The list of intersections that form the shortest path from
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
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
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 4


        Comparator<MapNode> AstarOrder = new Comparator<MapNode>() {
            @Override
            public int compare(MapNode o1, MapNode o2) {
                double f1 = distance.get(o1) + o1.getGeograpicPoint().distance(hashMap.get(goal).getGeograpicPoint());
                double f2 = distance.get(o2) + o2.getGeograpicPoint().distance(hashMap.get(goal).getGeograpicPoint());
                if(f1 < f2){
                    return -1;
                }
                else if(f1 > f2){
                    return 1;
                }
                else{
                    return 0;
                }
            }
        };

        MapNode startNode = hashMap.get(start);
        MapNode goalNode = hashMap.get(goal);
        HashSet<MapNode> visited = new HashSet<>();
        PriorityQueue<MapNode> queue = new PriorityQueue<>(getNumVertices(),AstarOrder);
        MapNode curr = startNode;
        HashMap<MapNode,MapNode> parent = new HashMap<>();
        List<GeographicPoint> route = new LinkedList<>();

        //initialize the distances of all nodes
        for(MapNode node : hashMap.values()){
            distance.put(node,10000000.0);
        }

        queue.add(startNode);
        distance.put(startNode,0.0);

        while(!queue.isEmpty()){
            curr = queue.poll();
            aNum++;
            System.out.println(aNum+": aStar visiting:"+curr.getGeograpicPoint().toString());
            nodeSearched.accept(curr.getGeograpicPoint());
            if(!visited.contains(curr)){
                visited.add(curr);
                if(curr == goalNode){
                    while(parent.get(curr)!= null){
                        ((LinkedList<GeographicPoint>) route).addFirst(curr.getGeograpicPoint());
                        curr = parent.get(curr);
                    }
                    ((LinkedList<GeographicPoint>) route).addFirst(curr.getGeograpicPoint());
                    return route;
                }
                for(MapNode node : adjListsMap.get(curr)){
                    if(!visited.contains(node)){
                        double addedDis = curr.getEdgeTo(node).getDistance() + distance.get(curr);
                        if(addedDis < distance.get(node)){
                            distance.replace(node,addedDis);
                            parent.put(node,curr);
                            queue.add(node);
                        }
                    }
                }
            }
        }
        return null;
    }






    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph firstMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
        System.out.println("DONE.");

        // You can use this method for testing.


        /* Here are some test cases you should try before you attempt
         * the Week 3 End of Week Quiz, EVEN IF you score 100% on the
         * programming assignment.
         */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);



        /* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);



    }

}
