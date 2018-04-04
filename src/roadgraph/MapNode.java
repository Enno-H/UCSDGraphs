package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class MapNode {

    private GeographicPoint location;
    private List<MapEdge> edges;

    public MapNode(){
        this.edges = new ArrayList<MapEdge>();
    }

    public MapNode(GeographicPoint location){
        this.location = location;
        this.edges = new ArrayList<MapEdge>();
    }

    public void setEdges(List<MapEdge> edges){
        this.edges = edges;
    }

    public List<MapEdge> getEdges(){
        return this.edges;
    }

    public void addEdge(MapEdge edge){
        this.edges.add(edge);
    }

    public int getDegree(){
        return edges.size();
    }

    public GeographicPoint getGeograpicPoint(){
        return location;
    }

    public boolean equals(MapNode node){
        return this.location == node.location;
    }
}
