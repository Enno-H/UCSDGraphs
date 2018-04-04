package roadgraph;

import geography.GeographicPoint;

public class MapEdge {

    private GeographicPoint start;
    private GeographicPoint end;
    private String name;
    private String type;
    private double distance;


    public MapEdge(GeographicPoint start, GeographicPoint end){
        this.start = start;
        this.end = end;
    }

    public MapEdge(GeographicPoint from, GeographicPoint to, String roadName,
                   String roadType, double length){
        this.start = from;
        this.end = to;
        this.name = roadName;
        this.type = roadType;
        this.distance = length;
    }

    //set
    public void setStart(GeographicPoint start){
        this.start = start;
    }

    public void setEnd(GeographicPoint end){
        this.end = end;
    }

    public void setName(String name){
        this.name = name;
    }

    public void setType(String type){
        this.type = type;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    //get
    public GeographicPoint getStart(){
        return this.start;
    }

    public GeographicPoint getEnd(){
        return this.end;
    }

    public String getName(){
        return this.name;
    }

    public String getType(){
        return this.type;
    }

    public double getDistance(){
        return this.distance;
    }

}
