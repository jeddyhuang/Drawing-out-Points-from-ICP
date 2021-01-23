/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package test;

/**
 *
 * @author rxiao
 */
public class Vertex implements Comparable<Vertex>{
    private int index;
    private double x;
    private double y;
    private double z;
    private String name;
    
    public Vertex(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }
    
    public Vertex(double x, double y, double z, int index){
        this.x = x;
        this.y = y;
        this.z = z;
        this.index = index;
    }
    
    public Vertex(double x, double y, double z, String name){
        this.x = x;
        this.y = y;
        this.z = z;
        this.name = name;
    }
    
    public int getindex(){
        return index;
    }
    
    public String getname(){
        return name;
    }
    
    public double getX(){
        return x;
    }
    
    public double getY(){
        return y;
    }
    
    public double getZ(){
        return z;
    }
    
    public void setindex(int index){
        this.index = index;
    }
    
    public double distance(Vertex e){
        return Math.sqrt(Math.pow((e.getX()-x), 2) + Math.pow(e.getY()-y, 2) + Math.pow((e.getZ()-z), 2));
    }
    
    @Override
    public int compareTo(Vertex e){
        return this.name.compareTo(e.getname());
    }
}
