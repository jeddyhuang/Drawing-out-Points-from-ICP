/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package test;

import java.io.PrintWriter;
import java.util.ArrayList;

/**
 *
 * @author rxiao
 */
public class ICPRunner {
    private ArrayList<Vertex> finalpts;
    
    public ICPRunner(String directory, int iterations, int checklimit, ArrayList<Vertex> original, ArrayList<Vertex> target, ArrayList<Vertex> set) throws Exception{
        if(iterations < 1) finalpts = this.runBaseCompare(target, set);
        else finalpts = this.runICP(directory, iterations, checklimit, original, target, set);
    }
    
    public ArrayList<Vertex> runBaseCompare(ArrayList<Vertex> original, ArrayList<Vertex> set){
        ArrayList<Vertex> newclosest = new ArrayList<Vertex>(set.size());
        for(int i = 0; i < set.size(); i ++){
            Vertex pose = set.get(i);
            Vertex closest = null;
            double dist = Double.POSITIVE_INFINITY;
            for(int j = 0; j < original.size(); j++){
                if(-1 < pose.getX() - original.get(j).getX() || pose.getX() - original.get(j).getX() < 1){
                    if(-1 < pose.getY() - original.get(j).getY() || pose.getY() - original.get(j).getY() < 1){
                        if(-1 < pose.getZ() - original.get(j).getZ() || pose.getZ() - original.get(j).getZ() < 1){
                            Vertex potential = original.get(j);
                            double distance = Math.sqrt(Math.pow(pose.getX() - potential.getX(),2) + Math.pow(pose.getY() - potential.getY(),2) + Math.pow(pose.getZ() - potential.getZ(),2));
                            if(distance < dist){
                                closest = potential;
                                dist = distance;
                            }
                        }
                    }
                }
            }
            newclosest.add(closest);
        }
        return newclosest;
    }
    
    public ArrayList<Vertex> runICP(String directory, int iterations, int checklimit, ArrayList<Vertex> original, ArrayList<Vertex> target, ArrayList<Vertex> set) throws Exception{
        ArrayList<Vertex> ctarget = target;
        ArrayList<Vertex> coriginal = original;
        ArrayList<Vertex> toriginal = this.firsttranslate(coriginal, ctarget, set);
        ArrayList<Vertex> noriginal = this.firstscale(toriginal, ctarget);
        
        ArrayList<Vertex> newclosest = new ArrayList<Vertex>(set.size());
        ArrayList<Vertex> newPoints = null;
        ArrayList<ArrayList<Integer>> checker = new ArrayList<ArrayList<Integer>>();
        for(int i = 0; i < set.size(); i ++){
            checker.add(new ArrayList<Integer>());
        }
        
        for(int times = 1; times <= iterations; times ++){
            IterativeClosestPoint runner;
            if(times == 1) runner = new IterativeClosestPoint(noriginal, ctarget, false);
            else if(times <= 50) runner = new IterativeClosestPoint(newPoints, ctarget, false);
            else runner = new IterativeClosestPoint(newPoints, ctarget, false);
            newPoints = runner.getNewPoints();
            newclosest.clear();
            
            for(int i = 0; i < set.size(); i ++){
                Vertex pose = set.get(i);
                Vertex closest = null;
                double dist = Double.POSITIVE_INFINITY;
                for(int j = 0; j < newPoints.size(); j++){
                    if(-1 < pose.getX() - newPoints.get(j).getX() || pose.getX() - newPoints.get(j).getX() < 1){
                        if(-1 < pose.getY() - newPoints.get(j).getY() || pose.getY() - newPoints.get(j).getY() < 1){
                            if(-1 < pose.getZ() - newPoints.get(j).getZ() || pose.getZ() - newPoints.get(j).getZ() < 1){
                                Vertex potential = newPoints.get(j);
                                double distance = Math.sqrt(Math.pow(pose.getX() - potential.getX(),2) + Math.pow(pose.getY() - potential.getY(),2) + Math.pow(pose.getZ() - potential.getZ(),2));
                                if(distance < dist){
                                    closest = potential;
                                    dist = distance;
                                }
                            }
                        }
                    }
                }
                newclosest.add(closest);
                if(times <= checklimit){
                    checker.get(i).add(closest.getindex());
                } else{
                    checker.get(i).remove(0);
                    checker.get(i).add(closest.getindex());
                }
            }
            if(this.checkArray(checklimit, checker)){
                String ICP_dir = directory + " After ICP.obj";
                PrintWriter writer = new PrintWriter(ICP_dir, "UTF-8");
                for(int i = 0; i < newPoints.size(); i ++){
                    writer.print("# ");
                    writer.println(newPoints.get(i).getindex());
                    writer.print("v ");
                    writer.print(newPoints.get(i).getX());
                    writer.print(" ");
                    writer.print(newPoints.get(i).getY());
                    writer.print(" ");
                    writer.println(newPoints.get(i).getZ());
                }
                writer.close();
                return newclosest;
            }
        }
        String ICP_dir = directory + " After ICP.obj";
        PrintWriter writer = new PrintWriter(ICP_dir, "UTF-8");
        for(int i = 0; i < newPoints.size(); i ++){
            writer.print("# ");
            writer.println(newPoints.get(i).getindex());
            writer.print("v ");
            writer.print(newPoints.get(i).getX());
            writer.print(" ");
            writer.print(newPoints.get(i).getY());
            writer.print(" ");
            writer.println(newPoints.get(i).getZ());
        }
        writer.close();
        return newclosest;
    }
    
    public ArrayList<Vertex> firsttranslate(ArrayList<Vertex> original, ArrayList<Vertex> target, ArrayList<Vertex> set){
        ArrayList<Vertex> editor = this.runBaseCompare(original, set);
        Vertex tcenter = this.getBary(set);
        Vertex ocenter = this.getBary(editor);
        double x = tcenter.getX() - ocenter.getX();
        double y = tcenter.getY() - ocenter.getY();
        double z = tcenter.getZ() - ocenter.getZ();
        
        ArrayList<Vertex> corrected = new ArrayList<Vertex>(original.size());
        for(int i = 0; i < original.size(); i ++){
            corrected.add(new Vertex(original.get(i).getX() + x, original.get(i).getY() + y, original.get(i).getZ() + z, original.get(i).getindex()));
        }
        return corrected;
    }
    
    public Vertex getBary(ArrayList<Vertex> vertices){
        double x = 0, y = 0, z = 0;
        for(int i = 0; i < vertices.size(); i ++){
            x += vertices.get(i).getX();
            y += vertices.get(i).getY();
            z += vertices.get(i).getZ();
        }
        x /= vertices.size();
        y /= vertices.size();
        z /= vertices.size();
        return new Vertex(x, y, z);
    }
    
    public ArrayList<Vertex> firstscale(ArrayList<Vertex> original, ArrayList<Vertex> target){
        Vertex tmed = this.getBary(target);
        Vertex omed = this.getBary(original);
        double prod = omed.getX() * tmed.getX() + omed.getY() * tmed.getY() + omed.getZ() * tmed.getZ();
        double prod1 = omed.getX() * omed.getX() + omed.getY() * omed.getY() + omed.getZ() * omed.getZ();
        double scaling =  prod / prod1;
        ArrayList<Vertex> corrected = new ArrayList<Vertex>(original.size());
        for(int i = 0; i < original.size(); i ++){
            corrected.add(new Vertex(original.get(i).getX() * scaling, original.get(i).getY() * scaling, original.get(i).getZ() * scaling, original.get(i).getindex()));
        }
        return corrected;
    }
    
    public boolean checkArray(int checklimit, ArrayList<ArrayList<Integer>> array){
        if(array.get(0).size() != checklimit) return false;
        for(int i = 0; i < array.size(); i ++){
            int temp = array.get(i).get(0);
            for(int j = 1; j < array.get(i).size(); j ++){
                if(temp != array.get(i).get(j)) return false;
            }
        }
        return true;
    }
    
    public ArrayList<Vertex> getresults(){
        return finalpts;
    }
}
