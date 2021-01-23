/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package test;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;

/**
 *
 * @author rxiao
 */
public class Test {
    private static int iterations, icplimit;

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) throws Exception {
        iterations = 100; icplimit = 4;
        ArrayList<String> people = new ArrayList<String>();
        people.add("84471");
        people.add("123592");
        people.add("204542");
        people.add("304494");
        people.add("305426");
        people.add("307094");
        people.add("307275");
        people.add("307873");
        people.add("308059");
        people.add("308256");
        people.add("309576");
        people.add("310162");
        people.add("312797");
        people.add("314601");
        people.add("317157");
        people.add("319043");
        people.add("319611");
        people.add("320182");
        people.add("320411");
        people.add("321233");
        people.add("321294");
        people.add("323478");
        people.add("323691");
        people.add("324536");
        people.add("324728");
        people.add("325276");
        people.add("325321");
        people.add("454385");
        people.add("546051");
        people.add("588944");
        people.add("599995");
        people.add("609510");
        people.add("762372");
        people.add("767880");
        people.add("812207");
        //people.add("26792 D");
        //people.add("192641 B");
        //people.add("316137 flag");
        //people.add("460042 E");
        
        String username = System.getProperty("user.name");
        String tempdirectory = "C:\\Users\\" + username + "\\Desktop\\ManualLandmark-JW_1\\460042 E";
        String tempobjdir = tempdirectory + "\\460042 Cropped.obj";
        String tempselptdir = tempdirectory + "\\460042 SEL.obj";
        ObjReader baseobj = new ObjReader(tempobjdir);
        ObjReader baseselpt = new ObjReader(tempselptdir);
        baseobj.setindices();
        baseselpt.alphabetizeVertices();
        
        for(int index = 0; index < people.size(); index ++){
            String name = people.get(index);
            String directory = "C:\\Users\\" + username + "\\Desktop\\ManualLandmark-JW_1\\"  + name + "\\" + name;
            String objdir = directory + " Cropped.obj";
            ObjReader compareobj = new ObjReader(objdir);
            compareobj.setindices();
            
            ICPRunner runner = new ICPRunner(directory, iterations, icplimit, compareobj.compileVertices(), baseobj.compileVertices(), baseselpt.compileVertices());
            ArrayList<Vertex> results = runner.getresults();
            
            String ICP_dir = directory + " ICP.obj";
            PrintWriter writer = new PrintWriter(ICP_dir, "UTF-8");
            for(int i = 0; i < results.size(); i ++){
                writer.print("# ");
                writer.println(baseselpt.compileVertices().get(i).getname());
                writer.print("v ");
                writer.print(compareobj.find(results.get(i).getindex()).getX());
                writer.print(" ");
                writer.print(compareobj.find(results.get(i).getindex()).getY());
                writer.print(" ");
                writer.println(compareobj.find(results.get(i).getindex()).getZ());
            }
            writer.close();
            System.out.println(name + " Complete");
        }
    }
}
