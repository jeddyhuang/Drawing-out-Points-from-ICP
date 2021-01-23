/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package test;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.StringTokenizer;

/**
 *
 * @author lenovo
 */
public class ObjReader {
    private String directory;
    private ArrayList<Character> content;
    private ArrayList<Integer> linebreak;
    
    //private ArrayList<String> mtllibs;
    private ArrayList<Vertex> vertices;
    //private ArrayList<Texture> textures;
    //private ArrayList<Vector> vectors;
    //private ArrayList<String> materials;
    //private ArrayList<ArrayList<Facet>> facets;
    
    public ObjReader(String directory) throws FileNotFoundException, Exception{
        this.directory = directory;
        FileReader File = new FileReader(directory);
        content = new ArrayList<Character>();
        int marker;
        while((marker = File.read())!= -1) content.add((char)marker);
        File.close();
        linebreak = new ArrayList<Integer>();
        linebreak.add(0);
        for(int i = 0; i < content.size(); i ++) if(content.get(i).equals('\n') && (i+1) < content.size()) linebreak.add(i+1);
        
        //mtllibs = new ArrayList<String>();
        vertices = new ArrayList<Vertex>();
        //textures = new ArrayList<Texture>();
        //vectors = new ArrayList<Vector>();
        //materials = new ArrayList<String>();
        //facets = new ArrayList<ArrayList<Facet>>();
        
        int vindex = 1;
        int vtindex = 1;
        int vnindex = 1;
        int findex = 1;
        
        boolean temp = false;
        String name = "";
        
        for(int i = 0; i < linebreak.size(); i ++){
            String line = "";
            if(i != linebreak.size()-1) for(int j = linebreak.get(i); j < linebreak.get(i+1); j ++) line += content.get(j);
            else for(int j = linebreak.get(i); j < content.size(); j ++) line += content.get(j);
            StringTokenizer tokens = new StringTokenizer(line);
            
            switch(tokens.nextToken()){
                case "#":{
                    name = tokens.nextToken();
                    temp = true;
                    break;
                }
                /*case "mtllib":{
                    mtllibs.add(tokens.nextToken());
                    break;
                }*/
                case "v":{
                    if(temp){
                        vertices.add(new Vertex(Double.parseDouble(tokens.nextToken()), Double.parseDouble(tokens.nextToken()), Double.parseDouble(tokens.nextToken()), name));
                        temp = false;
                    }
                    else vertices.add(new Vertex(Double.parseDouble(tokens.nextToken()), Double.parseDouble(tokens.nextToken()), Double.parseDouble(tokens.nextToken()), vindex));
                    vindex ++;
                    break;
                }
                /*case "vt":{
                    textures.add(new Texture(vtindex, Double.parseDouble(tokens.nextToken()), Double.parseDouble(tokens.nextToken())));
                    vtindex ++;
                    break;
                }*/
                /*case "vn":{
                    vectors.add(new Vector(vnindex, Double.parseDouble(tokens.nextToken()), Double.parseDouble(tokens.nextToken())));
                    vnindex ++;
                    break;
                }*/
                /*case "usemtl":{
                    materials.add(tokens.nextToken());
                    facets.add(new ArrayList<Facet>());
                    break;
                }*/
                /*case "f":{
                    facets.get(facets.size()-1).add(new Facet(findex, tokens.nextToken(), tokens.nextToken(), tokens.nextToken()));
                    findex ++;
                    break;
                }*/
            }
        }
    }
    
    public void alphabetizeVertices(){
        Collections.sort(vertices);
    }
    
    public void setindices(){
        for(int i = 0; i < vertices.size(); i ++) vertices.get(i).setindex(Integer.parseInt(vertices.get(i).getname()));
    }
    
    public Vertex find(int m){
        for(int i = 0; i < vertices.size(); i ++) if(vertices.get(i).getindex() == m) return vertices.get(i);
        return null;
    }
    
    public ArrayList<Character> compileContent(){
        return content;
    }
    
    /*public ArrayList<String> compileLibraries(){
        return mtllibs;
    }*/
    
    public ArrayList<Vertex> compileVertices(){
        return vertices;
    }
    
    /*public ArrayList<Texture> compileTextures(){
        return textures;
    }*/
    
    /*public ArrayList<Vector> compileVectors(){
        return vectors;
    }*/
    
    /*public ArrayList<String> compileMaterials(){
        return materials;
    }*/
    
    /*public ArrayList<ArrayList<Facet>> compileFacets(){
        return facets;
    }*/
}