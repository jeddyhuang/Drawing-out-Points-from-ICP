/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package test;

import Jama.EigenvalueDecomposition;
import Jama.Matrix;

import java.util.ArrayList;

/**
 *
 * @author jerrywang
 */
public class IterativeClosestPoint {
    private ArrayList<Vertex> original, target, applied;
    
    public IterativeClosestPoint(ArrayList<Vertex> newPoints, ArrayList<Vertex> targetpoints, boolean scaling){
        original = newPoints;
        target = getClosestPoints(targetpoints);
        Vertex omed = getExpectedValue(original), tmed = getExpectedValue(target);
        double[][] covarianceMatrix = getCovarianceMatrix(original, target, omed, tmed);
        double[][] quaternion = createQuaternion(covarianceMatrix);
        double[] eigenVector = getMaxEigenVector(quaternion);
        double[][] rotationMatrix = getRotationMatrix(eigenVector);
        double scalingFactor;
        if(scaling){
            scalingFactor = getScaling(tmed, rotationMatrix, omed);
        } else scalingFactor = 1;
        double[] translationalVector = getTranslationVector(tmed, rotationMatrix, scalingFactor, omed);
        applymatrix(rotationMatrix, scalingFactor, translationalVector);
    }
    
    private ArrayList<Vertex> getClosestPoints(ArrayList<Vertex> newPoints){
        ArrayList<Vertex> x = new ArrayList<Vertex>(original.size());
        for(int i = 0; i < original.size(); i ++){
            Vertex pose = original.get(i);
            Vertex closest = null;
            double dist = Double.POSITIVE_INFINITY;
            for(int j = 0; j < newPoints.size(); j++){
                Vertex potential = newPoints.get(j);
                double distance = Math.sqrt(Math.pow(pose.getX() - potential.getX(),2) + Math.pow(pose.getY() - potential.getY(),2) + Math.pow(pose.getZ() - potential.getZ(),2));
                if(distance < dist){
                    closest = potential;
                    dist = distance;
                }
            }
            x.add(closest);
        }
        return x;
    }

    private Vertex getExpectedValue(ArrayList<Vertex> u){
        double x = 0, y = 0, z = 0;
        
        for(int i =0; i < u.size(); i++){
            x += u.get(i).getX();
            y += u.get(i).getY();
            z += u.get(i).getZ();
        }
        x /= u.size();
        y /= u.size();
        z /= u.size();
        return new Vertex(x,y,z);
    }

    private double[][] getCovarianceMatrix(ArrayList<Vertex> p, ArrayList<Vertex> x, Vertex omed, Vertex tmed){       
        double[][] cov = new double[3][3];
        for(int i = 0; i < p.size(); i++){
            cov[0][0] += p.get(i).getX()*x.get(i).getX();
            cov[0][1] += p.get(i).getX()*x.get(i).getY();
            cov[0][2] += p.get(i).getX()*x.get(i).getZ();
            
            cov[1][0] += p.get(i).getY()*x.get(i).getX();
            cov[1][1] += p.get(i).getY()*x.get(i).getY();
            cov[1][2] += p.get(i).getY()*x.get(i).getZ();
            
            cov[2][0] += p.get(i).getZ()*x.get(i).getX();
            cov[2][1] += p.get(i).getZ()*x.get(i).getY();
            cov[2][2] += p.get(i).getZ()*x.get(i).getZ();
        }
        for(int i = 0; i < cov.length; i ++) for(int j = 0; j < cov[0].length; j ++){
            cov[i][j] /= p.size();
        }
        
        cov[0][0] -= omed.getX()*tmed.getX();
        cov[0][1] -= omed.getX()*tmed.getY();
        cov[0][2] -= omed.getX()*tmed.getZ();
        
	cov[1][0] -= omed.getY()*tmed.getX();
        cov[1][1] -= omed.getY()*tmed.getY();
        cov[1][2] -= omed.getY()*tmed.getZ();
        
	cov[2][0] -= omed.getZ()*tmed.getX();
        cov[2][1] -= omed.getZ()*tmed.getY();
        cov[2][2] -= omed.getZ()*tmed.getZ();
        return cov;
    }

    private double getTrace(double[][] e){
        double trace = 0;
        for(int i = 0; i < e.length; i++){
            trace+=e[i][i];
        }
        return trace;
    }

    private double[][] createQuaternion(double[][] e){
        double[][] q = new double[4][4];
        
        q[0][0] = getTrace(e);
        
        q[1][0] = e[1][2] - e[2][1];
        q[2][0] = e[2][0] - e[0][2];
        q[3][0] = e[0][1] - e[1][0];
        
        q[0][1] = q[1][0];
        q[0][2] = q[2][0];
        q[0][3] = q[3][0];
        
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                q[1+i][1+j] = e[i][j] + e[j][i] - (i==j?q[0][0]:0);
            }
        }
        return q;
    }

    private double[] getMaxEigenVector(double[][] q){        
        Matrix m = new Matrix(q);
        
        EigenvalueDecomposition evd = new EigenvalueDecomposition(m);
        
        double[] eigenValues = evd.getRealEigenvalues();
        double max = Double.NEGATIVE_INFINITY;
        int index = 0;
        
        for(int i =0; i < eigenValues.length; i++){
            if(eigenValues[i] > max){
                max = eigenValues[i];
                index = i;
            }
        }
        return evd.getV().transpose().getArray()[index];
    }

    private double[][] getRotationMatrix(double[] rotationVector){
        double[][] r = new double[3][3];
        double[] rv = rotationVector;
        
        r[0][0] = rv[0]*rv[0] + rv[1]*rv[1] - rv[2]*rv[2] - rv[3]*rv[3];
        r[1][1] = rv[0]*rv[0] + rv[2]*rv[2] - rv[1]*rv[1] - rv[3]*rv[3];
        r[2][2] = rv[0]*rv[0] + rv[3]*rv[3] - rv[1]*rv[1] - rv[2]*rv[2];
        
        r[0][1] = 2 * ( rv[1]*rv[2] - rv[0]*rv[3]);
        r[0][2] = 2 * ( rv[1]*rv[3] + rv[0]*rv[2]);
        
        r[1][0] = 2 * ( rv[1]*rv[2] + rv[0]*rv[3]);
        r[1][2] = 2 * ( rv[2]*rv[3] - rv[0]*rv[1]);
        
        r[2][0] = 2 * ( rv[1]*rv[3] - rv[0]*rv[2]);
        r[2][1] = 2 * ( rv[2]*rv[3] + rv[0]*rv[1]);
        
        return r;
    }
    
    private double getScaling(Vertex tmed, double[][] r, Vertex omed){
        double[] prod1a = new double[3];
        double prod1b, prod2;
        prod1a[0] = omed.getX() * r[0][0] + omed.getY() * r[0][1] + omed.getZ() * r[0][2];
        prod1a[1] = omed.getX() * r[1][0] + omed.getY() * r[1][1] + omed.getZ() * r[1][2];
        prod1a[2] = omed.getX() * r[2][0] + omed.getY() * r[2][1] + omed.getZ() * r[2][2];
        
        prod1b = prod1a[0] * tmed.getX() + prod1a[1] * tmed.getY() + prod1a[2] * tmed.getZ();
        prod2 = omed.getX() * omed.getX() + omed.getY() * omed.getY() + omed.getZ() * omed.getZ();
        return prod1b / prod2;
    }
    
    private double[] getTranslationVector(Vertex tmed, double[][] r, double scaling, Vertex omed){
        double[] t = new double[3];
        t[0] = tmed.getX() - scaling * (r[0][0]*omed.getX() + r[0][1]*omed.getY() + r[0][2]*omed.getZ());
        t[1] = tmed.getY() - scaling * (r[1][0]*omed.getX() + r[1][1]*omed.getY() + r[1][2]*omed.getZ());
        t[2] = tmed.getZ() - scaling * (r[2][0]*omed.getX() + r[2][1]*omed.getY() + r[2][2]*omed.getZ());
        return t;
    }
    
    private void applymatrix(double[][] rotation, double scaling, double[] translation){
        ArrayList<Vertex> newlist = new ArrayList<Vertex>();
        for(int i = 0; i < original.size(); i ++){
            double[] newpoint = new double[3];
            newpoint[0] = original.get(i).getX() * rotation[0][0] + original.get(i).getY() * rotation[0][1] + original.get(i).getZ() * rotation[0][2];
            newpoint[1] = original.get(i).getX() * rotation[1][0] + original.get(i).getY() * rotation[1][1] + original.get(i).getZ() * rotation[1][2];
            newpoint[2] = original.get(i).getX() * rotation[2][0] + original.get(i).getY() * rotation[2][1] + original.get(i).getZ() * rotation[2][2];
            
            newpoint[0] *= scaling;
            newpoint[1] *= scaling;
            newpoint[2] *= scaling;
            
            newpoint[0] += translation[0];
            newpoint[1] += translation[1];
            newpoint[2] += translation[2];
            newlist.add(new Vertex(newpoint[0], newpoint[1], newpoint[2], original.get(i).getindex()));
        }
        applied = newlist;
    }
    
    public ArrayList<Vertex> getNewPoints(){
        return applied;
    }
    
    public ArrayList<Vertex> getOriginalPoints(){
        return original;
    }
    
    public ArrayList<Vertex> getTargetPoints(){
        return target;
    }
}
