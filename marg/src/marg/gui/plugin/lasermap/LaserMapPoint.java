/*
 * LaseMapPoint.java
 * 
 * @author Jonas Eriksen (s082598)
 */

package marg.gui.plugin.lasermap;

public class LaserMapPoint {
    private double distance;
    private double angle;
    private boolean flag;
    private LaserMapPoint nextPoint;
    
    public LaserMapPoint(double angle, double distance, LaserMapPoint point){
        // Polar coordinates
        this.angle = angle; // radians
        this.distance = distance; // metre
        this.nextPoint = point;
    }
    
    public double getAngle(){
        return this.angle;
    }
    
    public double getDistance(){
        return this.distance;
    }
    
    public double getCartesianX(){
        return (distance * Math.cos(angle)); 
        
    }
    
    public double getCartesianY(){
        return (distance * Math.sin(angle));
    }
    
    // One way linked list structure
    public boolean hasNext(){
        return !(this.nextPoint == null);
    }
    
    public LaserMapPoint next(){
        return this.nextPoint;
    }
}
