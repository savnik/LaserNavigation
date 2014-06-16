/*
 * LaseMapData.java
 * 
 * @author Jonas Eriksen (s082598)
 */

package marg.gui.plugin.lasermap;

import java.lang.Math;
import marg.model.LaserXMLTag;
import marg.gui.plugin.lasermap.LaserMapPoint;

public class LaserMapData {
    private double unit = 0.001;
    private int sensorNumber = 1;
    private double sensorRange = 5.0;
    private double sensorMinAngle = 0;
    private double sensorMaxAngle = 0;
    private double sensorX = 0; 
    private double sensorY = 0;
    private double sensorZ = 0;
    private double robotX = 0;
    private double robotY = 0;
    private double robotH = 0;
    private LaserMapPoint firstPoint = null;
    private boolean isReady = false;
    private String codec = "HEX";
    
    
    public boolean isReady(){
        return this.isReady;
    }
    
    public void parseXML(LaserXMLTag t){
        isReady = false;
        
        if(t.tagNameEquals("scanget")){
            
            if(t.getAttributeValue("min") != null && t.getAttributeValue("max") != null){
                // rotate given angles with 90 degrees in order for the center of the scanner to allign with the positive y-axis
                sensorMinAngle = Double.parseDouble(t.getAttributeValue("min"))+90; // in degrees
                sensorMaxAngle = Double.parseDouble(t.getAttributeValue("max"))+90; // in degrees
            }
            
            if(t.getAttributeValue("codec") != null){
                codec = t.getAttributeValue("codec");
            }
            
            if(t.getAttributeValue("unit") != null){
                
                if(t.getAttributeValue("unit").equals("mm")){
                    unit = 0.001;
                }else if(t.getAttributeValue("unit").equals("10cm") || t.getAttributeValue("unit").equals("dm")){
                    unit = 0.1;
                }else if(t.getAttributeValue("unit").equals("cm")){
                    unit = 0.01;
                }
            }
        }else if(t.tagNameEquals("pos3d")){
            
            // Rotate given coordinates 90 degrees (y=x, x=-y, z=-1)
            if(t.getAttributeValue("x") != null){
                sensorY = Double.parseDouble(t.getAttributeValue("x"));
            }else if(t.getAttributeValue("y") != null){
                sensorX = -Double.parseDouble(t.getAttributeValue("y"));
            }else if(t.getAttributeValue("z") != null){
                sensorZ = -Double.parseDouble(t.getAttributeValue("z"));
            }
            
        }else if(t.tagNameEquals("pose")){
            
            if(t.getAttributeValue("x") != null){
                robotY = Double.parseDouble(t.getAttributeValue("x"));
            }
            if(t.getAttributeValue("y") != null){
                robotX = -Double.parseDouble(t.getAttributeValue("y"));
            }
            if(t.getAttributeValue("h") != null){
                robotH = Double.parseDouble(t.getAttributeValue("h"));
            }
            
        }else if(t.tagNameEquals("bin") && t.hasContents()){
            
            parseHEX(t.getContents());
            isReady = true;
            
        }else if(t.tagNameEquals("var") && t.getAttributeValue("name") != null && t.getAttributeValue("value") != null){
            
            if(t.getAttributeValue("name").equals("lasPool.default") ){
                sensorNumber = Integer.parseInt(t.getAttributeValue("value"));
            }
            
            if(t.getAttributeValue("name").equals("lasPool.dev" + sensorNumber + ".maxRange") ){
                sensorRange = Double.parseDouble(t.getAttributeValue("value"));
            }            
        }
    }
        
    private void parseHEX(String hex){
        int numberOfPoints = hex.length()/4;
        double angleInterval = numberOfPoints > 1 ? (sensorMaxAngle-sensorMinAngle)/((double)numberOfPoints - 1.0 ) : 1; // in degrees
        int msB,lsB;
        double angle, distance;
        boolean isValid;
        firstPoint = null;
        for(int n = 0; n < numberOfPoints; n++){
            int i = n*4;
            // get the two next bytes in the sting
            lsB = Integer.parseInt(hex.substring(i,i+2), 16);
            msB = Integer.parseInt(hex.substring(i+2,i+4), 16);
            
            distance = ((msB & 0x7F) * 256 + lsB) * unit;
          
            isValid = (msB & 0x80) == 0x80 ? false : true;
            
            if(isValid && distance > 0.2){
                angle = -(n * angleInterval + sensorMinAngle) * Math.PI / 180.0; // angle in radians
                firstPoint = new LaserMapPoint(angle, distance, firstPoint);
            } 
        }
    }
    
    public LaserMapPoint getFirstPoint(){
        return this.firstPoint;
    }
    
    public int getMinAngleInDeg(){
        return (int)(this.sensorMinAngle);
    }
    public int getMaxAngleInDeg(){
        return (int)(this.sensorMaxAngle);
    }
    
    public double getSensorX(){
        return sensorX;
    }
    public double getSensorY(){
        return sensorY;
    }
    public double getSensorZ(){
        return sensorZ;
    }
    public double getSensorRange(){
        return sensorRange;
    }
    
    public double getRobotX(){
        return robotX;
    }
    public double getRobotY(){
        return robotY;
    }
    public double getRobotHeading(){
        return robotH;
    }

}
