/*
 * LaserMapPainter.java
 * 
 * @author Jonas Eriksen (s082598)
 */

package marg.gui.plugin.lasermap;

import java.awt.*;
import javax.swing.*;

public class LaserMapPainter extends javax.swing.JPanel {
    
    public enum RobotType{
       SMR, MMR, Hako, iRobot, Guidebot;
    }
    public enum DisplayOption{
        Fixed, Rotate;
    }
    
    private RobotType robot = RobotType.SMR;
    private int scale = 6; // unit=metre
    private int ppm, rx, ry, sx, sy, mapHeight, mapWidth;
    private int translateX = 0;
    private int translateY = 0;
    private DisplayOption selectedDisplayOption = DisplayOption.Fixed;
    private LaserMapData mapData = new LaserMapData();
    
    
    public LaserMapPainter() {
        super();
        this.setBackground(new Color(200,221,242));
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        mapHeight = getHeight();
        mapWidth = getWidth();
        ppm = mapHeight/scale;
        
        
        //Robot center coordinates
        rx = mapWidth/2;
        if(selectedDisplayOption == DisplayOption.Fixed){
            ry = (int)(mapHeight*0.8);
        }else if(selectedDisplayOption == DisplayOption.Rotate){
            ry = mapHeight/2;
        }
        
        // Sensor coordinates
        sx = rx - (int)(mapData.getSensorX()*ppm);
        sy = ry - (int)(mapData.getSensorY()*ppm);
        
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON); // makes smooth lines
        
        
        // Translate Map (userinput)
        if(translateX != 0 || translateY != 0)
            g2.translate(translateX, translateY);
        
        drawGrid(g2);
        
        if(selectedDisplayOption == DisplayOption.Rotate){
            g2.rotate(mapData.getRobotHeading(), rx, ry);
        }
        
        drawDataPoints(g2);

        drawRangeArcs(g2);

        drawRobot(g2);
        
        if(selectedDisplayOption == DisplayOption.Rotate){
            g2.rotate(-mapData.getRobotHeading(), rx, ry);
        }
        
        drawRangeArcsText(g2);
        
        // Translate back so buttons are drawn correctly
        if(translateX != 0 || translateY != 0)
            g2.translate(-translateX, -translateY);
    }
    
    public void updateMap(LaserMapData mapData){
        this.mapData = mapData;
        this.redraw();
    }
    
    public void zoomOut(){
        if(this.scale < 100){
            this.scale += 2;
        }
        this.redraw();
    }
    
    public void zoomIn(){
        if(this.scale > 2){
            this.scale -= 2;
        }
        this.redraw();
    }
    
    public void translateLeft(){
        this.translateX += (getWidth()+getHeight())/20;
        this.redraw();
    }
    public void translateRight(){
        this.translateX -= (getWidth()+getHeight())/20;
        this.redraw();
    }
    public void translateUp(){
        this.translateY += (getWidth()+getHeight())/20;
        this.redraw();
    }
    public void translateDown(){
        this.translateY -= (getWidth()+getHeight())/20;
        this.redraw();
    }
    public void translateReset(){
        this.translateX = 0;
        this.translateY = 0;
        this.redraw();
    }   
    
    public String[] getRobotTypes(){
        int i = 0;
        String[] s = new String[RobotType.values().length];
        for(RobotType r : RobotType.values()){
            s[i] = r.name();
            i++;
        }
        return s;
    }
    
    public void setRobotType(String s){
        this.robot = RobotType.valueOf(s);
        this.redraw();
    }
    
    public String[] getDisplayOptions(){
        int i = 0;
        String[] s = new String[DisplayOption.values().length];
        for(DisplayOption r : DisplayOption.values()){
            s[i] = r.name();
            i++;
        }
        return s;
    }
    public void setDisplayOption(String s){
        this.selectedDisplayOption = DisplayOption.valueOf(s);
        this.redraw();
    }
    
    private void redraw() {
    SwingUtilities.invokeLater(
        new Runnable() {
            public void run() {
                LaserMapPainter.this.repaint();
            }
        });
    }
    
    private void drawGrid(Graphics2D g){
        
        // Set size of grid
        int lineInterval;
        if(ppm < 20){
            lineInterval = ppm*2;
        }else if(ppm < 40){
            lineInterval = ppm;
        }else if(ppm < 100){
            lineInterval = ppm/2;
        }else{
            lineInterval = ppm/10;
        }
        
        
        // Set grid color
        g.setPaint(new Color(210, 230, 255));
        
        int x = rx;
        int y = ry;
        
        // Choose between moving or fixed grid
        if(selectedDisplayOption == DisplayOption.Fixed){
            // Origo cross
            g.setStroke(new BasicStroke(2));
            g.drawLine(-translateX, y, mapWidth - translateX , y);
            g.drawLine(x, -translateY, x, mapHeight - translateY);
        }else if(selectedDisplayOption == DisplayOption.Rotate){
            // Move grid with the datapoints
            x = rx + (int)(mapData.getRobotX()*ppm);
            y = ry + (int)(mapData.getRobotY()*ppm);
        }
        
        // Horizontal lines
        int l = y;
        while(l > 0 - translateY){
            g.drawLine(-translateX, l, mapWidth - translateX , l);
            l -= lineInterval;
        }
        l = y;
        while(l < mapHeight - translateY){
            g.drawLine(-translateX, l, mapWidth - translateX, l);
            l += lineInterval;
        }
        
        // Vertical lines
        l = x;
        while(l > 0 - translateX){
            g.drawLine(l, -translateY, l, mapHeight - translateY);
            l -= lineInterval;
        }
        l = x;
        while(l < mapWidth - translateX){
            g.drawLine(l, -translateY, l, mapHeight - translateY);
            l += lineInterval;
        }
        
    }
    
    private void drawRangeArcs(Graphics2D g){
        
        int range = (int) mapData.getSensorRange();
        
        g.setPaint(new Color(100, 150, 183));
        for(int i=1; i <= range; i++){
            g.drawArc(sx-ppm*i, sy-ppm*i, i*2*ppm, i*2*ppm, mapData.getMinAngleInDeg(), mapData.getMaxAngleInDeg()-mapData.getMinAngleInDeg());
        }
    }
    
    private void drawRangeArcsText(Graphics2D g){
   
        // Set range arc text color
        g.setPaint(new Color(100, 150, 183));
        // itterate over the range arcs
        int range = (int) mapData.getSensorRange();
        
        if(selectedDisplayOption == DisplayOption.Rotate){
            int lx,ly;
            double cosTheta = Math.cos(mapData.getRobotHeading());
            double sinTheta = Math.sin(mapData.getRobotHeading());
            
            for(int i=1; i <= range; i++){
                lx = (int)((i*ppm+20) * cosTheta);
                ly = (int)((i*ppm+20) * sinTheta);
                // draw text on the left and right side of the robot
                g.drawString(i+"m", rx+lx-10, ry+ly);
                g.drawString(i+"m", rx-lx-10, ry-ly);
            }

        }else if(selectedDisplayOption == DisplayOption.Fixed){
            
            for(int i=1; i <= range; i++){
                g.drawString(i+"m", sx+i*ppm+10, sy);
                g.drawString(i+"m", sx-i*ppm-30, sy);
            }
        }
    }
    
    private void drawDataPoints(Graphics2D g){
        
        g.setPaint(new Color(30, 100, 20));

        int pointSize = 8;
        if(this.scale > 12){
            pointSize = 5;
        }else if(this.scale > 6){
            pointSize = 6;
        }else if(this.scale > 2){
            pointSize = 7;
        }
               
        int i=0;
        LaserMapPoint p = mapData.getFirstPoint();
        while(p != null){
            i++;
            g.fillOval( sx + (int)(p.getCartesianX()*ppm-pointSize/2), sy + (int)(p.getCartesianY()*ppm-pointSize/2), pointSize, pointSize);
            p = p.next();
        }
    }
    
    private void drawRobot(Graphics2D g){
        
        int cornerDiameter = 4; // diameter of the coners on the roundedrectangles
        
        // SMR Robot
        if(this.robot.equals(RobotType.SMR)){
            
            final double robotBodyLength = 0.28;
            final double robotBodyWidth = 0.26;
            final double robotWidth = 0.28;
            final double wheelWidth = 0.03 ;
            final double wheelDiameter = 0.065;
            final double wheelToFront = 0.235;
            
            int l1 = (int)(robotBodyWidth/2*ppm);
            int l2 = (int)(robotBodyLength/2*ppm);
            int l3 = (int)(wheelDiameter/2*ppm);
            int l4 = (int)(wheelWidth*ppm);
            int l5 = (int)(wheelToFront*ppm);
            int l6 = (int)(robotWidth/2*ppm);
            
            g.setPaint(new Color(50, 50, 50));
            g.drawRoundRect(rx-l1, ry-l5, l1*2, l2*2, cornerDiameter, cornerDiameter);
            g.drawLine(rx-l1, ry-l5+(int)(0.18*ppm), rx+l1, ry-l5+(int)(0.18*ppm));
            // Wheels back
            g.setPaint(new Color(0, 0, 0));
            g.fillRoundRect(rx-l6, ry-l3, l4, l3*2, cornerDiameter, cornerDiameter);
            g.fillRoundRect(rx+l6-l4, ry-l3, l4, l3*2, cornerDiameter, cornerDiameter);
            // Wheels front
            g.fillRoundRect(rx-l6, ry-l5, l4, l3, cornerDiameter, cornerDiameter);
            g.fillRoundRect(rx+l6-l4, ry-l5, l4, l3, cornerDiameter, cornerDiameter);
            
            // Body
            g.setPaint(new Color(200, 200, 200));
            g.fillRoundRect(rx-l1, ry-l5, l1*2, l2*2, cornerDiameter, cornerDiameter);
            g.setPaint(new GradientPaint(rx,ry-l5,Color.LIGHT_GRAY,rx+l1,(float)(ry-l5+0.18*ppm),Color.WHITE ));
            g.fillRoundRect(rx-l1, ry-l5, l1*2, (int)(0.18*ppm), cornerDiameter, cornerDiameter);

        // MMR Robot
        }else if(this.robot.equals(RobotType.MMR)){
                        
            final double robotBodyLength = 0.70;
            final double robotBodyWidth = 0.40;
            final double robotWidth = 0.52;
            final double wheelWidth = 0.06 ;
            final double wheelDiameter = 0.35;
            final double wheelToFront = 0.465;
            
            int l1 = (int)(robotBodyWidth/2*ppm);
            int l2 = (int)(robotBodyLength/2*ppm);
            int l3 = (int)(wheelDiameter/2*ppm);
            int l4 = (int)(wheelWidth*ppm);
            int l5 = (int)(wheelToFront*ppm);
            int l6 = (int)(robotWidth/2*ppm);
            // Body
            g.setPaint(new Color(200, 200, 200));
            g.fillRoundRect(rx-l1, ry-l5, l1*2, l2*2, cornerDiameter, cornerDiameter);
            g.setPaint(new Color(50, 50, 50));
            g.drawRoundRect(rx-l1, ry-l5, l1*2, l2*2, cornerDiameter, cornerDiameter);
            // Wheels back
            g.setPaint(new Color(0, 0, 0));
            g.fillRoundRect(rx-l6, ry-l3, l4, l3*2, cornerDiameter, cornerDiameter);
            g.fillRoundRect(rx+l6-l4+1, ry-l3, l4, l3*2, cornerDiameter, cornerDiameter);
            
        // iRobot
        }else if(this.robot.equals(RobotType.iRobot)){
            
            final double robotBodyLength = 0.64;
            final double robotBodyWidth = 0.40;
            final double robotWidth = 0.62;
            final double wheelWidth = 0.1;
            final double wheelDiameter = 0.3;
            final double wheelToCenter = 0.2;
            final double bumperThickness = 0.05;
            final double frontBumperToCenter = 0.5;
            final double rearBumperToCenter = 0.4;
            
            int l1 = (int)(robotBodyWidth/2*ppm);
            int l2 = (int)(robotBodyLength/2*ppm);
            int l3 = (int)(wheelDiameter/2*ppm);
            int l4 = (int)(wheelWidth*ppm);
            int l5 = (int)(wheelToCenter*ppm);
            int l6 = (int)(robotWidth/2*ppm);
            int r1 = (int)(bumperThickness*ppm);
            int r2 = (int)(frontBumperToCenter*ppm);
            int r3 = (int)(rearBumperToCenter*ppm);
            
            // Wheels back
            g.setPaint(new Color(0, 0, 0));
            g.fillRoundRect(rx-l6, ry-l3-l5, l4, l3*2, cornerDiameter, cornerDiameter);
            g.fillRoundRect(rx+l6-l4+1, ry-l3-l5, l4, l3*2, cornerDiameter, cornerDiameter);
            // Wheels front
            g.fillRoundRect(rx-l6, ry-l3+l5, l4, l3*2, cornerDiameter, cornerDiameter);
            g.fillRoundRect(rx+l6-l4+1, ry-l3+l5, l4, l3*2, cornerDiameter, cornerDiameter);
            // Front bumper
            g.setPaint(new Color(100, 100, 100));
            g.fillRect(rx-l1, ry-r1-r2, l1*2, r1);
            g.setPaint(new Color(0, 0, 0));
            g.fillRoundRect(rx-l1, ry-r1-r2, l1*2, r1/2, cornerDiameter, cornerDiameter);
            g.fillRect(rx-l1, ry-r2, r1/3, r2);
            g.fillRect(rx+l1-r1/3, ry-r2, r1/3, r2);
            // Rear bumper
            g.setPaint(new Color(100, 100, 100));
            g.fillRect(rx-l1, ry+r3, l1*2, r1);
            g.setPaint(new Color(0, 0, 0));
            g.fillRoundRect(rx-l1, ry+r1/2+r3, l1*2, r1/2, cornerDiameter, cornerDiameter);
            g.fillRect(rx-l1, ry, r1/3, r3);
            g.fillRect(rx+l1-r1/3, ry, r1/3, r3);
            // Body
            g.setPaint(new GradientPaint(rx-l1,0,new Color(80, 20, 20),rx-l4,0,new Color(170, 20, 20),true ));
            g.fillRoundRect(rx-l1, ry-l2, l1*2, l2*2, cornerDiameter, cornerDiameter);
            g.setPaint(new Color(150, 20, 20));
            g.fillRoundRect(rx-(int)(l4*1.5), ry-l2, l4*3, l2*2, cornerDiameter, cornerDiameter);
            g.setPaint(Color.BLACK);
            g.drawRoundRect(rx-l1, ry-l2, l1*2, l2*2, cornerDiameter, cornerDiameter);
          
        // Guidebot
        }else if(this.robot.equals(RobotType.Guidebot)){
            
            final double robotBodyRadius = 0.34;
            final double wheelWidth = 0.05;
            final double wheelDiameter = 0.2;
            final double wheelToCenter = 0.25;
            
            int br = (int)(robotBodyRadius*ppm);
            int ww = (int)(wheelWidth/2*ppm);
            int wd = (int)(wheelDiameter/2*ppm);
            int wc = (int)(wheelToCenter*ppm);
            
            // The body
            g.setPaint(new Color(40, 40, 40));
            g.fillOval(rx-br, ry-br, br*2, br*2);
            
            // The wheels
            g.setPaint(new Color(0, 0, 0));
            g.fillRoundRect(rx-wc-ww, ry-wd, ww*2, wd*2, cornerDiameter*2, cornerDiameter*2);
            g.fillRoundRect(rx+wc-ww, ry-wd, ww*2, wd*2, cornerDiameter*2, cornerDiameter*2);
         
            
        // Hako Robot
        }else if(this.robot.equals(RobotType.Hako)){
            
            
            
        }
        
        // sensor
        int sensorWidth = (int)(0.04*ppm);
        g.setPaint(new Color(50, 100, 200));
        g.fillRoundRect(sx-sensorWidth/2-1, sy-sensorWidth/2-1, sensorWidth+2, sensorWidth+2, cornerDiameter, cornerDiameter);
        g.setPaint(Color.BLACK);
        g.fillOval(sx-sensorWidth/2, sy-sensorWidth/2, sensorWidth, sensorWidth);
    }
}
