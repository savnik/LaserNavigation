/*
 * LaserMap.java
 * 
 * @author Jonas Eriksen (s082598)
 */

package marg.gui.plugin;

import java.awt.*;
import java.awt.event.*;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import javax.swing.JPanel;
import marg.handlers.XMLClientHandler;
import marg.model.LaserXMLTag;
import marg.model.plugin.LaserParsePlugin;
import marg.gui.plugin.lasermap.LaserMapPainter;
import marg.gui.plugin.lasermap.LaserMapData;

public class LaserMap extends javax.swing.JPanel implements ModulePlugin, PropertyChangeListener {

    private XMLClientHandler handler;
    private LaserMapPainter mapPainter;
    private LaserMapData mapData;
    private javax.swing.JButton zoomInButton;
    private javax.swing.JButton zoomOutButton;
    private javax.swing.JButton panLeft;
    private javax.swing.JButton panRight;
    private javax.swing.JButton panUp;
    private javax.swing.JButton panDown;
    private javax.swing.JButton panReset;
    private javax.swing.JComboBox robotSelector;
    private javax.swing.JLabel robotSelectorLabel;
    private javax.swing.JComboBox displayOption;
    private javax.swing.JLabel displayOptionLabel;
    

    /** Creates new form ModulePluginJPanelTemplate */
    public LaserMap() {
        
        setBackground(new java.awt.Color(200, 221, 242));
       
        // Contruct components
        mapData = new LaserMapData();
        mapPainter = new LaserMapPainter();
       
        // Layout components
        zoomInButton = new javax.swing.JButton();
        zoomInButton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/marg/gui/plugin/lasermap/zoomIn.png")));
        zoomInButton.setToolTipText("Zoom In");
        zoomInButton.setBorder(null);
        zoomInButton.setContentAreaFilled(false);
        zoomInButton.setFocusPainted(false);
        zoomInButton.setCursor(new java.awt.Cursor(java.awt.Cursor.HAND_CURSOR));
        zoomInButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                mapPainter.zoomIn();
            }
        });
        zoomOutButton = new javax.swing.JButton();
        zoomOutButton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/marg/gui/plugin/lasermap/zoomOut.png")));
        zoomOutButton.setToolTipText("Zoom Out");
        zoomOutButton.setBorder(null);
        zoomOutButton.setContentAreaFilled(false);
        zoomOutButton.setFocusPainted(false);
        zoomOutButton.setCursor(new java.awt.Cursor(java.awt.Cursor.HAND_CURSOR));
        zoomOutButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                mapPainter.zoomOut();
            }
        });
        panLeft = new javax.swing.JButton();
        panLeft.setIcon(new javax.swing.ImageIcon(getClass().getResource("/marg/gui/plugin/lasermap/panLeft.png")));
        panLeft.setToolTipText("Pan Left");
        panLeft.setBorder(null);
        panLeft.setContentAreaFilled(false);
        panLeft.setFocusPainted(false);
        panLeft.setCursor(new java.awt.Cursor(java.awt.Cursor.HAND_CURSOR));
        panLeft.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                mapPainter.translateLeft();
            }
        });
        panRight = new javax.swing.JButton();
        panRight.setIcon(new javax.swing.ImageIcon(getClass().getResource("/marg/gui/plugin/lasermap/panRight.png")));
        panRight.setToolTipText("Pan Right");
        panRight.setBorder(null);
        panRight.setContentAreaFilled(false);
        panRight.setFocusPainted(false);
        panRight.setCursor(new java.awt.Cursor(java.awt.Cursor.HAND_CURSOR));
        panRight.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                mapPainter.translateRight();
            }
        });
        panUp = new javax.swing.JButton();
        panUp.setIcon(new javax.swing.ImageIcon(getClass().getResource("/marg/gui/plugin/lasermap/panUp.png")));
        panUp.setToolTipText("Pan Up");
        panUp.setBorder(null);
        panUp.setContentAreaFilled(false);
        panUp.setFocusPainted(false);
        panUp.setCursor(new java.awt.Cursor(java.awt.Cursor.HAND_CURSOR));
        panUp.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                mapPainter.translateUp();
            }
        });
        panDown = new javax.swing.JButton();
        panDown.setIcon(new javax.swing.ImageIcon(getClass().getResource("/marg/gui/plugin/lasermap/panDown.png")));
        panDown.setToolTipText("Pan Down");
        panDown.setBorder(null);
        panDown.setContentAreaFilled(false);
        panDown.setFocusPainted(false);
        panDown.setCursor(new java.awt.Cursor(java.awt.Cursor.HAND_CURSOR));
        panDown.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                mapPainter.translateDown();
            }
        });
        panReset = new javax.swing.JButton();
        panReset.setIcon(new javax.swing.ImageIcon(getClass().getResource("/marg/gui/plugin/lasermap/panReset.png")));
        panReset.setToolTipText("Center Robot");
        panReset.setBorder(null);
        panReset.setContentAreaFilled(false);
        panReset.setFocusPainted(false);
        panReset.setCursor(new java.awt.Cursor(java.awt.Cursor.HAND_CURSOR));
        panReset.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                mapPainter.translateReset();
            }
        });
        // Selectors
        robotSelector = new javax.swing.JComboBox();
        robotSelector.setModel(new javax.swing.DefaultComboBoxModel(mapPainter.getRobotTypes()));
        robotSelector.setBorder(null);
        robotSelector.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                mapPainter.setRobotType(robotSelector.getSelectedItem().toString());
            }
        });
        robotSelectorLabel = new javax.swing.JLabel();
        robotSelectorLabel.setForeground(new java.awt.Color(0, 0, 0));
        robotSelectorLabel.setText("Robot Type:");

        displayOption = new javax.swing.JComboBox();
        displayOption.setModel(new javax.swing.DefaultComboBoxModel(mapPainter.getDisplayOptions()));
        displayOption.setBorder(null);
        displayOption.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                mapPainter.setDisplayOption(displayOption.getSelectedItem().toString());
            }
        });
        displayOptionLabel = new javax.swing.JLabel();
        displayOptionLabel.setForeground(new java.awt.Color(0, 0, 0));
        displayOptionLabel.setText("Robot Display Style:");

        javax.swing.GroupLayout mapPainterLayout = new javax.swing.GroupLayout(mapPainter);
        mapPainter.setLayout(mapPainterLayout);
        mapPainterLayout.setHorizontalGroup(
            mapPainterLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(mapPainterLayout.createSequentialGroup()
                .addGroup(mapPainterLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(mapPainterLayout.createSequentialGroup()
                        .addComponent(robotSelectorLabel)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(robotSelector, javax.swing.GroupLayout.PREFERRED_SIZE, 119, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(displayOptionLabel)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(displayOption, javax.swing.GroupLayout.PREFERRED_SIZE, 120, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(0, 0, Short.MAX_VALUE))
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, mapPainterLayout.createSequentialGroup()
                        .addGroup(mapPainterLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addComponent(panDown, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addGroup(mapPainterLayout.createSequentialGroup()
                                .addComponent(panLeft, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(panReset, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addComponent(panUp, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(panRight, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 356, Short.MAX_VALUE)
                        .addComponent(zoomOutButton)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(zoomInButton)))
                .addContainerGap())
        );
        mapPainterLayout.setVerticalGroup(
            mapPainterLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(mapPainterLayout.createSequentialGroup()
                .addContainerGap()
                .addGroup(mapPainterLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(zoomInButton)
                    .addComponent(zoomOutButton))
                .addContainerGap())
            .addGroup(mapPainterLayout.createSequentialGroup()
                .addGroup(mapPainterLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(panRight, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addGroup(mapPainterLayout.createSequentialGroup()
                        .addComponent(panUp, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(mapPainterLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addComponent(panLeft, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(panReset, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE))))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(panDown, javax.swing.GroupLayout.PREFERRED_SIZE, 24, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 167, Short.MAX_VALUE)
                .addGroup(mapPainterLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(robotSelector, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(robotSelectorLabel)
                    .addComponent(displayOptionLabel)
                    .addComponent(displayOption, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)))
        );

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(mapPainter, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addContainerGap())
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(mapPainter, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addContainerGap())
        );
       
       
    }
    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        setBackground(new java.awt.Color(200, 221, 242));

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 554, Short.MAX_VALUE)
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 307, Short.MAX_VALUE)
        );
    }// </editor-fold>//GEN-END:initComponents

    // Variables declaration - do not modify//GEN-BEGIN:variables
    // End of variables declaration//GEN-END:variables

    //This will always be called before startPlugin()
    public void setXMLClientHandler(XMLClientHandler handler) {
        this.handler = handler;
    }
    
    public XMLClientHandler getXMLClientHandler() {
        return handler;
    }

    public String getPluginName() {
        return "LaserMap";
    }

    public JPanel getJPanel() {
        return this;
    }
    
    @Override
    public void setVisible(boolean flag){
        super.setVisible(flag);
        
        if(handler != null){
            if(flag){
                handler.sendCmd("scanget codec=HEX");
                handler.sendCmd("push t=\""+ 1 +"\" cmd=\"scanget codec=HEX\"");
            }else{
                handler.sendCmd("push flush=\"scanget codec=HEX\"");
            }
        }
            
    }

    public void startPlugin() {
        //Initialize needed ressources
        handler.getXMLParser().addParsePlugin(new LaserParsePlugin());
        handler.addPropertiesChangeListener(this, LaserParsePlugin.SUBSCRIBE_LASERDATA);

    }
    
    
    public void stopPlugin() {
        handler.sendCmd("push flush=\"scanget codec=HEX\"");
    }

    public void doUpdate() {
        //Update some content
    }

    public void propertyChange(PropertyChangeEvent evt) {
        if (evt.getPropertyName().equals(LaserParsePlugin.SUBSCRIBE_LASERDATA)) {
            
            mapData.parseXML((LaserXMLTag) evt.getNewValue());
            
            if(mapData.isReady()){
                mapPainter.updateMap(mapData);
            }

        }
    }       
}
