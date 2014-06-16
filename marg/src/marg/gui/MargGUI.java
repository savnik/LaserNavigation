/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * MargGUI2.java
 *
 * Created on 21-09-2009, 09:40:07
 */
package marg.gui;

import java.awt.BorderLayout;
import java.awt.CardLayout;
import java.awt.Component;
import java.net.URL;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.SwingUtilities;
import marg.config.Module;
import marg.config.Robot;
import marg.config.XMLConfigHandler;
import marg.gui.plugin.ModulePlugin;
import marg.util.PluginManager;
import marg.util.Log;

/**
 *
 * @author Kristina
 */
public class MargGUI extends javax.swing.JApplet {

    private static final String ROBOT_CONFIG_FILENAME = "robot.xml";
    private ModuleMenu menu;

    /** Initializes the applet MargGUI2 */
    public void init() {
        try {
            java.awt.EventQueue.invokeAndWait(new Runnable() {
                public void run() {
                    initComponents();
                    MainGUIHandler.getInstance().setMainGUI(MargGUI.this);

                    menu = new ModuleMenu(); //MargGUI must be set in MainGUIHandler
                    moduleMenuTab.add(menu, BorderLayout.CENTER);

                    Thread loadThread = new Thread(new Runnable() {
                        public void run() {
                            try {
                                Thread.sleep(300);
                            } catch (InterruptedException ex) {
                                Logger.getLogger(MargGUI.class.getName()).log(Level.SEVERE, null, ex);
                            }
                            final Robot loadedRobot = loadRobot();
                            loadTitle(loadedRobot);
                            loadControlButtons(loadedRobot);
                            addLoadedModules(loadedRobot.getModules());
                        }
                    });
                    loadThread.start();
                }
            });
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public void addModule(final ModuleTabs modTab) {
        SwingUtilities.invokeLater(new Runnable() {

            public void run() {
                moduleTabs.add(modTab, modTab.getModuleName());
            }
        });
    }

    public void showModule(final String moduleName) {
        SwingUtilities.invokeLater(new Runnable() {

            public void run() {
                CardLayout cl = (CardLayout) moduleTabs.getLayout();
                cl.show(moduleTabs, moduleName);
            }
        });
    }

    private Robot loadRobot() {
        XMLConfigHandler configHandler = new XMLConfigHandler();
        java.net.URL codebaseURL = MargGUI.this.getCodeBase();
        System.out.println("CodeBase: " + codebaseURL);
        if (XMLConfigHandler.isLocalURL(codebaseURL)) {
            codebaseURL = XMLConfigHandler.fixLocalUrl(codebaseURL);
        }
        URL configFile = XMLConfigHandler.getFileURL(codebaseURL, ROBOT_CONFIG_FILENAME);
        return configHandler.getConfigFrom(configFile);
    }

    private void loadTitle(Robot loadedRobot) {
        menu.setRobotName(loadedRobot.getRobotName());
    }

    private void loadControlButtons(Robot robot) {
        menu.setStartButton(robot.getStartButton());
        menu.setPauseButton(robot.getPauseButton());
        menu.setStopButton(robot.getStopButton());
    }

    //TODO: Refactor this out somewhere else? Keep margGUI as clean as possible!
    private void addLoadedModules(List<Module> loadedModules) {
        for (Module mod : loadedModules) {
            //Adding a module
            ModuleTabs module = new ModuleTabs(mod.getModuleName(), mod.getModuleHost(), mod.getPort(), mod.isAutoConnect());
            List<Class> availablePlugins = PluginManager.getInstance().getAvailableModulePlugins(); //Get available plugins
            for (String pluginName : mod.getModulePlugins()) {
                boolean foundAvailablePlugin = false;
                for (Class aClass : availablePlugins) {
                    if (aClass.getSimpleName().equalsIgnoreCase(pluginName)) { //Is the requested plugin available?
                        Object instance = PluginManager.getInstance().getInstanceOfClass(aClass); //Create an instance if so
                        if (instance != null) {
                            module.addModulePlugin((ModulePlugin) instance); //Add it to the module that requested the plugin
                            foundAvailablePlugin = true;
                            Log.GlobalLogger.info("Added plugin '" + pluginName + "' for module " + mod.getModuleName());
                        }
                        break; //Get out of the for-loop, we already found the plugin we needed.
                    }
                }
                if (!foundAvailablePlugin) {
                    Log.GlobalLogger.info("Could not load requested plugin '" + pluginName + "' for module " + mod.getModuleName());
                }
            }
            //Adding it to the gui
            menu.addModule(module);
        }
    }

    /**
     * Gets currently visible module in the cardlayout
     * @return The visible ModuleTabs object. null if no object is visisble.
     */
    public ModuleTabs getCurrentModule() {
        Component[] comp = moduleTabs.getComponents();
        for (Component com : comp) {
            if (com.isVisible() && com instanceof ModuleTabs) {
                return (ModuleTabs) com;
            }
        }
        return null;
    }

    /** This method is called from within the init() method to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        jPanel1 = new javax.swing.JPanel();
        moduleTabs = new javax.swing.JPanel();
        moduleMenuTab = new javax.swing.JPanel();

        jPanel1.setBackground(new java.awt.Color(50, 100, 133));

        moduleTabs.setLayout(new java.awt.CardLayout());

        moduleMenuTab.setLayout(new java.awt.BorderLayout());

        javax.swing.GroupLayout jPanel1Layout = new javax.swing.GroupLayout(jPanel1);
        jPanel1.setLayout(jPanel1Layout);
        jPanel1Layout.setHorizontalGroup(
            jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel1Layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(moduleMenuTab, javax.swing.GroupLayout.PREFERRED_SIZE, 304, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(moduleTabs, javax.swing.GroupLayout.DEFAULT_SIZE, 240, Short.MAX_VALUE)
                .addContainerGap())
        );
        jPanel1Layout.setVerticalGroup(
            jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel1Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(moduleMenuTab, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.DEFAULT_SIZE, 373, Short.MAX_VALUE)
                    .addComponent(moduleTabs, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.DEFAULT_SIZE, 373, Short.MAX_VALUE))
                .addContainerGap())
        );

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(jPanel1, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(jPanel1, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
        );
    }// </editor-fold>//GEN-END:initComponents
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JPanel jPanel1;
    private javax.swing.JPanel moduleMenuTab;
    private javax.swing.JPanel moduleTabs;
    // End of variables declaration//GEN-END:variables

    /**
     * Called by the browser when an applet is closed.. Cleanup!
     */
    @Override
    public void destroy() {
        Component[] comps = moduleTabs.getComponents();
        for (Component comp : comps) {
            if (comp instanceof RobotModule) {
                RobotModule module = (RobotModule) comp;
                Log.GlobalLogger.info("Closing Module: " + module.getModuleName());
                module.getXMLClientHandler().disconnect();
                module.stopPlugins();
            }
        }
    }
}
