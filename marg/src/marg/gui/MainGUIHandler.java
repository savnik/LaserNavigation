/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.gui;

import java.util.Collection;
import java.util.HashMap;
import javax.swing.JToggleButton;
import marg.model.MonitoredVariable;

/**
 *
 * @author Daniel
 */
public class MainGUIHandler {
    
    private static MainGUIHandler instance;
    private ModuleMenu moduleMenu;
    private MargGUI mainGUI;
    
    public static MainGUIHandler getInstance() {
        if (instance == null)
            instance = new MainGUIHandler();
        return instance;
    }

    private MainGUIHandler() {
    }

    private HashMap<String, RobotModule> moduleMap = new HashMap<String, RobotModule>();
    private HashMap<String, JToggleButton> buttonMap = new HashMap<String, JToggleButton>();

    public void addModuleMapping(RobotModule module) {
        moduleMap.put(module.getModuleName(), module);
    }

    public RobotModule getModuleByName(String moduleName) {
        return moduleMap.get(moduleName);
    }

    public Collection<RobotModule> getAllModules() {
        return moduleMap.values();
    }

    public void addButtonMapping(String moduleName, JToggleButton button) {
        buttonMap.put(moduleName, button);
    }

    public JToggleButton getMenuButtonByName(String moduleName) {
        return buttonMap.get(moduleName);
    }

    public void setModuleMenu(ModuleMenu modMenu) {
        this.moduleMenu = modMenu;
    }

    public MargGUI getMainGUI() {
        return mainGUI;
    }

    public ModuleMenu getModuleMenu() {
        return moduleMenu;
    }

    public void setMainGUI(MargGUI margGUI) {
        this.mainGUI = margGUI;
    }

    public void addModule(ModuleTabs moduleTabs) {
        mainGUI.addModule(moduleTabs);
    }

    public ModuleTabs getCurrentModule() {
        return mainGUI.getCurrentModule();
    }

    public void showModule(String moduleName) {
        mainGUI.showModule(moduleName);
    }

    public void addMonitoredVariable(MonitoredVariable monitoredVar) {
        moduleMenu.addMonitoredVariable(monitoredVar);
    }

}
