/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.gui;

import java.awt.Color;
import marg.gui.plugin.ModulePlugin;
import marg.handlers.XMLClientHandler;

/**
 *
 * @author Daniel
 */
public interface RobotModule {

    /**
     * Adds a plugin to the module
     * @param modPlugin the plugin to be added
     */
    public void addModulePlugin(ModulePlugin modPlugin);

    /**
     * Removes a plugin from the module
     * @param modPlugin the plugin to be removed
     */
    public void removeModulePlugin(ModulePlugin modPlugin);

    /**
     * Gets the name of this module. Used on its button in the menu
     * @return the name of this module
     */
    public String getModuleName();

    /**
     * Gets the current status of the module as a Color object
     * @return a Color object representing the module's status
     */
    public Color getModuleStatusColor();

    /**
     * Get's the underlying XMLClientHandler used by the module.
     * The handler is the one used by the module to communicate
     * with an actual module on the robot.
     * @return the underlying handler used by the module
     */
    public XMLClientHandler getXMLClientHandler();

    /**
     * Stops all plugins running on the module. Mainly used to do cleanup.
     */
    public void stopPlugins();

}
