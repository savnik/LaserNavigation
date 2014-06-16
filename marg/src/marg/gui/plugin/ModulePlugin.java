/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.gui.plugin;

import javax.swing.JPanel;
import marg.handlers.XMLClientHandler;

/**
 *
 * @author Daniel
 */
public interface ModulePlugin {

    /**
     * Offers the handler to your plugin.
     * The handler is responsible for communicating with the parent module and
     * is also where you should register for propertyChangeEvents and add new XMLParsePlugins.
     * This method will by contract always be called before startPlugin();
     * @param handler the XMLClientHandler object sent from the Module
     */
    public void setXMLClientHandler(XMLClientHandler handler);

    /**
     * Gets the name of your plugin.
     * Used as the header for the tab of your plugin.
     * @return a String representing the name of your plugin
     */
    public String getPluginName();

    /**
     * Return the visual representation of your plugin here.
     * Parent module calls this method to retrieve a JPanel and adds it to its tabbed pane.
     * @return a JPanel object created by your plugin
     */
    public JPanel getJPanel();

    /**
     * Called by the parent module to start a plugin.
     * Use this method to initialize ressources that are not needed until
     * your plugin is actually started
     */
    public void startPlugin();

    /**
     * Called by the parent module during cleanup process when the module or entire applet is closed.
     * Clean-up of ressources should be placed here.
     */
    public void stopPlugin();

    /**
     * Called by parent module regularly every second by default.
     * Saves you having to make a Thread if you need a simple regular update in your plugin.
     */
    public void doUpdate();
}
