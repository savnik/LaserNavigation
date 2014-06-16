package marg;

import marg.gui.plugin.*;
import javax.swing.JPanel;
import marg.handlers.XMLClientHandler;

/**
 * Insert description of this plugin
 * @author Insert author of plugin
 */
public class ModulePluginTemplate implements ModulePlugin {

    private XMLClientHandler handler;

    public void setXMLClientHandler(XMLClientHandler handler) {
        this.handler = handler;
    }

    /**
     * Gets the name of your plugin.
     * Used as the header for the tab of your plugin.
     * @return a String representing the name of your plugin
     */
    public String getPluginName() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    /**
     * Return the visual representation of your plugin here.
     * Parent module calls this method to retrieve a JPanel and adds it to its tabbed pane.
     * @return a JPanel object created by your plugin
     */
    public JPanel getJPanel() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    /**
     * Called by the parent module to start a plugin.
     * Use this method to initialize ressources that are not needed until
     * your plugin is actually started
     */
    public void startPlugin() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    /**
     * Called by the parent module during cleanup process when the module or entire applet is closed.
     * Clean-up of ressources should be placed here.
     */
    public void stopPlugin() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    /**
     * Called by parent module regularly every second by default.
     * Saves you having to make a Thread if you need a simple regular update in your plugin.
     */
    public void doUpdate() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

}
