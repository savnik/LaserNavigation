/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.handlers;

import java.beans.PropertyChangeListener;
import marg.model.XMLClient;
import marg.model.plugin.XMLParsePlugin;
import marg.model.XMLParser;

/**
 *
 * @author Daniel
 */
public class XMLClientHandler {

    private static final String SETVALUE_CMD = "var %s=\"%s\"";
    private XMLClient client;

    public XMLClientHandler(XMLClient client) {
        this.client = client;
    }

    public boolean connect(String host, int port) {
        if (!client.isConnected())
            return client.connect(host, port);
        else
            return false;
    }

    public void disconnect() {
        client.disconnect();
    }

    public void addParsePlugin(XMLParsePlugin plugin) {
        client.getXMLParser().addParsePlugin(plugin);
    }

    public void addPropertiesChangeListener(PropertyChangeListener listener, String... propertyNames) {
        for (String propName : propertyNames) {
            client.getXMLParser().addPropertyChangeListener(propName, listener);
        }
    }

    public void addPropertyChangeListener(String propertyName, PropertyChangeListener listener) {
        client.getXMLParser().addPropertyChangeListener(propertyName, listener);
    }

    public void removePropertyChangeListener(String propertyName, PropertyChangeListener listener) {
        client.getXMLParser().removePropertyChangeListener(propertyName, listener);
    }
    
    public void sendCmd(String cmd) {
        if (client.isConnected()) {
            client.sendCmd(cmd);
        } else {
            System.err.println("Not Connected; Tried sending cmd: "+ cmd);
        }
    }

    public void setVariable(String varName, String newValue) {
        String setValCmd = String.format(SETVALUE_CMD, varName, newValue);
        client.sendCmd(setValCmd);
        client.sendCmd("var allcopy");
    }

    public boolean isConnected() {
        return client.isConnected();
    }

    public XMLParser getXMLParser() {
        return client.getXMLParser();
    }

}
