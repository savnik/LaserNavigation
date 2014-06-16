/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package marg.model;

import marg.model.plugin.XMLParsePlugin;
import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.util.ArrayList;
import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.DefaultHandler;

/**
 *
 * @author Daniel
 */
public class XMLParser extends DefaultHandler {

    private String currentTag;
    XMLParsedData data;

    public XMLParser() {
        this.data = new XMLParsedData();
    }

    private ArrayList<XMLParsePlugin> parsePlugins = new ArrayList<XMLParsePlugin>();
    PropertyChangeSupport prop = new PropertyChangeSupport(this);

    public void addParsePlugin(XMLParsePlugin plugin) {
        plugin.setPropertyChangeSupport(prop);
        parsePlugins.add(plugin);
    }

    public void removeParsePlugin(XMLParsePlugin plugin) {
        parsePlugins.remove(plugin);
    }

    public void addPropertyChangeListener(String propertyName, PropertyChangeListener listener) {
        prop.addPropertyChangeListener(propertyName, listener);
    }

    public void removePropertyChangeListener(String propertyName, PropertyChangeListener listener) {
        prop.removePropertyChangeListener(propertyName, listener);
    }

    public void startDocument() throws SAXException {
        //System.out.println("Started Document");
    }

    public void endDocument() throws SAXException {
        //System.out.println("Ended Document");
    }

    public void startElement(String uri, String localName,
            String qName, Attributes attributes)
            throws SAXException {
        currentTag = qName;
        for (XMLParsePlugin plugin : parsePlugins) {
            plugin.startElement(qName, attributes);
        }
    }

    public void endElement(String uri, String localName, String qName)
            throws SAXException {
        for (XMLParsePlugin plugin : parsePlugins) {
            plugin.endElement(qName);
        }
    }

    public void characters(char ch[], int start, int length)
            throws SAXException {
        for (XMLParsePlugin plugin : parsePlugins) {
            plugin.tagContents(currentTag, ch, start, length);
        }
    }
}
