/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.model.plugin;

import java.beans.PropertyChangeSupport;
import org.xml.sax.Attributes;


/**
 *
 * @author Daniel
 */
public interface XMLParsePlugin {

    /**
     * Called when the starttag of an element is received
     * @param tagName name of the element/tag
     * @param atts attributes of the element
     */
    public void startElement(String tagName, Attributes atts);

    /**
     * Called when the endtag of an element is received
     * @param tagName name of the element/tag
     */
    public void endElement(String tagName);

    /**
     * Called when contents of an element is received. Data can be of any kind
     * @param parentTag name of the first parent tag this content was wrapped in
     * @param ch character buffer
     * @param start start offset for the content in character buffer
     * @param length length of content
     */
    public void tagContents(String parentTag, char ch[], int start, int length);

    /**
     * Sets the PropertyChangeSupport object of the parent XMLParser.
     * By contract this method will be called as the first thing when an XMLParsePlugin is added.
     * This is a central callback hub that should be used to fire PropertyChangeEvents.
     * This is the standard way to make callbacks when new data is received.
     * Data can be sent as the new value when you fire a callback,
     * but feel free to make your own data access methods as well
     * @param prop the PropertyChangeSupport object sent from parent XMLParser
     */
    public void setPropertyChangeSupport(PropertyChangeSupport prop);
}
