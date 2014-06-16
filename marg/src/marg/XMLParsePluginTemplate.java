/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg;

import marg.model.plugin.*;
import java.beans.PropertyChangeSupport;
import org.xml.sax.Attributes;

/**
 *
 * @author Daniel
 */
public class XMLParsePluginTemplate implements XMLParsePlugin {

    private PropertyChangeSupport prop;

    public void setPropertyChangeSupport(PropertyChangeSupport prop) {
        this.prop = prop;
    }

    /**
     * Called when the starttag of an element is received
     * @param tagName name of the element/tag
     * @param atts attributes of the element
     */
    public void startElement(String tagName, Attributes atts) {
        //Example: Firing a callback
        //All listeners who has subscribed to "propertyName" will receive this callback
        prop.firePropertyChange("propertyName", "oldValue", tagName);
        //End of Example
    }

    /**
     * Called when the endtag of an element is received
     * @param tagName name of the element/tag
     */
    public void endElement(String tagName) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    /**
     * Called when contents of an element is received. Data can be of any kind
     * @param parentTag name of the first parent tag this content was wrapped in
     * @param ch character buffer
     * @param start start offset for the content in character buffer
     * @param length length of content
     */
    public void tagContents(String parentTag, char[] ch, int start, int length) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

}
