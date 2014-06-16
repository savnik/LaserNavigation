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
public class RawDataPlugin implements XMLParsePlugin {

    public final static String SUBSCRIBE_RAWDATA = "rawData";
    private PropertyChangeSupport prop;
    private String rawData;

    public RawDataPlugin() {
    }

    public void setPropertyChangeSupport(PropertyChangeSupport prop) {
        this.prop = prop;
    }

    public void startElement(String tagName, Attributes atts) {
        String oldRaw = rawData;
        StringBuilder sb = new StringBuilder();
        sb.append("<");
        sb.append(tagName);
        //System.out.println("Start Element: " + tagName);
        //System.out.println("Attributes:");
        for (int i = 0; i < atts.getLength(); i++) {
            //System.out.println("\t" + atts.getQName(i) + " = " + atts.getValue(i));
            sb.append(" ");
            sb.append(atts.getQName(i) + "=\"" + atts.getValue(i)+ "\"");
        }
        sb.append(">");
        rawData = sb.toString();
        prop.firePropertyChange(SUBSCRIBE_RAWDATA, oldRaw, rawData);
    }

    public void endElement(String tagName) {
        String oldRaw = rawData;
        rawData = "</" + tagName + ">";
        prop.firePropertyChange(SUBSCRIBE_RAWDATA, oldRaw, rawData);
    }

    public void tagContents(String parentTag, char ch[], int start, int length) {
        String contents = new String(ch, start, length);
        prop.firePropertyChange(SUBSCRIBE_RAWDATA, "", contents);
    }
}
