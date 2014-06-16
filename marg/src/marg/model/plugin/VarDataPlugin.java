/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.model.plugin;

import java.beans.PropertyChangeSupport;
import marg.model.ModuleVariable;
import org.xml.sax.Attributes;

/**
 *
 * @author Daniel
 */
public class VarDataPlugin implements XMLParsePlugin {

    public final static String SUBSCRIBE_VARDATA = "varData";
    private PropertyChangeSupport prop;
    private ModuleVariable lastVariable;

    public VarDataPlugin() {
    }

    public void setPropertyChangeSupport(PropertyChangeSupport prop) {
        this.prop = prop;
    }

    public void startElement(String tagName, Attributes atts) {
        if (tagName.equals("var") && atts.getLength() > 2) {
            ModuleVariable oldVar = lastVariable;
            String varName = atts.getValue("name");
            String varType = atts.getValue("typ");
            String varValue = atts.getValue("value");
            ModuleVariable newVar = new ModuleVariable(varName, varType, varValue);
            lastVariable = newVar;
            prop.firePropertyChange(SUBSCRIBE_VARDATA, oldVar, newVar);
        }
    }

    public void endElement(String tagName) {
    }

    public void tagContents(String parentTag, char ch[], int start, int length) {
    }

}
