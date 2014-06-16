/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.model;

import java.util.IllegalFormatException;

/**
 *
 * @author Daniel
 */
public class MonitoredVariable {

    private ModuleVariable moduleVar;
    private String presentation;

    public MonitoredVariable(ModuleVariable var, String presentation) {
        this.moduleVar = var;
        this.presentation = presentation;
    }

    public String getPresentation() {
        return presentation;
    }

    public ModuleVariable getModuleVar() {
        return moduleVar;
    }

    public void setPresentation(String presentation) {
        this.presentation = presentation;
    }

    @Override
    public String toString() {
        String presentedModVar = "!! Formatting Failed !!";
        try {
            presentedModVar = String.format(presentation, moduleVar.getValue());
        } catch (IllegalFormatException e) {
            System.out.println("Formatting of MonitoredVariable failed");
        }
        return presentedModVar;
    }
}
