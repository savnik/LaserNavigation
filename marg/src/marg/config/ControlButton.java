/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.config;

/**
 *
 * @author Daniel
 */
public class ControlButton {

    private String moduleName;
    private String command;

    public ControlButton(String moduleName, String command) {
        this.moduleName = moduleName;
        this.command = command;
    }

    public String getCommand() {
        return command;
    }

    public String getModuleName() {
        return moduleName;
    }
}
