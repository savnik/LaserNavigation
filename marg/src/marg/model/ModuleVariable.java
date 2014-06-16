/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.model;

/**
 *
 * @author Daniel
 */
public class ModuleVariable {

    private String varName;
    private String varType;
    private String value;

    public ModuleVariable(String varName, String varType, String value) {
        this.varName = varName;
        this.varType = varType;
        this.value = value;
    }

    public String getValue() {
        return value;
    }

    public String getVarName() {
        return varName;
    }

    public String getShortVarName() {
        int offset = varName.lastIndexOf('.');
        return varName.substring(offset+1);
    }

    public String getVarType() {
        return varType;
    }

    public void setValue(String value) {
        this.value = value;
    }

    public String toString() {
        return "("+ varType +") "+ getShortVarName() +" = "+ value;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final ModuleVariable other = (ModuleVariable) obj;
        if ((this.varName == null) ? (other.varName != null) : !this.varName.equals(other.varName)) {
            return false;
        }
        if ((this.varType == null) ? (other.varType != null) : !this.varType.equals(other.varType)) {
            return false;
        }
        if (this.value != other.value && (this.value == null || !this.value.equals(other.value))) {
            return false;
        }
        return true;
    }

}
