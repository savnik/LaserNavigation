/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.model;

import marg.util.Log;

/**
 *
 * @author Daniel
 */
public class StatusVariable {

    private ModuleVariable var;
    private ValueLimit limit;

    public StatusVariable(ModuleVariable var) {
        this.var = var;
    }

    public StatusVariable(ModuleVariable var, ValueLimit limit) {
        this.var = var;
        this.limit = limit;
    }

    public void setLimit(ValueLimit limit) {
        this.limit = limit;
    }

    public ModuleVariable getModuleVar() {
        return var;
    }

    public boolean isStatusCorrect() {
        if (limit != null && var.getVarType().equals("d")) { //d represented as Double
            try {
                Double value = Double.parseDouble(var.getValue());
                return limit.checkCorrect(value);
            } catch (NumberFormatException ex) {
                Log.GlobalLogger.info("Could not parse variable with type (d)\n"+ ex);
                return false;
            }
        } else
            return true;
    }

    public ValueLimit getLimit() {
        return limit;
    }

    @Override
    public String toString() {
        String out = "";
        if (isStatusCorrect()) {
            out = var.getVarName()+ " ("+ var.getVarType() +")"+ " = "+ var.getValue();
        } else {
            out = var.getVarName()+ " ("+ var.getVarType() +")"+ " = "+ var.getValue() +" (Outside Value Limit)";
        }
        return out;
    }
}
