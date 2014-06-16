/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.gui;

import javax.swing.JTree;
import javax.swing.SwingUtilities;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeModel;
import marg.model.ModuleVariable;

/**
 *
 * @author Daniel
 */
public class VariableTree {

    private DefaultMutableTreeNode varTop;
    private JTree jTree;
    private String moduleName;

    public VariableTree(JTree jTree, String moduleName) {
        this.jTree = jTree;
        this.moduleName = moduleName;
    }

    public void updateVarTreeView() {
        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                jTree.updateUI();
            }
        });
    }

    public void updateVarTreeStructure(ModuleVariable var) {
        String shortName = "";
        boolean foundExistingLeaf = false;
        boolean isOnLeafNode = false;
        DefaultMutableTreeNode node;
        if (varTop == null) {
            varTop = new DefaultMutableTreeNode(moduleName); //Top node represents module
            jTree.setModel(new DefaultTreeModel(varTop));
        }
        if (var.getVarName() != null && var.getValue() != null && !var.getVarName().equals("")) {
            node = (DefaultMutableTreeNode) jTree.getModel().getRoot();
            String[] varStringTree = var.getVarName().split("\\.");
            shortName = varStringTree[varStringTree.length - 1];

            //Loop finds the node to attach the new leafnode (or updates exciting leaf node)
            for (int i = 0; i < varStringTree.length; i++) { //goes through each part of struct.struct.struct.leaf sequentially
                String sNode = varStringTree[i];
                if (i == varStringTree.length - 1) {
                    isOnLeafNode = true;
                }
                DefaultMutableTreeNode oldNode = node;
                for (int j = 0; j < node.getChildCount(); j++) {
                    DefaultMutableTreeNode aNode = (DefaultMutableTreeNode) node.getChildAt(j);
                    Object userObject = aNode.getUserObject();
                    if (userObject instanceof String) {
                        //Structure node, let's compare to our current sNode
                        if (((String) userObject).equals(sNode)) {
                            node = aNode;
                            break;
                        }
                    } else if (userObject instanceof ModuleVariable) {
                        if (((ModuleVariable) userObject).getShortVarName().equals(shortName) && isOnLeafNode) {
                            foundExistingLeaf = true;
                            node = aNode;
                            break;
                        }
                    }
                }
                if (oldNode == node && !isOnLeafNode) { //If no matches and not a leaf we make a struct node
                    DefaultMutableTreeNode newNode = new DefaultMutableTreeNode(sNode);
                    node.add(newNode);
                    node = newNode;
                }
            }
            if (foundExistingLeaf) {
                ModuleVariable moduleVar = (ModuleVariable) node.getUserObject();
                moduleVar.setValue(var.getValue());
            } else {
                DefaultMutableTreeNode leafNode = new DefaultMutableTreeNode(var); //Var has a toString that provides proper formatting
                node.add(leafNode);
            }
        }
    }

}
