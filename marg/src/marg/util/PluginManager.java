/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package marg.util;

import java.io.File;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.JarURLConnection;
import java.net.URL;
import java.net.URLDecoder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Enumeration;
import java.util.List;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.logging.Level;
import java.util.logging.Logger;
import marg.gui.plugin.ModulePlugin;
import marg.model.plugin.XMLParsePlugin;

/**
 * Developed with help from http://forums.sun.com/thread.jspa?threadID=341935&start=15 to get clasess from a package inside a loaded jar
 * @author Daniel
 */
public class PluginManager {

    private static PluginManager instance;
    List<XMLParsePlugin> parsePlugins = new ArrayList<XMLParsePlugin>();
    List<Class> modulePluginClasses = new ArrayList<Class>();

    public static PluginManager getInstance() {
        if (instance == null)
            instance = new PluginManager();
        return instance;
    }

    private PluginManager() {
        //initParsePlugins(); TODO: Enable auto-detection of parseplugins
        initModulePlugins();
    }

    private void initModulePlugins() {
        modulePluginClasses = getClassessOfInterface("marg.gui.plugin", ModulePlugin.class);
    }

    private void initParsePlugins() {
        List<Class> pluginClasses = getClassessOfInterface("marg.model.plugin", XMLParsePlugin.class);
        createClassesAndFillIntoList(pluginClasses, parsePlugins);
    }

    private <T> void createClassesAndFillIntoList(List<Class> classes, List<T> container) {
        List<String> addedPlugins = new ArrayList<String>(); //Keeping track so we do not add the same plugin more than once.
        for (Class aClass : classes) {
            if (!addedPlugins.contains(aClass.getSimpleName())) {
                //System.out.println("Creating class: " + aClass);
                Object inst = getInstanceOfClass(aClass); //Refactored to get rid of try-catch clutter
                if (inst != null) {
                    container.add((T) inst);
                    addedPlugins.add(aClass.getSimpleName());
                }
            }
        }
    }

    public List<XMLParsePlugin> getParsePlugins() {
        return parsePlugins;
    }

    public List<Class> getAvailableModulePlugins() {
        return modulePluginClasses;
    }

    public Object getInstanceOfClass(Class aClass) {
        Object returnObject = null;
        try {
            returnObject = aClass.newInstance();
        } catch (InstantiationException ex) {
            Logger.getLogger(PluginManager.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IllegalAccessException ex) {
            Logger.getLogger(PluginManager.class.getName()).log(Level.SEVERE, null, ex);
        }
        return returnObject;
    }

    private static List<Class> getClassesForPackage(String pckgname)
            throws ClassNotFoundException {
        // This will hold a list of directories matching the pckgname.
        //There may be more than one if a package is split over multiple jars/paths
        List<Class> classes = new ArrayList<Class>();
        ArrayList<File> directories = new ArrayList<File>();
        try {
            ClassLoader cld = Thread.currentThread().getContextClassLoader();
            if (cld == null) {
                throw new ClassNotFoundException("Can't get class loader.");
            }
            // Ask for all resources for the path
            Enumeration<URL> resources = cld.getResources(pckgname.replace('.', '/'));
            while (resources.hasMoreElements()) {
                URL res = resources.nextElement();
                if (res.getProtocol().equalsIgnoreCase("jar")) {
                    JarURLConnection conn = (JarURLConnection) res.openConnection();
                    JarFile jar = conn.getJarFile();
                    for (JarEntry e : Collections.list(jar.entries())) {
                        if (e.getName().startsWith(pckgname.replace('.', '/')) && e.getName().endsWith(".class") && !e.getName().contains("$")) {
                            String className =
                                    e.getName().replace("/", ".").substring(0, e.getName().length() - 6);
                            classes.add(Class.forName(className));
                        }
                    }
                } else {
                    directories.add(new File(URLDecoder.decode(res.getPath(), "UTF-8")));
                }
            }
        } catch (NullPointerException x) {
            throw new ClassNotFoundException(pckgname + " does not appear to be " +
                    "a valid package (Null pointer exception)");
        } catch (UnsupportedEncodingException encex) {
            throw new ClassNotFoundException(pckgname + " does not appear to be " +
                    "a valid package (Unsupported encoding)");
        } catch (IOException ioex) {
            throw new ClassNotFoundException("IOException was thrown when trying " +
                    "to get all resources for " + pckgname);
        }

        // For every directory identified capture all the .class files
        for (File directory : directories) {
            if (directory.exists()) {
                // Get the list of the files contained in the package
                String[] files = directory.list();
                for (String file : files) {
                    // we are only interested in .class files
                    if (file.endsWith(".class")) {
                        // removes the .class extension
                        classes.add(Class.forName(pckgname + '.' + file.substring(0, file.length() - 6)));
                    }
                }
            } else {
                throw new ClassNotFoundException(pckgname + " (" + directory.getPath() +
                        ") does not appear to be a valid package");
            }
        }
        return classes;
    }

    private static List<Class> getClassessOfInterface(String thePackage, Class theInterface) {
        List<Class> classList = new ArrayList<Class>();
        try {
            for (Class discovered : getClassesForPackage(thePackage)) {
                if (Arrays.asList(discovered.getInterfaces()).contains(theInterface)) {
                    if (!classList.contains(discovered)) //We don't want duplicates of same implementation.
                        classList.add(discovered);
                }
            }
        } catch (ClassNotFoundException ex) {
            Logger.getLogger(PluginManager.class.getName()).log(Level.SEVERE, null, ex);
        }
        return classList;
    }

    public static void main(String[] args) {
        for (Class aClass : PluginManager.getInstance().getAvailableModulePlugins()) {
            System.out.println("Found Plugin: "+ aClass.getName());
        }
    }
}
