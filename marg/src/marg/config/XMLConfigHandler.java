/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package marg.config;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.logging.Level;
import java.util.logging.Logger;
import marg.gui.MargGUI;

/**
 *
 * @author Daniel
 */
public class XMLConfigHandler {

    private XMLConfigManager configMan;

    public XMLConfigHandler() {
        configMan = new XMLConfigManager();
    }

    public Robot getConfigFrom(URL url) {
        org.w3c.dom.Document doc = configMan.getXMLDocument(url);
        Robot loadedRobot = configMan.getConfigFromDocument(doc);
        return loadedRobot;
    }

    public static boolean isLocalURL(URL url) {
        if (url.getProtocol().equals("file"))
            return true;
        else
            return false;
    }

    public static URL getFileURL(URL baseURL, String fileName) {
        try {
            URL fileURL = new URL(baseURL, fileName);
            return fileURL;
        } catch (MalformedURLException ex) {
            Logger.getLogger(XMLConfigHandler.class.getName()).log(Level.SEVERE, null, ex);
            return baseURL;
        }
    }

    public static URL fixLocalUrl(URL url) {
        String protocol = url.getProtocol();
        if (protocol.equals("file")) {
            String path = url.getPath();
            try {
                int buildOffset = path.indexOf("build");
                int distOffset = path.indexOf("dist");
                if (buildOffset != -1) { //We have a build-folder match
                    url = new URL(protocol +":"+ path.substring(0, buildOffset));
                } else if (distOffset != -1) { //we have a dist-folder match;
                    url = new URL(protocol +":"+ path.substring(0, distOffset));
                }
            } catch (MalformedURLException ex) {
                Logger.getLogger(MargGUI.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
        System.out.println("Fixed Local URL: " + url);
        return url;
    }
}
