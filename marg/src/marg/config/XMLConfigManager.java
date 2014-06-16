/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package marg.config;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

/**
 *
 * @author Daniel
 */
public class XMLConfigManager {

    private Logger log = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
    private Robot robot;

    public Document getXMLDocument(URL url) {
        Document doc = null;
        try {
            InputStream in = url.openConnection().getInputStream();
            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            DocumentBuilder builder = factory.newDocumentBuilder();
            doc = builder.parse(in);
        } catch (FileNotFoundException ex) {
            log.warning("File not found for url: " + url);
        } catch (SAXException ex) {
            Logger.getLogger(XMLConfigManager.class.getName()).log(Level.SEVERE, null, ex);
        } catch (ParserConfigurationException ex) {
            Logger.getLogger(XMLConfigManager.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IOException ex) {
            Logger.getLogger(XMLConfigManager.class.getName()).log(Level.SEVERE, null, ex);
        }
        return doc;
    }

    //TODO: I think this could use some refactoring perhaps? :D
    public Robot getConfigFromDocument(Document document) {
        robot = new Robot();
        if (document != null) {
            //<robot>
            NodeList robotElements = document.getElementsByTagName("robot");
            Element robotNode = (Element) robotElements.item(0);
            if (robotNode != null) {
                //<name> [1]
                NodeList nameNodes = robotNode.getElementsByTagName("name");
                if (nameNodes.getLength() > 0) {
                    Node nameNode = nameNodes.item(0).getChildNodes().item(0);
                    String name = getNodeValue(nameNode);
                    robot.setRobotName(name);
                }

                //<controlButtons> [0..1]
                NodeList ctrlBtnNodes = robotNode.getElementsByTagName("controlButtons");
                if (ctrlBtnNodes.getLength() > 0) {
                    Element ctrlBtnNode = (Element) ctrlBtnNodes.item(0);
                    Node startNode = ctrlBtnNode.getElementsByTagName("start").item(0);
                    Node pauseNode = ctrlBtnNode.getElementsByTagName("pause").item(0);
                    Node stopNode = ctrlBtnNode.getElementsByTagName("stop").item(0);

                    String startModule = getAttributeValueFor(startNode, "moduleName");
                    String startCmd = getAttributeValueFor(startNode, "command");
                    ControlButton startButton = new ControlButton(startModule, startCmd);
                    String pauseModule = getAttributeValueFor(pauseNode, "moduleName");
                    String pauseCmd = getAttributeValueFor(pauseNode, "command");
                    ControlButton pauseButton = new ControlButton(pauseModule, pauseCmd);
                    String stopModule = getAttributeValueFor(stopNode, "moduleName");
                    String stopCmd = getAttributeValueFor(stopNode, "command");
                    ControlButton stopButton = new ControlButton(stopModule, stopCmd);
                    robot.setStartButton(startButton);
                    robot.setPauseButton(pauseButton);
                    robot.setStopButton(stopButton);
                }

                //<module> [1..*]
                NodeList moduleNodes = robotNode.getElementsByTagName("module");
                for (int i = 0; i < moduleNodes.getLength(); i++) {
                    Element moduleNode = (Element) moduleNodes.item(i);
                    if (moduleNode != null) {
                        //<name> [1]
                        Node nameNode = moduleNode.getElementsByTagName("name").item(0).getChildNodes().item(0);
                        String name = getNodeValue(nameNode);
                        //<host> [0..1]
                        Node hostNode = moduleNode.getElementsByTagName("host").item(0).getChildNodes().item(0);
                        String host = getNodeValue(hostNode);
                        //<port> [0..1]
                        Node portNode = moduleNode.getElementsByTagName("port").item(0).getChildNodes().item(0);
                        String portString = getNodeValue(portNode);
                        Module newModule = null;
                        try {
                            if (name.equals("")) {
                                throw new IllegalArgumentException("XML Module " + (i + 1) + " had no name and could therefore not be added");
                            }
                            int port = 0;
                            try {
                                port = Integer.parseInt(portString);
                            } catch (NumberFormatException ex) {
                            }
                            newModule = new Module(name, host, port);
                        } catch (IllegalArgumentException ex) {
                            log.info(ex.getMessage());
                        }
                        if (newModule != null) {
                            //<plugin> [0..*]
                            NodeList pluginNodes = moduleNode.getElementsByTagName("plugin");
                            if (pluginNodes.getLength() > 0) {
                                for (int j = 0; j < pluginNodes.getLength(); j++) {
                                    Node pluginNode = pluginNodes.item(j).getChildNodes().item(0);
                                    String plugin = getNodeValue(pluginNode);
                                    newModule.addPlugin(plugin);
                                }
                            }

                            //<autoConnect> [0..1]
                            NodeList autoConnectNodes = moduleNode.getElementsByTagName("autoConnect");
                            if (autoConnectNodes.getLength() > 0) {
                                Node autoConnectNode = moduleNode.getElementsByTagName("autoConnect").item(0).getChildNodes().item(0);
                                String autoConnectString = getNodeValue(autoConnectNode);
                                boolean autoConnect = false;
                                if (autoConnectString.equalsIgnoreCase("true")) {
                                    autoConnect = true;
                                }
                                newModule.setAutoConnect(autoConnect);
                            }
                            //Done, now add it to the arraylist
                            robot.addModule(newModule);
                            log.info("Parsed module from XML: " + newModule);
                        }
                    }
                }
            }
        }
        return robot;
    }

    private String getAttributeValueFor(Node node, String attName) {
        NamedNodeMap atts = node.getAttributes();
        if (atts != null) {
            Node attribNode = atts.getNamedItem(attName);
            if (attribNode != null) {
                return getNodeValue(attribNode.getChildNodes().item(0));
            }
        }
        return "";
    }

    private String getNodeValue(Node node) {
        if (node != null) {
            return node.getNodeValue();
        } else {
            return "";
        }
    }

    public static void main(String[] args) throws Exception {
        XMLConfigManager config = new XMLConfigManager();
        //URL url = config.getClass().getClassLoader().getResource("robot.xml");
        URL url = new File("robot.xml").toURI().toURL();
        Document doc = config.getXMLDocument(url);
        config.getConfigFromDocument(doc);
    }
}
