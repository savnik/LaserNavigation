package marg.model;

import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import marg.util.Log;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;
import org.xml.sax.XMLReader;
import org.xml.sax.helpers.XMLReaderFactory;

public class XMLClient extends AbstractModuleClient {

    private XMLReader xr;
    private XMLParser xp;
    private InputSource in;
    private static final String NAMESPACE = "marg";

    public XMLClient() {
        try {
            xr = XMLReaderFactory.createXMLReader();
            xp = new XMLParser();
            xr.setContentHandler(xp);
        } catch (SAXException ex) {
            Logger.getLogger(XMLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    @Override
    public void run() {
        Log.GlobalLogger.fine("Run Thread Started");
        while (threadRunning) {
            try {
                if (connected) {
                    Log.GlobalLogger.info("Socket is connected, parsing XML");
                    xr.parse(in);
                } else {
                    Thread.sleep(200);
                }
            } catch (IOException ex) {
                //Logger.getLogger(XMLClient.class.getName()).log(Level.SEVERE, null, ex);
                System.err.println("IOException, Socket closed while parsing");
                connected = false;
            } catch (SAXException ex) {
                Logger.getLogger(XMLClient.class.getName()).log(Level.SEVERE, null, ex);
                System.out.println("SAX xr.parse exit");
                connected = false;
            } catch (InterruptedException ex) {
                Logger.getLogger(XMLClient.class.getName()).log(Level.SEVERE, null, ex);
                System.out.println("Interrupted while sleeping");
            }
        }
    }

    @Override
    public void sendCmd(String cmd) {
        if (out != null) {
            String xmlCmd = xmlFormatCmd(cmd);
            Log.GlobalLogger.info("Sending cmd: " + xmlCmd);
            out.println(xmlCmd);
            out.flush();
        }
    }

    public String xmlFormatCmd(String cmd) {
        return "<" + cmd + "/>";
    }

    @Override
    void openInputStream() {
        try {
            in = new InputSource(socket.getInputStream());
            //Declare XML Language and namespace
            out.println("<?xml version=\"1.0\" encoding=\"UTF-8\" ?>");
            out.println("<" + NAMESPACE + " name=\"XMLClient\" version=\"1.0\">");
        } catch (IOException ex) {
            Logger.getLogger(XMLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    @Override
    void closeInputStream() {
        in = null;
        //Close namespace
        if (out != null) {
            out.println("</" + NAMESPACE + ">");
        }
    }

    public XMLParser getXMLParser() {
        return xp;
    }
}

