package marg.model;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.ConnectException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.logging.Level;
import java.util.logging.Logger;

public abstract class AbstractModuleClient implements ModuleClient, Runnable {

    private static final int SOCKET_CONNECT_TIMEOUT = 1500;
    Socket socket;
    volatile boolean threadRunning = false;
    volatile boolean connected = false;
    PrintWriter out;

    public void disconnect() {
        try {
            connected = false;
            threadRunning = false;
            closeInputStream();
            if (socket != null) {
                socket.close();
            }
        } catch (IOException ex) {
            Logger.getLogger(AbstractModuleClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public boolean connect(String host, int port) {
        try {
            if (port < 1024 || port > 65535)
                throw new IllegalArgumentException();
            socket = new Socket();
            socket.bind(null);
            socket.connect(new InetSocketAddress(host, port), SOCKET_CONNECT_TIMEOUT);
            out = new PrintWriter(socket.getOutputStream());
            openInputStream();
            connected = true;
            if (!threadRunning) {
                startUpdateThread();
            }
            return true;
        } catch (ConnectException ex) {
            marg.util.Log.GlobalLogger.warning("Failed to connect to: "+ host +":"+ port);
        } catch (UnknownHostException ex) {
            marg.util.Log.GlobalLogger.warning("Could not resolve given host: "+ ex.getMessage());
        } catch (IOException ex) {
            Logger.getLogger(AbstractModuleClient.class.getName()).log(Level.SEVERE, null, ex);
        }
        return false;
    }

    public void sendCmd(String cmd) {
        if (out != null) {
            out.println(cmd);
        }
    }

    private void startUpdateThread() {
        threadRunning = true;
        Thread t = new Thread(this);
        t.start();
    }

    public boolean isConnected() {
        return connected;
    }

    public abstract void run();

    abstract void openInputStream();
    abstract void closeInputStream();
}