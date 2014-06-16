package marg.model;


public interface ModuleClient {

    public void disconnect();

    public boolean connect(String host, int port);

    public void sendCmd(String cmd);
}

