/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.config;

import java.util.ArrayList;

/**
 *
 * @author Daniel
 */
public class Module {

    public Module(String name, String host, int port) {
            this.port = port;
            this.host = host;
            this.name = name;
        }
        private ArrayList<String> modulePlugins = new ArrayList<String>();

        public void addPlugin(String name) {
            if (name != null && !name.equals("")) {
                modulePlugins.add(name);
            }
        }

        public void removePlugin(String name) {
            modulePlugins.remove(name);
        }

        public ArrayList<String> getModulePlugins() {
            return modulePlugins;
        }
        private boolean autoConnect = false;

        public boolean isAutoConnect() {
            return autoConnect;
        }

        public void setAutoConnect(boolean autoConnect) {
            this.autoConnect = autoConnect;
        }
        private int port = 0;

        /**
         * Get the value of port
         *
         * @return the value of port
         */
        public int getPort() {
            return port;
        }

        /**
         * Set the value of port
         *
         * @param port new value of port
         */
        public void setPort(int port) {
            this.port = port;
        }
        private String host = "";

        /**
         * Get the value of moduleHost
         *
         * @return the value of moduleHost
         */
        public String getModuleHost() {
            return host;
        }

        /**
         * Set the value of moduleHost
         *
         * @param moduleHost new value of moduleHost
         */
        public void setModuleHost(String moduleHost) {
            this.host = moduleHost;
        }
        private String name = "";

        /**
         * Get the value of moduleName
         *
         * @return the value of moduleName
         */
        public String getModuleName() {
            return name;
        }

        /**
         * Set the value of moduleName
         *
         * @param moduleName new value of moduleName
         */
        public void setModuleName(String moduleName) {
            this.name = moduleName;
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            sb.append(name + ", " + host + ":" + port);
            sb.append(", Auto-Connect: " + autoConnect);
            for (String s : modulePlugins) {
                sb.append(", Plugin: " + s);
            }
            return sb.toString();
        }

}
