/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package marg.config;

import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Daniel
 */
public class Robot {

    private List<Module> modules = new ArrayList<Module>();
    private ControlButton startButton;
    private ControlButton pauseButton;
    private ControlButton stopButton;
    private String robotName;

    public Robot() {
    }

    public boolean removeModule(Module module) {
        return modules.remove(module);
    }

    public boolean containsModule(Module module) {
        return modules.contains(module);
    }

    public boolean addModule(Module module) {
        return modules.add(module);
    }

    public List<Module> getModules() {
        return modules;
    }

    public ControlButton getPauseButton() {
        return pauseButton;
    }

    public void setPauseButton(ControlButton pauseButton) {
        this.pauseButton = pauseButton;
    }

    public ControlButton getStartButton() {
        return startButton;
    }

    public void setStartButton(ControlButton startButton) {
        this.startButton = startButton;
    }

    public ControlButton getStopButton() {
        return stopButton;
    }

    public void setStopButton(ControlButton stopButton) {
        this.stopButton = stopButton;
    }

    public void setRobotName(String robotName) {
        this.robotName = robotName;
    }

    public String getRobotName() {
        return robotName;
    }



}
