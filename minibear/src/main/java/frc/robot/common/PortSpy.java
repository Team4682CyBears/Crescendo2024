// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: PortSpy.java
// Intent: Forms util class to contain PDP overcrrent watch stats.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Forms a util class to contain PDP overcurrent watch stats.
 */
public class PortSpy {
    private int portToWatch = -1;
    private double currentLimit = 0.0;
    private CommandBase action = null;
    private String description = "";
    private boolean enabled = true;

    /**
     * The constructor to assemble port watching metadata
     * @param port
     * @param limit
     * @param action
     * @param actionDescription
     */
    public PortSpy(int port, double limit, CommandBase action, String actionDescription)
    {
        this.portToWatch = port;
        this.currentLimit = limit;
        this.action = action;
        this.description = actionDescription;
    }

    /**
     * A method to set the port to be watched
     * @param port - port number
     */
    public void setPort(int port) {
        this.portToWatch = port;
    }

    /**
     * A method to get the port being watched
     * @return the port number
     */
    public int getPort() {
        return this.portToWatch;
    }

    /** 
     * A method to set the current limit for the port
     * @param limit - current limit in miliamps
     */
    public void setCurrentLimit(Double limit) {
        this.currentLimit = limit;
    }

    /**
     * A method to get the current limit
     * @return the current limit
     */
    public double getCurrentLimit() {
        return this.currentLimit;
    }

    /**
     * A method to set the action to take when the limit is exceeded
     * @param action - the command to execute
     */
    public void setAction(CommandBase action) {
        this.action = action;
    }

    /**
     * A method to get the configured action 
     * @return
     */
    public CommandBase getAction() {
        return this.action;
    }

    /**
     * A method to set the description printed when the action is taken. 
     * @param actionDescription
     */
    public void setActionDescription(String actionDescription) {
        this.description = actionDescription;
    }

    /**
     * A method to get the description to be printed when the action is taken
     * @return
     */
    public String getActionDescription() {
        return this.description;
    }

    /**
     * A method to get the enabled status of the port watcher
     * @return
     */
    public boolean getEnabled() { 
        return enabled; 
    }
    
    /**
     * A method to enable/disable state of the port watcher
     * @param value - true when enabled
     */
    public void setEnabled(boolean value) { 
        this.enabled = value; 
    }

}