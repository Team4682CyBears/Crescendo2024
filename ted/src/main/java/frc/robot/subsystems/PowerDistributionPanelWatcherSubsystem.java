// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: PowerDistributionPanelWatcherSubsystem.java
// Intent: Forms util class to watch PDP ports for overcrrent.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.PortSpy;
import frc.robot.control.Constants;

public class PowerDistributionPanelWatcherSubsystem extends SubsystemBase {
    private PowerDistribution distroPannel = new PowerDistribution(
        Constants.currentPowerDistributionPanelCanId,
        Constants.currentPowerDistributionPanelType);
    private ArrayList<PortSpy> myList = new ArrayList<PortSpy>();

    private int brownoutEventCount = 0;
    private Runnable brownoutAction = null;
    private int brownoutEventsPerAction;
    private boolean handleBrownouts = false;

    public PowerDistributionPanelWatcherSubsystem() {
        /* 
        System.out.println("CTOR PowerDistributionPanelWatcherSubsystem");
        for(var next: Thread.currentThread().getStackTrace()) {
            System.out.println(next.toString());
        }
        */
//        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /*
     * Method to add new ports to watch for overcurrent protection on
     * @param spy
     */
    public void add(PortSpy spy) {
        myList.add(spy);
    }

    /**
     * Get the power distro
     * @return the power distro object
     */
    public PowerDistribution getPowerDistribution() { return distroPannel; }

    /**
     * enable or disable a port that is currently under watch
     * @param targetPort - the port we want to make sure is disabled 
     * @param enabledValue - if true the port will be disabled, else if false it will be enabled
     */
    public void setEnabledWatchOnPort(int targetPort, boolean enabledValue) {
        for (int counter = 0; counter < myList.size(); counter++) {
            PortSpy nextSpy = myList.get(counter);
            if(nextSpy.getPort() == targetPort) {
                nextSpy.setEnabled(enabledValue);
                break;
            }
        }
    }

    /**
     * Sets the callback action to run every brownoutEventsPerAction times a brownout happens
     * @param brownoutAction
     * @param brownoutEventsPerAction
     */
    public void setBrownoutCallback(Runnable brownoutAction, int brownoutEventsPerAction){
        this.brownoutAction = brownoutAction;
        this.brownoutEventsPerAction = brownoutEventsPerAction;
        this.handleBrownouts = true;
        this.brownoutEventCount = 0; 
    }

    @Override
    public void periodic() {
        // handle brownouts
        handleBrownouts();
        SmartDashboard.putNumber("Brownout Count", this.brownoutEventCount);
        // handle port spies
        for (int counter = 0; counter < myList.size(); counter++) {
            PortSpy nextSpy = myList.get(counter);
            double current = distroPannel.getCurrent(nextSpy.getPort());

            if(nextSpy.getEnabled() && current > nextSpy.getCurrentLimit())
            {
                System.out.println(
                    "Overcurrent detected for port " + nextSpy.getPort() +
                    " with maximum of " + nextSpy.getCurrentLimit() + 
                    " and actual of " + current + 
                    ". -> " + nextSpy.getActionDescription());
                // lanunch the command
                CommandScheduler.getInstance().schedule(nextSpy.getAction());
            }
            SmartDashboard.putNumber(nextSpy.getActionDescription(), current);
        }
    }

    /**
     * A method to check for and handle brownout events
     * runs the brownoutAction every brownoutEventsPerAction times a brownout event is detected.
     */
    private void handleBrownouts(){
        if (isNearBrownout()){
            brownoutEventCount += 1;
            if (this.handleBrownouts && (brownoutEventCount % this.brownoutEventsPerAction == 0)){
                brownoutAction.run();
            }
        }
    }

    /**
     * A Method to check for brownouts. 
     * The PDP voltage is compared against the brownout voltage + specified voltage margin
     * @return true when a near brownout is detected
     */
    private boolean isNearBrownout(){
        // this way seemed to way over-count brownouts with a safety margin of 0.5V. Maybe need to decrease margin to ~0.1V?
        // return distroPannel.getVoltage() < PowerJNI.getBrownoutVoltage() + Constants.bownoutVoltageSafetyMarginVolts;
        // Trying this way instead:
        return RobotController.isBrownedOut();
    }

}
