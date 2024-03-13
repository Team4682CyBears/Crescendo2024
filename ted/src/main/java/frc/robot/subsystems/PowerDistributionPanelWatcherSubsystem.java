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
import edu.wpi.first.wpilibj.PowerDistribution;
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

    @Override
    public void periodic() {
        for (int counter = 0; counter < myList.size(); counter++) {
            PortSpy nextSpy = myList.get(counter);
            double current = distroPannel.getCurrent(nextSpy.getPort());

            if(nextSpy.getEnabled() && current > nextSpy.getCurrentLimit())
            {
                System.out.printf(
                    "Overcurrent detected for port %1$d with maximum of %2$f and actual of %3$f. -> %4$s\n", 
                    nextSpy.getPort(), nextSpy.getCurrentLimit(), current, nextSpy.getActionDescription());
                // lanunch the command
                CommandScheduler.getInstance().schedule(nextSpy.getAction());
            }
            SmartDashboard.putNumber(nextSpy.getActionDescription(), current);
        }
    }
}
