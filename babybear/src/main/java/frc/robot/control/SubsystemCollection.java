// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: Subsystem.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import frc.robot.subsystems.*;

public class SubsystemCollection {
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems
    private DrivetrainSubsystem driveTrainSubsystem = null;
    private DrivetrainPowerSubsystem driveTrainPowerSubsystem = null;
    private EveryBotPickerSubsystem everyBotPickerSubsystem = null;
    private PowerDistributionPanelWatcherSubsystem powerDistributionPanelWatcherSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {
    }

    public DrivetrainSubsystem getDriveTrainSubsystem() {
        return driveTrainSubsystem;
    }

    public void setDriveTrainSubsystem(DrivetrainSubsystem value) {
        driveTrainSubsystem = value;
    }

    public DrivetrainPowerSubsystem getDriveTrainPowerSubsystem() {
        return driveTrainPowerSubsystem;
    }

    public void setDriveTrainPowerSubsystem(DrivetrainPowerSubsystem value) {
        driveTrainPowerSubsystem = value;
    }

    public EveryBotPickerSubsystem getEveryBotPickerSubsystem() {
        return everyBotPickerSubsystem;
    }

    public void setEveryBotPickerSubsystem(EveryBotPickerSubsystem value) {
        everyBotPickerSubsystem = value;
    }

    public PowerDistributionPanelWatcherSubsystem getPowerDistributionPanelWatcherSubsystem() {
        return powerDistributionPanelWatcherSubsystem;
    }

    public void setPowerDistributionPanelWatcherSubsystem(PowerDistributionPanelWatcherSubsystem value) {
        powerDistributionPanelWatcherSubsystem = value;
    }

    public ManualInputInterfaces getManualInputInterfaces() {
        return manualInput;
    }

    public void setManualInputInterfaces(ManualInputInterfaces value) {
        manualInput = value;
    }

}
