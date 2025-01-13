package frc.robot.control;

import frc.robot.subsystems.FeederSubsystem;

public class SubsystemCollection {

    private FeederSubsystem feederSubsystem = null;
    
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }
    public boolean isManualInputInterfacesAvailable() { return manualInput != null; }
 
    // Feeder subsystem
    // Uses bag motors, so using it for the 
    public FeederSubsystem getFeederSubsystem() { return feederSubsystem; }
    public void setFeederSubsystem(FeederSubsystem value) { feederSubsystem = value; }
    public boolean isFeederSubsystemAvailable() { return feederSubsystem != null; }
}
