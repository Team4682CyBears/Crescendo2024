package frc.robot.control;

import frc.robot.subsystems.BagSubsystem;

public class SubsystemCollection {

    private BagSubsystem bagSubsystem = null;
    
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }
    public boolean isManualInputInterfacesAvailable() { return manualInput != null; }
 
    // bag subsystem
    // Uses bag motors, so using it for the 
    public BagSubsystem getBagSubsystem() { return bagSubsystem; }
    public void setBagSubsystem(BagSubsystem value) { bagSubsystem = value; }
    public boolean isBagSubsystemAvailable() { return bagSubsystem != null; } // 
}
