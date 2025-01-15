// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SubsystemCollection.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// declare package to contain class
package frc.robot.control;

// import local class
import frc.robot.subsystems.BagSubsystem;

public class SubsystemCollection {
   
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems  
    private BagSubsystem bagSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {}

    // getters/setters for ManualInputInterfaces
    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }
    public boolean isManualInputInterfacesAvailable() { return manualInput != null; }
 
    // bag subsystem
    // Uses bag motors, so using it for the 
    public BagSubsystem getBagSubsystem() { return bagSubsystem; }
    public void setBagSubsystem(BagSubsystem value) { bagSubsystem = value; }
    public boolean isBagSubsystemAvailable() { return bagSubsystem != null; } // 
}
