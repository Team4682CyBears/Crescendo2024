// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// File: SubsystemCollection.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;
import frc.robot.subsystems.*;

public class SubsystemCollection
{
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems  
    private ControlledFalcon falconSubsystem = null;


    public SubsystemCollection() {}

    public ControlledFalcon getFalconSubsystem() { return falconSubsystem; }
    public void setFalconSubsystem(ControlledFalcon value) { falconSubsystem = value; }

    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }

}
