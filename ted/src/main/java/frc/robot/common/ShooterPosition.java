// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: public enum ShooterPosition.java
// Intent: Forms enum to specify the ShooterPositions.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

public enum ShooterPosition{
    Stow, // stow is the position we want when transiting. Something low enough to drive under the stage.
    Minimum, // fully down at lowest position
    Medium,
    Maximum // up at highest position 
}