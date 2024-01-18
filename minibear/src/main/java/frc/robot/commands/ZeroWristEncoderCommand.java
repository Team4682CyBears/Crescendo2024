// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ZeroWristEncoderCommand.java
// Intent: Zeros the wrist encoder.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

/**
 * Forms a command to zero the wrist. 
 */
public class ZeroWristEncoderCommand extends CommandBase {

    private WristSubsystem wrist;

    public ZeroWristEncoderCommand(WristSubsystem wristSubsystem) {
        this.wrist = wristSubsystem;
        addRequirements(wristSubsystem);  // This ensures no other command which uses the wrist can run simultaneously.
    }

    /**
     * this method is called when the command is created. 
     */
    @Override
    public void initialize() {
        wrist.zeroWrist();
    }
}
