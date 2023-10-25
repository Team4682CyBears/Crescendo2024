// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ZeroWristEncoder.java
// Intent: Zeros the wrist encoder.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.WristSubsystem;

public class ZeroWristEncoder extends CommandBase {

    private WristSubsystem wrist;

    public ZeroWristEncoder(WristSubsystem wristSubsystem) {
        this.wrist = wristSubsystem;
        addRequirements(wristSubsystem);  // This ensures no other command which uses the wrist can run simultaneously.
    }

    @Override
    public void initialize() {
        wrist.zeroWrist();
    }
}
