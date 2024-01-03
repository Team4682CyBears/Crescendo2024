// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DefaultArmCommand.java
// Intent: Forms a command to move the dual part arm with y and z inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

/*
 * Forms a command to move the dual part arm with y and z motor inputs.  
 * Y motor input for the extension/horizontal arm.
 * Z motor input for the angle/vertical arm.
 */
public class DefaultArmCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier zSupplier;
    private final double inputThreshold = 0.2;

    /**
     * The constructor to create a default command for the arm subsystem.
     * @param theArmSubsystem - the arm subsystem
     * @param yMotionSupplier - Y motor input for the extension/horizontal arm where negative is retract, postitive is extend.
     * @param zMotionSupplier - Z motor input for the angle/vertical arm where negative is retract, postitive is extend.
     */
    public DefaultArmCommand(ArmSubsystem theArmSubsystem,
                             DoubleSupplier yMotionSupplier,
                             DoubleSupplier zMotionSupplier) {
        this.armSubsystem = theArmSubsystem;
        this.ySupplier = yMotionSupplier;
        this.zSupplier = zMotionSupplier;
        addRequirements(this.armSubsystem);
    }

    @Override
    public void execute() {
        double yValue = ySupplier.getAsDouble();
        double zValue = zSupplier.getAsDouble();
        if(Math.abs(yValue) < this.inputThreshold) {
            yValue = 0.0;
        }
        if(Math.abs(zValue) < this.inputThreshold) {
            zValue = 0.0;
        }
        this.armSubsystem.setArmSpeeds(yValue, zValue);
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.setArmSpeeds(0.0, 0.0);
    }
}