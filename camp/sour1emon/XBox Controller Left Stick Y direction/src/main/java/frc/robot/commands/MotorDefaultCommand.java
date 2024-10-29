package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

/**
 * A command that sets the speed of a motor subsystem based on the Y direction
 * of the left stick of an Xbox controller.
 */
public class MotorDefaultCommand extends Command {
    private XboxController controller;
    private MotorSubsystem motorSubsystem;

    /**
     * Constructs a MotorDefaultCommand.
     * 
     * @param motorSubsystem the motor subsystem to control
     * @param controller     the Xbox controller to read input from
     */
    public MotorDefaultCommand(MotorSubsystem motorSubsystem, XboxController controller) {
        this.motorSubsystem = motorSubsystem;
        this.controller = controller;
        addRequirements(motorSubsystem);
    }

    /**
     * Executes the command by setting the speed of the motor subsystem based on the
     * left Y direction of the controller.
     */
    @Override
    public void execute() {
        motorSubsystem.setSpeed(controller.getLeftY());
    }
}
