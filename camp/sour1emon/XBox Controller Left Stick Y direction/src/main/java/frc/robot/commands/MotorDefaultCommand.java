package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

public class MotorDefaultCommand extends Command {
    private XboxController controller;
    private MotorSubsystem motorSubsystem;

    public MotorDefaultCommand(MotorSubsystem motorSubsystem, XboxController controller) {
        this.motorSubsystem = motorSubsystem;
        this.controller = controller;
        addRequirements(motorSubsystem);
    }

    @Override
    public void execute() {
        motorSubsystem.setSpeed(controller.getLeftY());
    }
}
