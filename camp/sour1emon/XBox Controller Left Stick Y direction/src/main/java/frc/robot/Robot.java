// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.MotorDefaultCommand;
import frc.robot.subsystems.MotorSubsystem;

public class Robot extends TimedRobot {

  private static final int controllerPort = 0;

  private static final int motorPort = 1;

  private final MotorSubsystem motorSubsystem = new MotorSubsystem(motorPort);
  private final XboxController xboxController = new XboxController(controllerPort);

  @Override
  public void robotInit() {
    motorSubsystem.setDefaultCommand(new MotorDefaultCommand(motorSubsystem, xboxController));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
