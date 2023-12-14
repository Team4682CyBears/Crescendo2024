// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.matthewsCoolerMotor;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class YstickAxis extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final matthewsCoolerMotor subsystem;
  private DoubleSupplier yAxisController;
  

  /**
   * Creates a new leftbutton.
   *
   * @param subsystem The subsystem used by this command.
   */
  public YstickAxis(matthewsCoolerMotor movingMotor, DoubleSupplier yAxisController) {
    subsystem = movingMotor;
    this.yAxisController = yAxisController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(movingMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.YStickMovement(yAxisController.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
