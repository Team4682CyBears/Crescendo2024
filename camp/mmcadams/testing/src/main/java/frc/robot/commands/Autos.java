// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.NeoShooterSubsystem;
import frc.robot.subsystems.TalonShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  public static CommandBase talonAuto(TalonShooterSubsystem subsystem) {
    return Commands.sequence(new TalonShootAtSpeedCommand(subsystem));
  }

  public static CommandBase neoAuto(NeoShooterSubsystem subsystem) {
    return Commands.sequence(new NeoShootAtSpeedCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
