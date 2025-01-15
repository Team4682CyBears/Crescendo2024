// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Declare what package this class belongs to
package frc.robot;

// Import robot framework class (RobotBase)
import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}
  /**
   * Main initialization function.
   * Left the same on the main robot code
   */

   // Automatically called by RoboRio
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
