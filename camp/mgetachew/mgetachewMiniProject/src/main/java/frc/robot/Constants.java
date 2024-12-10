// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix6.signals.InvertedValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double shooterOutfeedStatorCurrentMaximumAmps = 100.0;
    public static final double shooterOutfeedSupplyCurrentMaximumAmps = 50.0;
    public static final double shooterOutfeedSupplyVoltageTimeConstant = 0.02;
    //Motor Talon Port
    public static final int motorTalonPort = 3;
    //Change to fit your prefered xbox controller port
    public static final int xboxControllerPort = 0;
    //Could be related to the bug.
    public static InvertedValue rightTalonShooterMotorDefaultDirection = InvertedValue.CounterClockwise_Positive;  }
}
