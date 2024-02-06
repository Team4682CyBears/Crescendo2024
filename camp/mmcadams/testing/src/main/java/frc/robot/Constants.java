// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int kDriverControllerPort = 0;

  public static final double talonMaximumRevolutionsPerMinute = 6380;
  public static final double CtreTalonFx500EncoderTicksPerRevolution = 2048; 

  public static final int leftTalonShooterMotorCanId = 7;
  public static TalonFXInvertType leftTalonShooterMotorDefaultDirection = TalonFXInvertType.Clockwise;  
  public static final int rightTalonShooterMotorCanId = 8;
  public static TalonFXInvertType rightTalonShooterMotorDefaultDirection = TalonFXInvertType.CounterClockwise;  

  public static final double neoMaximumRevolutionsPerMinute = 5676;
  public static final double neoEncoderTicksPerRevolution = 42;
  public static final int rightNeoOneShooterMotorCanId = 4;
  public static boolean rightNeoOneShooterMotorDefaultDirectionClockwise = true;  
  public static final int rightNeoTwoShooterMotorCanId = 5;
  public static boolean rightNeoTwoShooterMotorDefaultDirectionClockwise = true;  
  public static final int leftNeoOneShooterMotorCanId = 6;
  public static boolean leftNeoOneShooterMotorDefaultDirectionClockwise = false;  
  public static final int leftNeoTwoShooterMotorCanId = 7;
  public static boolean leftNeoTwoShooterMotorDefaultDirectionClockwise = false;  

}
