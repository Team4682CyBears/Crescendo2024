// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class matthewsCoolerMotor extends SubsystemBase {
  //documentation https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax#set(double)
  //https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
  final CANSparkMax Motor = new CANSparkMax(Constants.canPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public matthewsCoolerMotor() {}

  public void YStickMovement(double speed) {
        Motor.set(speed);
        System.out.println(speed);
        System.out.println(Motor.get());
    }
    
  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
