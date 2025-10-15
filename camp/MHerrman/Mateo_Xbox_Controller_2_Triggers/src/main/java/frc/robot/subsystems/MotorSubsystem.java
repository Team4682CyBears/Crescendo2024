package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotorSubsystem extends SubsystemBase{

    TalonFX motor = new TalonFX(1);

    public MotorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; // In amps
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.StatorCurrentLimit = 40.0; // In amps

        config.CurrentLimits = currentLimits;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }
}
