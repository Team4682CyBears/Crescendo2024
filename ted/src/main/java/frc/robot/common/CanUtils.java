package frc.robot.common;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.control.Constants;

/**
 * A class intending to help keep CAN usage tidy
 */
public class CanUtils {

    /**
     * A method to update the TalonFX motor frequency to team-wide approved setting
     * @param motor - the TalonFX motor (usually not a follower motor)
     */
    public static void UpdateCtreFrequency(TalonFX motor) {
        double currentFrequency = motor.getPosition().getAppliedUpdateFrequency();
        if(currentFrequency < Constants.ctreMotorStatusFramePeriodFrequencyHertz) {
            motor.getPosition().setUpdateFrequency(Constants.ctreMotorStatusFramePeriodFrequencyHertz);
            System.out.println(
                "Updating frequency for TalonFX # " +
                motor.getDeviceID() +
                " from " + currentFrequency +
                " to " + Constants.ctreMotorStatusFramePeriodFrequencyHertz);
        }
        else {
            System.out.println(
                "No frequency change for TalonFX # " +
                motor.getDeviceID() +
                " existing frequency == " + currentFrequency);
        }
    }

    /**
     * A method to update the TalonFX motor frequency to team-wide approved setting
     * @param motor - the TalonFX motor (usually not a follower motor)
     */
    public static void UpdateCtreFrequency(TalonSRX motor) {
        int currentPeriod = motor.getStatusFramePeriod(StatusFrameEnhanced.Status_1_General);
        double currentFrequency = 1000.0/(double)currentPeriod;
        if(currentFrequency < Constants.ctreMotorStatusFramePeriodFrequencyHertz) {
            int targetPeriod = (int)(1000.0/Constants.ctreMotorStatusFramePeriodFrequencyHertz);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, targetPeriod);
            System.out.println(
                "Updating period for TalonSRX # " +
                motor.getDeviceID() +
                " from " + currentPeriod +
                " to " + targetPeriod);
        }
        else {
            System.out.println(
                "No period change for TalonSRX # " +
                motor.getDeviceID() +
                " existing period == " + currentPeriod);
        }
    }

    /**
     * A method to update the CANcoder sensor frequency to team-wide approved setting
     * @param canCoder - the CANcoder sensor
     */
    public static void UpdateCtreFrequency(CANcoder canCoder) {
        double currentFrequency = canCoder.getPosition().getAppliedUpdateFrequency();
        if(currentFrequency < Constants.ctreSensorStatusFramePeriodFrequencyHertz) {
            canCoder.getPosition().setUpdateFrequency(Constants.ctreSensorStatusFramePeriodFrequencyHertz);
            System.out.println(
                "Updating frequency for CANCoder # " +
                canCoder.getDeviceID() +
                " from " + currentFrequency +
                " to " + Constants.ctreMotorStatusFramePeriodFrequencyHertz);
        }
        else {
            System.out.println(
                "No frequency change for CANCoder # " +
                canCoder.getDeviceID() +
                " existing frequency == " + currentFrequency);
        }
    }

    /**
     * A method to update the TimeOfFlight sample rate to team-wide approved setting
     * @param sensor - the TimeOfFlight
     */
    public static void UpdateTofFrequency(TimeOfFlight sensor) {
        RangingMode currentMode = sensor.getRangingMode(); 
        double currentPeriod = sensor.getSampleTime();
        double currentFrequency = 1000.0/currentPeriod;
        if(currentFrequency < Constants.ctreSensorStatusFramePeriodFrequencyHertz) {
            int targetPeriod = (int)(1000.0/Constants.ctreSensorStatusFramePeriodFrequencyHertz);
            sensor.setRangingMode(currentMode, targetPeriod);
            System.out.println(
                "Updating period for TimeOfFlight serial # " +
                sensor.getSerialNumber() +
                " from " + currentPeriod +
                " to " + targetPeriod);
        }
        else {
            System.out.println(
                "No period change for TimeOfFlight serial # " +
                sensor.getSerialNumber() +
                " existing period == " + currentPeriod);
        }
    }
}
