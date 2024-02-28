package frc.robot.control;

public class ArmPlusPickerAction {
    
    private double yPointMeters = 0.0;
    private double zPointMeters = 0.0;
    private double motorSpeed = 0.0;
    private double overCurrentAmps = 0.0;

    public ArmPlusPickerAction(
        double yArmPointMeters,
        double zArmPointMeters,
        double pickerMotorSpeed,
        double pickerOverCurrentAmps) {
        yPointMeters = yArmPointMeters;
        zPointMeters = zArmPointMeters;
        motorSpeed = pickerMotorSpeed;
        overCurrentAmps = pickerOverCurrentAmps;
    }

    public double getYArmPointMeters() { return this.yPointMeters; }
    public double getZArmPointMeters() { return this.zPointMeters; }
    public double getPickerMotorSpeed() { return this.motorSpeed; }
    public double getPickerOverCurrentAmps() { return this.overCurrentAmps; }
}
