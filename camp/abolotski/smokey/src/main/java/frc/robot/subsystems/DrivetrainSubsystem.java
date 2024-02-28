// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DrivetrainSubsystem.java
// Intent: Forms the prelminary code for drive train subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.util.*;
import java.lang.Math;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.common.EulerAngle;
import frc.robot.common.VectorUtils;
import frc.robot.control.SwerveDriveMode;
import frc.robot.control.SubsystemCollection;
import frc.robot.common.MotorUtils;
import frc.robot.common.SwerveDriveCenterOfRotation;
import frc.robot.common.SwerveTrajectoryConfig;
import frc.robot.common.VisionMeasurement;
import frc.robot.swerveHelpers.SwerveModuleHelper;
import frc.robot.swerveHelpers.SwerveModule;
import frc.robot.swerveHelpers.WcpModuleConfigurations;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  CameraSubsystem cameraSubsystem;
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0 * Constants.DriveVoltageScalar;

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   * From: https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
   * Gear ratio: 7.85:1. Free speed of 14.19 ft/s = 4.3251 m/s
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.3251;
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 6.0;

  public static final double MIN_VELOCITY_BOUNDARY_METERS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND * 0.14; // 0.14 a magic number based on testing
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  // override with a lower value.  calculated value was 14.01
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 10.0; 
  // public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
  //         Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  public static final double MIN_ANGULAR_VELOCITY_BOUNDARY_RADIANS_PER_SECOND = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.06; // 0.06 a magic number based on testing
  private double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 15.0;

  private static final int PositionHistoryWindowTimeMilliseconds = 5000;
  private static final int CommandSchedulerPeriodMilliseconds = 20;
  private final double deltaTimeSeconds = 0.02; // 20ms scheduler time tick
  private static final int CommandSchedulerCyclesPerSecond = 1000/CommandSchedulerPeriodMilliseconds;
  private static final int PositionHistoryStorageSize = PositionHistoryWindowTimeMilliseconds/CommandSchedulerPeriodMilliseconds;

  private static final Lock theLock = new ReentrantLock();

  private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final AHRS swerveNavx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private double yawOffsetDegrees = 0.0;
  private double pitchOffsetDegrees = 0.0;
  private double rollOffsetDegrees = 0.0;

  // store yaw/pitch history
  private static final int LevelListMaxSize = 9; // 12 * 20ms = 0.24s total history time
  private ArrayList<Double> RecentRolls = new ArrayList<Double>();
  private ArrayList<Double> RecentPitches = new ArrayList<Double>();

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private SwerveDrivePoseEstimator swervePoseEstimator = null;
  private Pose2d currentPosition = new Pose2d();
  private ArrayDeque<Pose2d> historicPositions = new ArrayDeque<Pose2d>(PositionHistoryStorageSize + 1);

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private ChassisSpeeds previousChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private double speedReductionFactor = 1.0;

  private SwerveDriveMode swerveDriveMode = SwerveDriveMode.NORMAL_DRIVING;
  private SwerveDriveCenterOfRotation swerveDriveCenterOfRotation = SwerveDriveCenterOfRotation.RobotCenter;

  /**
   * Constructor for this DrivetrainSubsystem
   */
  public DrivetrainSubsystem(SubsystemCollection subsystems) {
    cameraSubsystem = subsystems.getCameraSubsystem();

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    frontLeftModule = SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration 
            WcpModuleConfigurations.SWERVEX,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    frontRightModule = SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            WcpModuleConfigurations.SWERVEX,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    backLeftModule = SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            WcpModuleConfigurations.SWERVEX,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    backRightModule = SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            WcpModuleConfigurations.SWERVEX,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    // We assume the robot is level at startup.  Take out any bias the NavX is reading on Pitch/Roll.  
    removePitchRollBias(); 
  }

  /**
   * Method avaliable so that callers can update the chassis speeds to induce changes in robot movement
   * @param updatedChassisSpeeds - the updated chassis speeds (x, y and rotation)
   */
  public void drive(ChassisSpeeds updatedChassisSpeeds) {
    this.chassisSpeeds = updatedChassisSpeeds;
  }

  /**
   * returns navx euler angle (pitch, roll, yaw) in degrees
   * @return EulerAngle
   */
  public EulerAngle getEulerAngle(){
    return new EulerAngle(
      getNavxPitch(), 
      getNavxRoll(), 
      swerveNavx.getYaw() + this.yawOffsetDegrees);
  }

  /**
   * Method to get the current speed reduction factor
   * @return - the current speed reduction factor
   */
  public double getSpeedReductionFactor() {
    return this.speedReductionFactor;
  }

  /**
   * A method to return the swerveDriveKinematics of the drivetrain
   * @return kinematics
   */
  public SwerveDriveKinematics getSwerveKinematics() {
    return swerveKinematics;
  }

  /**
   * Method to set the current speed reduction factor to a new value
   * @param value - the new speed reduction factor
   */
  public void setSpeedReductionFactor(double value) {
    this.speedReductionFactor = MotorUtils.truncateValue(value, 0.0, 1.0);
  }

  /**
   * Obtains the current gyroscope's rotation about the Z axis when looking down at the robot where positive is measured in the 
   * counter-clockwise direction.
   * 
   * Notes: 
   * According to https://pdocs.kauailabs.com/navx-mxp/wp-content/uploads/2020/09/navx2-mxp_robotics_navigation_sensor_user_guide-8.pdf
   * NavX .getYaw() is -180 to 180 where positive is clockwise looking at the sensor from above
   * According to https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
   * positive rotations are measured in the counter-clockwise looking at the robot from above
   * these two pieces of information imply the yaw should be inverted (or multiplied by -1.0)
   * @return A Rotation2d that describes the current orentation of the robot.
   */
  public Rotation2d getGyroscopeRotation() {
    // Note: for the navx1, isMagnetometerCalibrated was always false, and so the code exercised the second path (using getYaw)
    // when we switched to the navx2, isMagnetometerCalibrated was true, and so the code exercised the first path (using getFusedHeading)
    // this first path does not work for us because isMagtometerCalibrated does not zero when yaw is zeroed
    // our ability to both zero and set the yaw is critical to field-oriented drive.  
    // If we would like to us getFusedHeading in the future, we will need to change how we zero and set the yaw.  

    /* 
    if (swerveNavx.isMagnetometerCalibrated()) {

      // System.out.println("getGyroscopeRotation() using: swerveNavx.getFusedHeading()");

      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromRadians(
        MathUtil.angleModulus(
          (360 - swerveNavx.getFusedHeading()+ yawOffsetDegrees)*(2*Math.PI)/360));
    }
    */

    // System.out.println("getGyroscopeRotation() using: swerveNavx.getYaw()");

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    // TODO test angleModulus again with the NavX1.  We disabled it during debugging the NavX2, but it might have been
    // a red herring.  
    //return Rotation2d.fromRadians(
    //  MathUtil.angleModulus(
    //    (360.0 - swerveNavx.getYaw() + yawOffsetDegrees)*(2*Math.PI)/360));
    return Rotation2d.fromDegrees(360.0 - swerveNavx.getYaw() + yawOffsetDegrees);
  }
  
  /**
  * A method to get the quaternion representation of the navx pose. 
  * @return quaternion
  */
  public Quaternion getQuaterion() {
    Quaternion q = new Quaternion(
    swerveNavx.getQuaternionW(),
    swerveNavx.getQuaternionX(),
    swerveNavx.getQuaternionY(),
    swerveNavx.getQuaternionZ());
    return (q);
  }

  /**
   * A method to get the current position of the robot
   * @return the current Pose2d position of the robot
   */
  public Pose2d getRobotPosition()
  {
    return currentPosition;
  }

  /**
   * A method to obtain the distance along the segments recently traveled
   * @param historicDurationMilliseconds - the total milliseconds to look back in time
   * @return A double of the distance traveled by the robot in meters over the time window requested
   */
  public double getRecentTotalDistanceInMeters(int historicDurationMilliseconds)
  {
    ArrayList<Double> distances = this.getRecentDistanceTraveled(historicDurationMilliseconds);
    double resultDistance = 0.0;
    for(int inx = 0; inx < distances.size(); ++inx)
    {
      resultDistance += distances.get(inx);
    }
    return resultDistance;
  }

  /**
   * A method to obtain the average velocity of the robot recently traveled
   * @param historicDurationMilliseconds - the total milliseconds to look back in time
   * @return A double of the velocity of the robot in meters per second over the time window requested
   */
  public double getRecentAverageVelocityInMetersPerSecond(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentVelocities = this.getRecentVelocities(historicDurationMilliseconds);
    double sumOfVelocities = 0.0;
    int countOfDeltas = 0;
    for(; countOfDeltas < recentVelocities.size(); ++countOfDeltas)
    {
      sumOfVelocities += recentVelocities.get(countOfDeltas);
    }
    return sumOfVelocities/countOfDeltas;
  }

  /**
   * A method to obtain the average angular velocity of the robot recently experienced
   * @param historicDurationMilliseconds - the total milliseconds to look back in time
   * @return A double of the angular velocity of the robot in radians per second over the time window requested
   */
  public double getRecentAverageAngularVelocityInRadiansPerSecond(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentAngularVelocities = this.getRecentAngularVelocities(historicDurationMilliseconds);

    double sumOfAngularVelocities = 0.0;
    int countOfDeltas = 0;
    for(; countOfDeltas < recentAngularVelocities.size(); ++countOfDeltas)
    {
      sumOfAngularVelocities += recentAngularVelocities.get(countOfDeltas);
    }
    return sumOfAngularVelocities/countOfDeltas;
  }

  /**
   * A method to obtain the acceleration experienced by the robot over the 
   * @param historicDurationMilliseconds - the total milliseconds to look back in time
   * @return A double of the average acceleration experienced by the robot in meters/second^2 for the time window requested
   */
  public double getRecentAverageAccelerationInMetersPerSecondSquared(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentAccelerations = this.getRecentAccelerations(historicDurationMilliseconds);
    double sumOfAccelerations = 0.0;
    int countOfDeltas = 0;
    for(; countOfDeltas < recentAccelerations.size(); ++countOfDeltas)
    {
      sumOfAccelerations += recentAccelerations.get(countOfDeltas);
    }
    return sumOfAccelerations/countOfDeltas;
  }

   /**
    * A method to obtain the recent pitches 
    * @return a listing of recent pitches
    */
   public ArrayList<Double> getRecentPitches(){
     return RecentPitches;
   }
 
   /**
    * A method to obtain the recent rolls 
    * @return a listing of recent rolls
    */
    public ArrayList<Double> getRecentRolls(){
     return RecentRolls;
   }
 
   /**
    * A method to obtain the swerve drive mode
    * @return the mode
    */
   public SwerveDriveMode getSwerveDriveMode() {
    return swerveDriveMode;
  }

  /**
  * Function to obtain the TrajectoryConfig
  * returns a new trajectory config so that when customization are made downstream
  * they do not affect other trajectories.
  * Accelerations used in trjactories are lower than used for joystick driving. 
  * @return a SwerveTrajectoryConfig in use within the drive train subsystem
  */
  public SwerveTrajectoryConfig getTrajectoryConfig() {
    SwerveTrajectoryConfig config = new SwerveTrajectoryConfig(
      MAX_VELOCITY_METERS_PER_SECOND, 
      1.25, // 6.0 max
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
      8.0); // 12.0 max
    config.setReversed(false).setKinematics(swerveKinematics);
    return config;
  }

  /**
   * Determines if the navx is level.  
   * @return true if level, false otherwise
   */
  public boolean isLevel() {
    return this.areAllLevel(RecentPitches) && this.areAllLevel(RecentRolls);
  }

  /**
   * Perodic for this subsystem - very important for it to run every scheduler cycle - 50Hz
   */
  @Override
  public void periodic() {

    // refresh the position of the robot
    this.refreshRobotPosition();
    // update robot position with vision 
    this.addVisionMeasurement(cameraSubsystem.getVisionPosition());
    // store the recalculated position
    this.storeUpdatedPosition();
    // store navx info
    this.storePitch();
    this.storeRoll();   

    this.displayDiagnostics();

    SwerveModuleState[] states; 
    if (swerveDriveMode == SwerveDriveMode.IMMOVABLE_STANCE && chassisSpeedsAreZero()) {
      // only change to ImmovableStance if chassis is not moving.
      // otherwise, we could tip the robot moving to this stance when bot is at high velocity
      states = getImmovableStanceStates();
    }
    else { // SwerveDriveMode.NORMAL_DRIVING
      // apply the speed reduction factor to the chassis speeds
      ChassisSpeeds reducedChassisSpeeds = new ChassisSpeeds(
        chassisSpeeds.vxMetersPerSecond * this.speedReductionFactor, 
        chassisSpeeds.vyMetersPerSecond * this.speedReductionFactor, 
        // different speed reduction factor for rotation
        chassisSpeeds.omegaRadiansPerSecond * Math.min(1.0, this.speedReductionFactor * 1.25));

      // apply acceleration control and discretization correction
      reducedChassisSpeeds = discretize(limitChassisSpeedsAccel(reducedChassisSpeeds));
      previousChassisSpeeds = reducedChassisSpeeds; 

      // take the current 'requested' chassis speeds and ask the ask the swerve modules to attempt this
      // first we build a theoretical set of individual module states that the chassisSpeeds would corespond to
      if (swerveDriveCenterOfRotation == SwerveDriveCenterOfRotation.RobotFront) {
        states = swerveKinematics.toSwerveModuleStates(reducedChassisSpeeds, Constants.RobotFrontRotationalCenter);
      } 
      else { // normal rotation mode 
        states = swerveKinematics.toSwerveModuleStates(reducedChassisSpeeds);
      }
      // next we take the theoretical values and bring them down (if neecessary) to incorporate physical constraints (like motor maximum speeds)
      SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    } 

    // now we take the four states and ask that the modules attempt to perform the wheel speed and direction built above
    frontLeftModule.set(
      states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
      states[0].angle.getRadians());
    frontRightModule.set(
      states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
      states[1].angle.getRadians());
    backLeftModule.set(
      states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
      states[2].angle.getRadians());
    backRightModule.set(
      states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
      states[3].angle.getRadians());
  }

  /**
   * a method to print relevant state of the navx
   */
  public void printState(){
    System.out.println("**** NavX State ****");
    System.out.println("Quaternion ------>" + this.getQuaterion());
    System.out.println("Roll, Pitch, Yaw ------>" + this.getEulerAngle());
    System.out.println("Is the robot level? -------->" + this.isLevel());
    System.out.println("SteepestAscent --->" + VectorUtils.getAngleOfSteepestAscent(getEulerAngle()));
  }

  /**
   * A method to set the current position of the robot
   * @param updatedPosition - the new position of the robot
   */
  public void setRobotPosition(Pose2d updatedPosition)
  {
    //TOD this didn't work to fix the problem of the robot not getting correct initial position.  
    // try to debug why not.  
    try{
      theLock.lock();
      // initialize the odometry goo
      currentPosition = updatedPosition;
      this.initializeSwerveOdometry(currentPosition);
    }
    finally {
      theLock.unlock();
    }
  }

  /**
   * A method to set the swerve drive mode
   * @param swerveDriveMode
   */
  public void setSwerveDriveMode(SwerveDriveMode swerveDriveMode) {
    this.swerveDriveMode = swerveDriveMode;
    // if (swerveDriveMode == SwerveDriveMode.IMMOVABLE_STANCE) DO NOT set chassis speeds to 0.0 here.
    // we need to allow the robot to decel properly so that it doesn't tip.  
  }


  /**
   * A method to set the swerve rotation mode
   * @param swerveDriveRotationMode
   */
  public void setSwerveDriveCenterOfRotation(SwerveDriveCenterOfRotation swerveDriveCenterOfRotation) {
    this.swerveDriveCenterOfRotation = swerveDriveCenterOfRotation;
  }

  /**
   * sets the Yaw to a specific angle
   * @param offsetDegrees
   */
  public void setYaw(double offsetDegrees) {
    this.zeroGyroscope();
    this.yawOffsetDegrees = offsetDegrees;
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    if(swerveNavx.isCalibrating()){
      // From the NavX Docs: This method has no effect if the sensor is currently calibrating
      System.out.println("WARNING: Gyro is calibrating. Zeroing gyro has no effect while it is calibrating.");
    }
    swerveNavx.zeroYaw();
    this.yawOffsetDegrees = 0.0;
  }

  /**
   * A method to zero the current position
   */
  public void zeroRobotPosition()
  {
    this.setRobotPosition(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
  }

  /**
   * A method that updates the robot position with a vision measurement
   * @param visionMeasurement the most recent vision measurement provided by vision subsystem
   */
  private void addVisionMeasurement(VisionMeasurement visionMeasurement){
    // for now ignore all vision measurements that are null or contained robot position is null
    if (visionMeasurement != null && visionMeasurement.getRobotPosition() != null){
      swervePoseEstimator.addVisionMeasurement(visionMeasurement.getRobotPosition(), visionMeasurement.getTimestamp());
    }
  } 

  /**
   * Determine if recent navx is all level
   * @param recentAngles the recent set of angles recorded by navx 
   * @return true if the robot has been recently level
   */
  private boolean areAllLevel(ArrayList<Double> recentAngles){
    boolean levelChecker = true;
    for(int i = 0; i < LevelListMaxSize; i++){
      if (Math.abs(recentAngles.get(i))>=Constants.navxTolDegrees){
        levelChecker = false;
      }
    }
    System.out.println("Is Level? " + levelChecker);
    return levelChecker;
  }

  private boolean chassisSpeedsAreZero(){
    return (chassisSpeeds.vxMetersPerSecond == 0.0) 
    && (chassisSpeeds.vyMetersPerSecond == 0.0) 
    && (chassisSpeeds.omegaRadiansPerSecond == 0.0);
  }
  
  /**
   * Clamps the chassis speeds between a min and max.  
   * Separete min and max for translational (x,y) vs. rotational speeds
   * @param chassisSpeeds
   * @param translationMin
   * @param translationMax
   * @param rotationMin
   * @param rotationMax
   * @return clamped chassisSpeeds
   */
  private ChassisSpeeds clampChassisSpeeds(
    ChassisSpeeds chassisSpeeds, 
    double translationMin,
    double translationMax,
    double rotationMin,
    double rotationMax){
    // do not scale omega
    //double clampedOmega = MotorUtils.doubleSidedClamp(chassisSpeeds.omegaRadiansPerSecond, rotationMin, rotationMax);
    // if one or both of X or Y needs to be clamped, we need to scale both proportionally
    // form a Translation2d of x, y so the math is easier
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    double velocity = translation.getNorm();
    double clampedVelocity = MotorUtils.doubleSidedClamp(velocity, translationMin, translationMax);
    // scale the translation by the clamped velocity scale factor
    double scale = clampedVelocity/velocity;
    translation = translation.times(scale);
    return new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Clamps the chassis speeds between the drivetrain min and max velocities
   * @param chassisSpeeds
   * @return clamped chassisSpeeds
   */
  private ChassisSpeeds clampChassisSpeeds(ChassisSpeeds chassisSpeeds){
    return this.clampChassisSpeeds(chassisSpeeds, 
    MIN_VELOCITY_BOUNDARY_METERS_PER_SECOND,
    MAX_VELOCITY_METERS_PER_SECOND, 
    MIN_ANGULAR_VELOCITY_BOUNDARY_RADIANS_PER_SECOND, 
    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
  }

  private ChassisSpeeds discretize(ChassisSpeeds speeds) {
    // a fudge factor to increase the size of the discretization correction. 
    // other teams use [1..4]
    double timeScaleFactor = 1.9; 
    var desiredDeltaPose = new Pose2d(
      speeds.vxMetersPerSecond * deltaTimeSeconds, 
      speeds.vyMetersPerSecond * deltaTimeSeconds, 
      new Rotation2d(speeds.omegaRadiansPerSecond * deltaTimeSeconds * timeScaleFactor)
    );
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds(
      (twist.dx / deltaTimeSeconds), 
      (twist.dy / deltaTimeSeconds), 
      (speeds.omegaRadiansPerSecond));
    // return(speeds);
  }


  private SwerveModuleState[] getImmovableStanceStates(){
    // set wheels in "X" pattern
    return new SwerveModuleState[] {
      // frontLeftModule
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
      // frontRightModule
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
      // backLeftModule
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
      // backRightModule
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
    }; 
  };

  /**
   * A method to get the debiased navx pitch
   * @return pitch in degrees
   */
  private double getNavxPitch(){
    return swerveNavx.getPitch() + this.pitchOffsetDegrees;
  }

  /**
   * A method to get the debiased navx roll
   * @return roll in degrees
   */
  private double getNavxRoll(){
    return swerveNavx.getRoll() + this.rollOffsetDegrees;
  }

  /**
   * Limits chassis speeds based on max allowable acceleration
   * @param speeds
   * @return
   */
  private ChassisSpeeds limitChassisSpeedsAccel(ChassisSpeeds speeds) {
    double xVelocityLimited = limitAxisSpeed(speeds.vxMetersPerSecond, previousChassisSpeeds.vxMetersPerSecond, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    double yVelocityLimited = limitAxisSpeed(speeds.vyMetersPerSecond, previousChassisSpeeds.vyMetersPerSecond, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    double omegaVelocityLimited = limitAxisSpeed(speeds.omegaRadiansPerSecond, previousChassisSpeeds.omegaRadiansPerSecond, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    return new ChassisSpeeds(xVelocityLimited, yVelocityLimited, omegaVelocityLimited);
  }

  /**
   * Limits axis speed based on max allowable acceleration
   * @param commandedSpeed
   * @param previousSpeed
   * @param maxAccel
   * @return limited speed
   */
  private double limitAxisSpeed(double commandedSpeed, double previousSpeed, double maxAccel){
      double accel = (commandedSpeed - previousSpeed)/deltaTimeSeconds;
      double speedLimited = commandedSpeed;
      if (Math.abs(accel) > maxAccel){
          // new velocity is the old velocity + the maximum allowed change toward the new direction
          speedLimited = previousSpeed + Math.copySign(maxAccel * deltaTimeSeconds, accel);
      }
      return speedLimited;
  }

  /**
   * A method to remove any bias in the pitch/roll. 
   * Should only be called when the robot is on level ground!
   */
  private void removePitchRollBias(){
    this.pitchOffsetDegrees = -1 * swerveNavx.getPitch(); 
    this.rollOffsetDegrees = -1 * swerveNavx.getRoll(); 
  }

/**
   * Method that will store roll
   */
  private void storeRoll(){
    this.RecentRolls.add(getNavxRoll());
    while(this.RecentRolls.size() > LevelListMaxSize)
    {
      RecentRolls.remove(0);
    }
  }
   
  /**
   * Method that will store pitch
   */
  private void storePitch(){
    RecentPitches.add(getNavxPitch());
    while(RecentPitches.size() > LevelListMaxSize)
    {
      RecentPitches.remove(0);
    }
  }

  private void displayDiagnostics(){
    SmartDashboard.putBoolean("NavX is calibrating", swerveNavx.isCalibrating());
    SmartDashboard.putBoolean("NavX is calibrated", swerveNavx.isMagnetometerCalibrated());
  }
 
  /**
   * Method used to initialize the Odometry for the robot 
   * @param currentRobotPosition - the centroid of the robot using the proper game coordinate system, see:
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
   */
  private void initializeSwerveOdometry(Pose2d currentRobotPosition) {
    frontLeftModule.setDriveDistance(0.0);
    frontRightModule.setDriveDistance(0.0);
    backLeftModule.setDriveDistance(0.0);
    backRightModule.setDriveDistance(0.0);
    swervePoseEstimator = new SwerveDrivePoseEstimator(
        swerveKinematics,
        this.getGyroscopeRotation(),
        this.getSwerveModulePositions(),
        currentRobotPosition); 
  }

  /**
   * Helper method to obtain the SwerveModulePostion array from an existing SwerveModules in this class
   * @return - a SwerveModulePosition array for the modules
   */
  private SwerveModulePosition [] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      this.getSwerveModulePositionFromModule(frontLeftModule),
      this.getSwerveModulePositionFromModule(frontRightModule),
      this.getSwerveModulePositionFromModule(backLeftModule),
      this.getSwerveModulePositionFromModule(backRightModule)
    };
  }

  /**
   * Helper method to obtain the SwerveModulePostion from an existing SwerveModule
   * @param module - the module to extract info from
   * @return - a SwerveModulePosition class that wraps the orentiation
   */
  private SwerveModulePosition getSwerveModulePositionFromModule(SwerveModule module) {
    return new SwerveModulePosition(
      module.getDriveDistance(),
      new Rotation2d(module.getSteerAngle()));
  }

  /**
   * A function intended to be called from perodic to update the robots centroid position on the field.
   */
  private void refreshRobotPosition() {
    // Update the position of the robot
    Rotation2d angle = this.getGyroscopeRotation();
    SwerveModulePosition[] positions = null;
    try{
      theLock.lock();
      positions = this.getSwerveModulePositions();
      currentPosition = swervePoseEstimator.update(
        angle,
        positions);
    }
    finally {
      theLock.unlock();
    }

    SmartDashboard.putNumber("RobotFieldHeadingDegrees", this.getGyroscopeRotation().getDegrees() );
    // getGyroscopeRotation() should be the same as currentPosition.getRotation after one call to swerveOdometry.update in periodic
    SmartDashboard.putNumber("RobotFieldXCoordinateMeters", currentPosition.getX());
    SmartDashboard.putNumber("RobotFieldYCoordinateMeters", currentPosition.getY());
    SmartDashboard.putNumber("RobotPitchDegrees", this.getEulerAngle().getPitch());
    SmartDashboard.putNumber("RobotRollDegrees", this.getEulerAngle().getRoll());
    if(positions != null){
      SmartDashboard.putNumber("FrontLeftAngleDegrees", positions[0].angle.getDegrees());
      SmartDashboard.putNumber("FrontLeftDistanceMeters", positions[0].distanceMeters);
      SmartDashboard.putNumber("FrontRightAngleDegrees", positions[1].angle.getDegrees());
      SmartDashboard.putNumber("FrontRightDistanceMeters", positions[1].distanceMeters);
      SmartDashboard.putNumber("BackLeftAngleDegrees", positions[2].angle.getDegrees());
      SmartDashboard.putNumber("BackLeftDistanceMeters", positions[2].distanceMeters);
      SmartDashboard.putNumber("BackRightAngleDegrees", positions[3].angle.getDegrees());
      SmartDashboard.putNumber("BackRightDistanceMeters", positions[3].distanceMeters);
    }
  }

  /**
   * A method devoted to making sure the current position is added to the historic position listing
   * as well as maintaining the size of the list at a predefined maximum
   */
  private void storeUpdatedPosition()
  {
    this.historicPositions.add(currentPosition);
    while(this.historicPositions.size() > PositionHistoryStorageSize)
    {
      this.historicPositions.remove();
    }
  }

  /**
   * Method to get the recent translations
   * @param historicDurationMilliseconds - the historic time window in milliseconds of points to establish translations
   * @return the array of translations associated with the points at hand
   */
  private ArrayList<Transform2d> getRecentTransforms(int historicDurationMilliseconds)
  {
    // first get the list of translations between the Pos2d's stored
    int intendedCount = historicDurationMilliseconds/CommandSchedulerPeriodMilliseconds;
    int maxCount = (intendedCount>PositionHistoryStorageSize ? PositionHistoryStorageSize : intendedCount);
    int currentCount = 0;
    ArrayList<Transform2d> transforms = new ArrayList<Transform2d>();
    Pose2d lastPosition = null;
    Pose2d currentPosition = null;
    for (Iterator<Pose2d> iter = this.historicPositions.iterator(); iter.hasNext() && currentCount < maxCount; ++currentCount ) {
      currentPosition = iter.next();
      // assume the first position has no translation (even though it likely actually have had one)
      if(lastPosition != null)
      {
        Transform2d nextTransform = new Transform2d(lastPosition, currentPosition);
        transforms.add(nextTransform);
      }
      lastPosition = currentPosition;
    }
    return transforms;
  }

  /**
   * Method to obtain the array of distances recently traveled
   * @param historicDurationMilliseconds - the historic time window in milliseconds of points to establish distances
   * @return array of distances traveled recently
   */
  private ArrayList<Double> getRecentDistanceTraveled(int historicDurationMilliseconds)
  {
    ArrayList<Transform2d> transforms = this.getRecentTransforms(historicDurationMilliseconds);
    ArrayList<Double> resultDistances = new ArrayList<Double>();
    Translation2d previousTranslation = null;
    Translation2d currentTranslation = null;
    for(int inx = 0; inx < transforms.size(); ++inx)
    {
      currentTranslation = transforms.get(inx).getTranslation();
      if(previousTranslation != null)
      {
        resultDistances.add(previousTranslation.getDistance(currentTranslation));
      }
      previousTranslation = currentTranslation;
    }
    return resultDistances;
  }

  /**
   * Method to obtain the array of angular velocities recently traveled
   * @param historicDurationMilliseconds - the historic time window in milliseconds to obtain velocities from
   * @return array of rotation velocities traveled recently - in radians/s
   */
  private ArrayList<Double> getRecentAngularVelocities(int historicDurationMilliseconds)
  {
    ArrayList<Transform2d> transforms = this.getRecentTransforms(historicDurationMilliseconds);
    ArrayList<Double> resultRotationVelocities = new ArrayList<Double>();
    Transform2d previousTransform = null;
    Transform2d currentTransform = null;
    for(int inx = 0; inx < transforms.size(); ++inx)
    {
      currentTransform = transforms.get(inx);
      if(previousTransform != null)
      {
        // with this conversion below we will be making an assumption:
        // that angular velocities will never exceed pi radians per 20 ms clock cycle
        // (e.g., that the robot is unable to sweep > 180 degrees in 20 ms,
        // or said another way the robot can't physically spin > 25 spins per second - 1500 RPM)
        // to produce a result, we need to rely on MathUtil.angleModulus() static method - this keeps the math tidy
        double currentRadians = MathUtil.angleModulus(currentTransform.getRotation().getRadians());
        double previousRadians = MathUtil.angleModulus(previousTransform.getRotation().getRadians());
        double deltaRadians = MathUtil.angleModulus(currentRadians - previousRadians);

        // assumed cycle time and that the current distance is for a 20 ms movement
        resultRotationVelocities.add(deltaRadians * CommandSchedulerCyclesPerSecond); 
      }
      previousTransform = currentTransform;
    }
    return resultRotationVelocities;
  }

  /**
   * Method to obtain the array of velocities recently traveled
   * @param historicDurationMilliseconds - the historic time window in milliseconds to obtain velocities from
   * @return array of velocities traveled recently
   */
  private ArrayList<Double> getRecentVelocities(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentDistances = this.getRecentDistanceTraveled(historicDurationMilliseconds);
    ArrayList<Double> resultVelocities = new ArrayList<Double>();
    for(int inx = 0; inx < recentDistances.size(); ++inx)
    {
      double currentDistance = recentDistances.get(inx);
      double currentVelocity = currentDistance * CommandSchedulerCyclesPerSecond; // assumed cycle time and that the current distance is for a 20 ms movement
      resultVelocities.add(currentVelocity);
    }
    return resultVelocities;
  }

  /**
   * Method to obtain the array of accelerations recently having occured
   * @param historicDurationMilliseconds - the historic time window in milliseconds to obtain accelerations from
   * @return array of accelerations traveled recently
   */
  private ArrayList<Double> getRecentAccelerations(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentVelocities = this.getRecentVelocities(historicDurationMilliseconds);
    ArrayList<Double> resultAccelerations = new ArrayList<Double>();
    Double currentVelocity = null;
    Double previousVelocity = null;
    for(int inx = 0; inx < recentVelocities.size(); ++inx)
    {
      currentVelocity = recentVelocities.get(inx);
      if(previousVelocity != null)
      {
        double currentAcceleration = (currentVelocity - previousVelocity) * CommandSchedulerCyclesPerSecond; // assumed cycle time and that the current distance is for a 20 ms movement
        resultAccelerations.add(currentAcceleration);
      }
      previousVelocity = currentVelocity;
    }
    return resultAccelerations;
  }

}