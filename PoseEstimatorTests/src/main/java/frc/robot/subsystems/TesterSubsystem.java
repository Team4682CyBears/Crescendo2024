package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class TesterSubsystem extends SubsystemBase {

  private final double trackwidth = Units.inchesToMeters(17.179);
  private final double wheelbase = Units.inchesToMeters(17.179);

  private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
    new Translation2d(trackwidth / 2.0, wheelbase / 2.0),
    new Translation2d(trackwidth / 2.0, -wheelbase / 2.0),
    new Translation2d(-trackwidth / 2.0, wheelbase / 2.0),
    new Translation2d(-trackwidth / 2.0, -wheelbase / 2.0)
  );

  private final Rotation2d fakeGyroAngle = new Rotation2d(0);

  private final SwerveModulePosition[] fakeSwerveModulePositions = new SwerveModulePosition{
    
  }

  private SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
    swerveKinematics,
    fakeGyroAngle


  );
  /** Creates a new ExampleSubsystem. */
  public TesterSubsystem() {

  }


  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
