// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSubsystem.java
// Intent: Forms a command to make the XBox controller rumble/vibrate.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.subsystems.ShooterOutfeedSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootAllStopCommand extends Command {
  
  private final ShooterOutfeedSubsystem shooterSubsystem;

  public ShootAllStopCommand(ShooterOutfeedSubsystem theShooterSubsystem) {
    shooterSubsystem = theShooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize ShooterAllStopCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Execute ShooterAllStopCommand");
    shooterSubsystem.setAllStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("End ShooterAllStopCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}