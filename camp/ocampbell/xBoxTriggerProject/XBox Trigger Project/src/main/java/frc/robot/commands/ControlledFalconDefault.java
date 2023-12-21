// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// File: ControlledFalconDefault.java
// Intent: Assigns a motor speed value to a falcon.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.commands;

import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.ControlledFalcon;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlledFalconDefault extends CommandBase {

  private SubsystemCollection subsystems;


  private ControlledFalcon motor;
  private double speed = 0.0;
  private boolean done = false;
  private double lTrigger = 0.0;
  private double rTrigger = 0.0;


  public ControlledFalconDefault(SubsystemCollection collection, ControlledFalcon motor, double lTrigger, double rTrigger) {
      this.motor = motor;
      this.lTrigger = lTrigger;
      this.rTrigger = rTrigger;

    addRequirements(motor);
    subsystems = collection;
}

  @Override
  public void initialize() {}


  @Override
  public void execute() {

    lTrigger = -lTrigger;

    //delta approach
    speed = lTrigger + rTrigger;
    speed = Math.max(-1.0, Math.min(1.0, speed));

    //left trigger wins 
    if(Math.abs(lTrigger) > rTrigger){
      speed = lTrigger;
    } else {
      speed = rTrigger;
    }

    //right trigger wins 
    if(rTrigger >= Math.abs(lTrigger)){
      speed = rTrigger;
    } else {
      speed = lTrigger;
    } 

    //gta approach (favors right trigger but if both are pulled it breaks(speed = 0))
    if(rTrigger > 0){
      speed = rTrigger;
    } else if (rTrigger == Math.abs(lTrigger)){   //possibly needs deadband
      speed = 0;
    } else if (Math.abs(lTrigger) > 0) {
      speed = lTrigger;
    } 





    motor.setFalconRelativeSpeed(speed);
    done = true;

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return done;
  }
}
