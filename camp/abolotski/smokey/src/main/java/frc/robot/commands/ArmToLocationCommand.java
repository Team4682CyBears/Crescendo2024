// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmToLocationCommand.java
// Intent: Forms a command to move the dual part arm to defined 'named' spot in space.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.common.ChargedUpGamePiece;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToLocationCommand extends ArmToExtensionsCommand {
    ArmLocation location;
    ManualInputInterfaces input;

    /**
     * A command capable to push the arm to a point in space based on the points 'well known name'
     * @param arm - the arm subsystem that should be in use
     * @param location - the enum 'named' location to drive the arm to in space
     */
    public ArmToLocationCommand(ArmSubsystem arm, ArmLocation location, ManualInputInterfaces input) {
        super(arm, 0.0, 0.0);
        this.location = location;
        this.input = input; 
        setPositions();
    }

    public void initialize(){
        // reset positions in case cube/cone has changed since command was constructed
        setPositions();
        super.initialize();
    }

    private void setPositions(){
        switch(location){
            case ARM_STOW:
                super.setHorizontalExtension(Constants.armPresetPositionStowMetersHorizontalExtension);
                super.setVerticalExtension(Constants.armPresetPositionStowMetersVerticalExtension);
                break;

            case ARM_GRAB:
                if(input.getTargetGamePiece() == ChargedUpGamePiece.Cone) {
                    super.setHorizontalExtension(Constants.armPresetPositionConeGrabMetersHorizontalExtension);
                    super.setVerticalExtension(Constants.armPresetPositionConeGrabMetersVerticalExtension);
                }
                else if(input.getTargetGamePiece() == ChargedUpGamePiece.Cube) {
                    super.setHorizontalExtension(Constants.armPresetPositionCubeGrabMetersHorizontalExtension);
                    super.setVerticalExtension(Constants.armPresetPositionCubeGrabMetersVerticalExtension);
                }
                break;

            case ARM_LOW_SCORE:
                if(input.getTargetGamePiece() == ChargedUpGamePiece.Cone) {
                    super.setHorizontalExtension(Constants.armPresetPositionConeScoreLowMetersHorizontalExtension);
                    super.setVerticalExtension(Constants.armPresetPositionConeScoreLowMetersVerticalExtension);
                }
                else if(input.getTargetGamePiece() == ChargedUpGamePiece.Cube) {
                    super.setHorizontalExtension(Constants.armPresetPositionCubeScoreLowMetersHorizontalExtension);
                    super.setVerticalExtension(Constants.armPresetPositionCubeScoreLowMetersVerticalExtension);
                }
                break;

            case ARM_MED_SCORE:
                if(input.getTargetGamePiece() == ChargedUpGamePiece.Cone) {
                    super.setHorizontalExtension(Constants.armPresetPositionConeScoreMediumMetersHorizontalExtension);
                    super.setVerticalExtension(Constants.armPresetPositionConeScoreMediumMetersVerticalExtension);
                }
                else if(input.getTargetGamePiece() == ChargedUpGamePiece.Cube) {
                    super.setHorizontalExtension(Constants.armPresetPositionCubeScoreMediumMetersHorizontalExtension);
                    super.setVerticalExtension(Constants.armPresetPositionCubeScoreMediumMetersVerticalExtension);
                }
                break;

            case ARM_HIGH_SCORE: 
                if(input.getTargetGamePiece() == ChargedUpGamePiece.Cone) {
                    super.setHorizontalExtension(Constants.armPresetPositionConeScoreHighMetersHorizontalExtension);
                    super.setVerticalExtension(Constants.armPresetPositionConeScoreHighMetersVerticalExtension);
                }
                else if(input.getTargetGamePiece() == ChargedUpGamePiece.Cube) {
                    super.setHorizontalExtension(Constants.armPresetPositionCubeScoreHighMetersHorizontalExtension);
                    super.setVerticalExtension(Constants.armPresetPositionCubeScoreHighMetersVerticalExtension);
                }
                break;

            default: //use stow position
                super.setHorizontalExtension(Constants.armPresetPositionStowMetersHorizontalExtension);
                super.setVerticalExtension(Constants.armPresetPositionStowMetersVerticalExtension);
                break;
        }        
    }


    /**
     * The preset spots the arm should normally be driven to
     */
    public enum ArmLocation{
        ARM_STOW,
        ARM_GRAB,
        ARM_LOW_SCORE,
        ARM_MED_SCORE,
        ARM_HIGH_SCORE,
    }    
}
