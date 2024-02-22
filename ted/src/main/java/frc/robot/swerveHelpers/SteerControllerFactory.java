// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SteerControllerFactory.java
// Intent: Same name extension files based on Swerve Drive Specalties codebase but also ported from phoenix5 to phoenix6
// SDS codebase found at: https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/tree/develop/src/main/java/com/swervedrivespecialties/swervelib
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.swerveLib.ModuleConfiguration;

@FunctionalInterface
public interface SteerControllerFactory<Controller extends SteerController, SteerConfiguration> {
   default void addDashboardEntries(ShuffleboardContainer container, Controller controller) {
      container.addNumber("Current Angle", () -> {
         return Math.toDegrees(controller.getStateAngle());
      });
      container.addNumber("Target Angle", () -> {
         return Math.toDegrees(controller.getReferenceAngle());
      });
   }

   default Controller create(ShuffleboardContainer dashboardContainer, SteerConfiguration steerConfiguration, ModuleConfiguration moduleConfiguration) {
      Controller controller = this.create(steerConfiguration, moduleConfiguration);
      this.addDashboardEntries(dashboardContainer, controller);
      return controller;
   }

   Controller create(SteerConfiguration var1, ModuleConfiguration var2);
}