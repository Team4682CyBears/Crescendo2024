package frc.robot.swerveHelpers;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

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