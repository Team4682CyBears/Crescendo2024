package frc.robot.swerveHelpers;

public interface SteerController {
   double getAbsoluteEncoderOffset();

   double getReferenceAngle();

   void setAbsoluteEncoderOffset();
   
   void setReferenceAngle(double var1);

   double getStateAngle();

}
