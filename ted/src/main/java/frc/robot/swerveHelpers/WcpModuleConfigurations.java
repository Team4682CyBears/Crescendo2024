// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: WcpModuleConfigurations.java
// Intent: Same name extension files based on Swerve Drive Specalties codebase but also ported from phoenix5 to phoenix6
// SDS codebase found at: https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/tree/develop/src/main/java/com/swervedrivespecialties/swervelib
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import frc.robot.swerveLib.ModuleConfiguration;

public class WcpModuleConfigurations {
    public static final ModuleConfiguration SWERVEX = new ModuleConfiguration(
            0.1016,  //4" in meters
            1/7.85, // 7.85:1 (10:34 -> 26:20 -> 15:45)
            true, //three reductions
            1/15.43, // 15.43:1 (8:24 -> 14:72)
            false //two reductions
    );
}
