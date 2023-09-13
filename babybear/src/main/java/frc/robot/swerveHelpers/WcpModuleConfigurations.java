// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: WcpModuleConfigurations.java
// Intent: WC configs ... a modified copy of SWS content.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;

public class WcpModuleConfigurations {
    public static final ModuleConfiguration SWERVEX = new ModuleConfiguration(
            0.1016,  //4" in meters
            1/7.85, // 7.85:1 (10:34 -> 26:20 -> 15:45)
            true, //three reductions
            1/15.43, // 15.43:1 (8:24 -> 14:72)
            false //two reductions
    );
}
