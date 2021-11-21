package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModuleUtilities {
    public static SwerveModule createFalcon500(ShuffleboardLayout container,SwerveModuleConfig config) {
        return Mk4SwerveModuleHelper.createFalcon500(container,config.getGearRatio(),config.getDriveMotorID(),config.getSteerMotorID(),config.getEncoderId(),config.getOffset());
    }
}
