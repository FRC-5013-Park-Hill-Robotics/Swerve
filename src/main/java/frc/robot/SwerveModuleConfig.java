package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;

public class SwerveModuleConfig {
    private final int steerMotorID;
    private final int driveMotorID;
    private final int encoderId;
    private final double offset;
    private final GearRatio gearRatio;

    public SwerveModuleConfig(int steerMotorID, int driveMotorID, int encoderId, double offset, GearRatio gearRatio) {
        super();
        this.steerMotorID = steerMotorID;
		this.driveMotorID = driveMotorID;
		this.encoderId = encoderId;
        this.offset = offset;
        this.gearRatio = gearRatio;
	}

    public int getSteerMotorID() {
        return steerMotorID;
    }

    public int getDriveMotorID() {
        return driveMotorID;
    }

    public int getEncoderId() {
        return encoderId;
    }

    public double getOffset() {
		return offset;
    }

    public GearRatio getGearRatio(){
        return gearRatio;
    }
}
