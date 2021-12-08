// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DrivetrainConstants.*;

public class DrivetrainSubsystem extends SubsystemBase {

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    //FIX We need to figure out initial possition.
    private Pose2d m_pose = new Pose2d();
    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(),
            m_pose);

    
    // By default we use a Pigeon for our gyroscope. But if you use another
    // gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating
    // the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    private final PigeonIMU m_pigeon = new PigeonIMU(PIGEON_ID);

    
    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private SwerveModuleState[] m_desiredStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                SWERVE_GEAR_RATIO, FrontLeftSwerveConstants.DRIVE_MOTOR_ID, FrontLeftSwerveConstants.STEER_MOTOR_ID,
                FrontLeftSwerveConstants.ENCODER_ID, FrontLeftSwerveConstants.OFFSET);

        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                SWERVE_GEAR_RATIO, FrontRightSwerveConstants.DRIVE_MOTOR_ID, FrontRightSwerveConstants.STEER_MOTOR_ID,
                FrontRightSwerveConstants.ENCODER_ID, FrontRightSwerveConstants.OFFSET);

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                SWERVE_GEAR_RATIO, BackLeftSwerveConstants.DRIVE_MOTOR_ID, BackLeftSwerveConstants.STEER_MOTOR_ID,
                BackLeftSwerveConstants.ENCODER_ID, BackLeftSwerveConstants.OFFSET);

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                SWERVE_GEAR_RATIO, BackRightSwerveConstants.DRIVE_MOTOR_ID, BackRightSwerveConstants.STEER_MOTOR_ID,
                BackRightSwerveConstants.ENCODER_ID, BackRightSwerveConstants.OFFSET);

        
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_pigeon.setFusedHeading(0.0);

    }
    public void resetPosition(Pose2d newPosition, Rotation2d newRotation) {
        m_odometry.resetPosition(newPosition, newRotation);
        m_pose = m_odometry.getPoseMeters();
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    }

    public double getHeading() {
        return getGyroscopeRotation().getRadians();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_desiredStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    }

    public void setDesiredStates(SwerveModuleState[] newStates){
        m_desiredStates = newStates;
    }
    @Override
    public void periodic() {
        updateDriveStates(m_desiredStates);
    }

    public void updateDriveStates(SwerveModuleState[] desiredStates) {
        SwerveModuleState frontLeftState = desiredStates[FrontLeftSwerveConstants.STATES_INDEX];
        SwerveModuleState frontRightState = desiredStates[FrontRightSwerveConstants.STATES_INDEX];
        SwerveModuleState backLeftState = desiredStates[BackLeftSwerveConstants.STATES_INDEX];
        SwerveModuleState backRightState = desiredStates[BackRightSwerveConstants.STATES_INDEX];

        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(frontLeftState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                frontLeftState.angle.getRadians());
        m_frontRightModule.set(frontRightState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                frontRightState.angle.getRadians());
        m_backLeftModule.set(backLeftState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                backLeftState.angle.getRadians());
        m_backRightModule.set(backRightState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                backRightState.angle.getRadians());

        m_pose = m_odometry.update(getGyroscopeRotation(), frontLeftState, frontRightState, backLeftState,
               backRightState);  
    }

    public Pose2d getPose(){
        return m_pose;
    }
}