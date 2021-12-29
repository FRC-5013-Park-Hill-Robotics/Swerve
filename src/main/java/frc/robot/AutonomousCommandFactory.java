package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DrivetrainConstants.DrivetrainGeometry;
import frc.robot.Constants.DrivetrainConstants.ThetaGains;
import frc.robot.Constants.DrivetrainConstants.TranslationGains;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousCommandFactory {

    public static SwerveControllerCommand createSwerveControllerCommand(Trajectory trajectory,
            DrivetrainSubsystem drivetrain) {
        Constraints constraints = new TrapezoidProfile.Constraints(ThetaGains.kTurnToleranceRad,
                ThetaGains.kTurnRateToleranceRadPerS);
        ProfiledPIDController thetaController = new ProfiledPIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD,
                constraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, drivetrain::getPose,
                drivetrain.getKinematics(),
                new PIDController(TranslationGains.kP, TranslationGains.kI, TranslationGains.kD),
                new PIDController(TranslationGains.kP, TranslationGains.kI, TranslationGains.kD), thetaController,
                drivetrain::setDesiredStates, drivetrain);
        return swerveControllerCommand;
    }

    public static SwerveControllerCommand createAutonomous( DrivetrainSubsystem drivetrain){
        // Create a voltage constraint to ensure we don't accelerate too fast
        SwerveDriveKinematicsConstraint autoVoltageConstraint = new SwerveDriveKinematicsConstraint( drivetrain.getKinematics(),DrivetrainGeometry.MAX_VELOCITY_METERS_PER_SECOND );
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                DrivetrainGeometry.MAX_VELOCITY_METERS_PER_SECOND ,
                DrivetrainGeometry.MAX_VELOCITY_METERS_PER_SECOND / .33)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drivetrain.getKinematics())
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        Trajectory t = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(Math.PI)),
            // Pass config
            config);

        return createSwerveControllerCommand(t, drivetrain);
    }
}
