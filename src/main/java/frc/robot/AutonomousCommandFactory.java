package frc.robot;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DrivetrainConstants.ThetaGains;
import frc.robot.Constants.DrivetrainConstants.TranslationGains;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;

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

}
