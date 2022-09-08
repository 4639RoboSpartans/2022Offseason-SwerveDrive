package frc.robot;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.lang.model.element.ExecutableElement;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class AutonPath {
    Trajectory trajectory = new Trajectory();
    public AutonPath(String filePath){
        try{
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filePath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch(IOException e){
            DriverStation.reportError("Unable to open trajectory: " + filePath, e.getStackTrace());
        }
    }

    public RamseteCommand getRameseteCommand(DriveSubsystem m_drive){
        // return new RamseteCommand(
        //     trajectory,
        //     m_drive::getPose,
        //     new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        //     new SimpleMotorFeedforward(
        //         Constants.ksVolts,
        //         Constants.kvVoltSecondsPerMeter,
        //         Constants.kaVoltSecondsSquaredPerMeter),
        //     Constants.kDriveKinematics,
        //     m_drive::getWheelSpeeds,
        //     new PIDController(Constants.kPDriveVel, 0, 0),
        //     new PIDController(Constants.kPDriveVel, 0, 0),
        //     m_drive::tankDriveVolts,
        //     m_drive);
        return null;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}