package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

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