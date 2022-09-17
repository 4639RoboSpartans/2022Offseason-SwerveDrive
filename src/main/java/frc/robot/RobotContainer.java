// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.io.File;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.*;
import com.pathplanner.lib.commands.*;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public OI m_OI = new OI();
    public DriveSubsystem m_drive = new DriveSubsystem();
    public DriveCommand drive_cmd = new DriveCommand(m_drive, m_OI);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        m_drive.setDefaultCommand(drive_cmd);
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        final double kPXController = 1.5;
        final double kPYController = 1.5;
        final double kPThetaController = 3.0;

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("TestPath1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

        if(trajectory == null){
            throw new Error("Trajectory could not be loaded");
        }

        PIDController xController = new PIDController(kPXController, 0, 0);
        PIDController yController = new PIDController(kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, 0, 0, new TrapezoidProfile.Constraints(Constants.kMaxAngularSpeedRadiansPerSecond, Constants.kMaxAngularAccelerationRadiansPerSecondSquared)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var v = new PPSwerveControllerCommand(
            trajectory,
            m_drive::getPose,
            Constants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_drive::setModuleStates,
            m_drive 
        );

        return new SequentialCommandGroup(
            new InstantCommand(() -> m_drive.resetOdometry(trajectory.getInitialPose())),
            v,//swerveControllerCommand,
            new InstantCommand(() -> m_drive.stop()));
    }
}
