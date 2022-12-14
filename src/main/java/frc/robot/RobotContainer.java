// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
        m_OI.getButton(0, Constants.Buttons.LEFT_STICK).whenHeld(new RunCommand(() -> {
            
            SmartDashboard.putNumber("Encoder1", m_drive.SwerveMod1FrontRight.getRotationInDegrees());
            SmartDashboard.putNumber("Encoder2", m_drive.SwerveMod2FrontLeft.getRotationInDegrees());
            SmartDashboard.putNumber("Encoder3", m_drive.SwerveMod3RearLeft.getRotationInDegrees());
            SmartDashboard.putNumber("Encoder4", m_drive.SwerveMod4RearRight.getRotationInDegrees());
            SmartDashboard.putNumber("RobotHeading", m_drive.getHeading());
            SmartDashboard.putNumber("LeftStickX", m_OI.getAxis(0, Constants.Axes.LEFT_STICK_X));
            SmartDashboard.putNumber("LeftStickY", m_OI.getAxis(0, Constants.Axes.LEFT_STICK_Y));
            SmartDashboard.putNumber("RightStickX", m_OI.getAxis(0, Constants.Axes.RIGHT_STICK_X));
            SmartDashboard.putNumber("RightStickY", m_OI.getAxis(0, Constants.Axes.RIGHT_STICK_Y));
            
            m_drive.resetAnglesAndPositions();
        }, m_drive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // final double kPXController = 1.5;
        // final double kPYController = 1.5;
        // final double kPThetaController = 3.0;

        // PathPlannerTrajectory trajectory = PathPlanner.loadPath("TestPath1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

        // if(trajectory == null){
        //     throw new Error("Trajectory could not be loaded");
        // }

        // PIDController xController = new PIDController(kPXController, 0, 0);
        // PIDController yController = new PIDController(kPYController, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //     kPThetaController, 0, 0, new TrapezoidProfile.Constraints(Constants.kMaxAngularSpeedRadiansPerSecond, Constants.kMaxAngularAccelerationRadiansPerSecondSquared)
        // );
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // var v = new PPSwerveControllerCommand(
        //     trajectory,
        //     m_drive::getPose,
        //     Constants.kDriveKinematics,
        //     xController,
        //     yController,
        //     thetaController,
        //     m_drive::setModuleStates,
        //     m_drive 
        // );

        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> m_drive.resetOdometry(trajectory.getInitialPose())),
        //     v,//swerveControllerCommand,
        //     new InstantCommand(() -> m_drive.stop()));
        return null;
    }
}
