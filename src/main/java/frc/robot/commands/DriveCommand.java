package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveMovement;

public class DriveCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private OI oiSubsystem;

    public DriveCommand(DriveSubsystem driveSubsystem, OI oiSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.oiSubsystem = oiSubsystem;

        addRequirements(driveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double wa1 = 0, wa2 = 0, wa3 = 0, wa4 = 0;

        printValues();

        SwerveMovement rawSwerveMovement = getRawMovement();
        SwerveMovement swerveMovement = toRobotCentric(rawSwerveMovement);

        double diagonalLength = Math.sqrt(Math.pow(Constants.trackwidth, 2) + Math.pow(Constants.wheelbase, 2));

        // TODO: refactor these variable names to be more meaningful
        double A = swerveMovement.strideMovement - swerveMovement.rotationClockwise * (Constants.wheelbase / diagonalLength);
        double B = swerveMovement.strideMovement + swerveMovement.rotationClockwise * (Constants.wheelbase / diagonalLength);
        double C = swerveMovement.forwardMovement - swerveMovement.rotationClockwise * (Constants.trackwidth / diagonalLength);
        double D = swerveMovement.forwardMovement + swerveMovement.rotationClockwise * (Constants.trackwidth / diagonalLength);


        double ws1 = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        double ws2 = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double ws3 = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double ws4 = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        if (ws3 > 0.05 || ws3 < -0.05) {
            wa1 = Math.atan2(B, C) * 180 / Math.PI;
        }
        if (ws3 > 0.05 || ws3 < -0.05) {
            wa2 = Math.atan2(B, D) * 180 / Math.PI;
        }
        if (ws3 > 0.05 || ws3 < -0.05) {
            wa3 = Math.atan2(A, D) * 180 / Math.PI;
        }
        if (ws3 > 0.05 || ws3 < -0.05) {
            wa4 = Math.atan2(A, C) * 180 / Math.PI;
        }

        // 1 is FR, 2 is FL, 3 is RL, 4 is RR

        double max = ws1;
        if (ws2 > max)
            max = ws2;
        if (ws3 > max)
            max = ws3;
        if (ws4 > max)
            max = ws4;
        if (max > 1) {
            ws1 /= max;
            ws2 /= max;
            ws3 /= max;
            ws4 /= max;
        }
        // ws4 = ws3 = ws2 = ws1;
        SmartDashboard.putNumber("Speed", ws3);
        SmartDashboard.putNumber("Rotation", wa3);

        driveSubsystem.setModule1(ws1 * 0.4, -wa1);
        driveSubsystem.setModule2(ws2 * 0.4, -wa2);
        driveSubsystem.setModule3(ws3 * 0.4, -wa3);
        driveSubsystem.setModule4(ws4 * 0.4, -wa4);
    }

    public SwerveMovement getRawMovement(){
        double rawForwardsMovement = oiSubsystem.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        double rawStrideMovement = oiSubsystem.getAxis(0, Constants.Axes.LEFT_STICK_X);
        double rotationCW = oiSubsystem.getAxis(0, Constants.Axes.RIGHT_STICK_X);

        return new SwerveMovement(rawForwardsMovement, rawStrideMovement, rotationCW);
    }

    public SwerveMovement toRobotCentric(SwerveMovement movement) {
        SwerveMovement res = new SwerveMovement(movement);

        double heading = driveSubsystem.getHeadingRadians();

        res.forwardMovement = movement.strideMovement * Math.sin(heading) + movement.forwardMovement * Math.cos(heading);
        res.strideMovement  = movement.strideMovement * Math.cos(heading) - movement.forwardMovement * Math.sin(heading);

        return res;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void printValues() {
        SmartDashboard.putNumber("Encoder1", driveSubsystem.SwerveMod1FrontRight.getRotationInDegrees());
        SmartDashboard.putNumber("Encoder2", driveSubsystem.SwerveMod2FrontLeft.getRotationInDegrees());
        SmartDashboard.putNumber("Encoder3", driveSubsystem.SwerveMod3RearLeft.getRotationInDegrees());
        SmartDashboard.putNumber("Encoder4", driveSubsystem.SwerveMod4RearRight.getRotationInDegrees());
        SmartDashboard.putNumber("RobotHeading", driveSubsystem.getHeading());
        SmartDashboard.putNumber("LeftStickX", oiSubsystem.getAxis(0, Constants.Axes.LEFT_STICK_X));
        SmartDashboard.putNumber("LeftStickY", oiSubsystem.getAxis(0, Constants.Axes.LEFT_STICK_Y));
        SmartDashboard.putNumber("RightStickX", oiSubsystem.getAxis(0, Constants.Axes.RIGHT_STICK_X));
        SmartDashboard.putNumber("RightStickY", oiSubsystem.getAxis(0, Constants.Axes.RIGHT_STICK_Y));
    }
}
