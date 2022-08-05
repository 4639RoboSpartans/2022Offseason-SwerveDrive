package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
    public SwerveModule SwerveMod1FrontRight;
    public SwerveModule SwerveMod2FrontLeft;
    public SwerveModule SwerveMod3RearLeft;
    public SwerveModule SwerveMod4RearRight;

    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );


    public AHRS navx = new AHRS();
    public SwerveDriveOdometry m_odometry;
    public DriveSubsystem(){
        SwerveMod1FrontRight = new SwerveModule(Constants.DriverMotor1, Constants.RotaterMotor1);
        SwerveMod2FrontLeft = new SwerveModule(Constants.DriverMotor2, Constants.RotaterMotor2);
        SwerveMod3RearLeft = new SwerveModule(Constants.DriverMotor3, Constants.RotaterMotor3);
        SwerveMod4RearRight = new SwerveModule(Constants.DriverMotor4, Constants.RotaterMotor4);
        navx.calibrate();
        m_odometry=new SwerveDriveOdometry(m_kinematics,
        navx.getRotation2d());
    }
    public void zeroHeading() {
        navx.reset();
    }
    public double getHeading() {
        if(navx.getAngle()%360<0){
            return (navx.getAngle()%360)+360;
        }
        return navx.getAngle()%360;
    }
    public double getHeadingRadians(){
        return Math.toRadians(getHeading());
    }
    public void stop(){
        SwerveMod1FrontRight.stop();
        SwerveMod2FrontLeft.stop();
        SwerveMod3RearLeft.stop();
        SwerveMod4RearRight.stop();
    }
    public void setModule1(double speed, double rotation){
        SwerveMod1FrontRight.setSpeed(speed);
        SwerveMod1FrontRight.setDegrees(rotation);
    }
    public void setModule2(double speed, double rotation){
        SwerveMod2FrontLeft.setSpeed(speed);
        SwerveMod2FrontLeft.setDegrees(rotation);
    }
    public void setModule3(double speed, double rotation){
        SwerveMod3RearLeft.setSpeed(speed);
        SwerveMod3RearLeft.setDegrees(rotation);
    }
    public void setModule4(double speed, double rotation){
        SwerveMod4RearRight.setSpeed(speed);
        SwerveMod4RearRight.setDegrees(rotation);
    }
}
