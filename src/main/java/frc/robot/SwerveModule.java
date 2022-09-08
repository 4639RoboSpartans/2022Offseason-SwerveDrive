package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.*;

public class SwerveModule {
    private WPI_TalonFX driver;
    private WPI_TalonFX rotator;
    private CANCoder modEnc;
    private PIDController pid;
    private double kp=0.08;//0.07;
    private double ki=0.15;
    private int kd=0;
    public SwerveModule(int driverInd, int rotateInd, int channel){
        driver = new WPI_TalonFX(driverInd);
        rotator = new WPI_TalonFX(rotateInd);
        modEnc = new CANCoder(channel);
        driver.configFactoryDefault();
        rotator.configFactoryDefault();
        driver.setNeutralMode(NeutralMode.Coast);
        rotator.setNeutralMode(NeutralMode.Coast);
        driver.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        modEnc = new CANCoder(channel);
        pid = new PIDController(kp, ki, kd);
        pid.setTolerance(1);

        modEnc.setPosition(0);
    }
    public void setSpeed(double speed){
        driver.set(speed);
    }
    public double getDegrees(){
        return (modEnc.getPosition() % 360 + 540) % 360 - 180;
        // return modEnc.getPosition();
    
        // return ((rotator.getSelectedSensorPosition()%2048)/2048)*360;
    }
    public void setVoltage(double voltage){
        driver.setVoltage(voltage);
    }
    public void setDegrees(double degrees){
        rotator.setVoltage(-pid.calculate(getDegrees(), degrees));
    }
    
    public double angleDiff(double a1, double a2){
        return ((a1 - a2) % 180 + 270) % 180 - 90;
    }

    public boolean shouldReverseSpeed(double a1, double a2){
        double delta = ((a1 - a2) % 360 + 540) % 360 - 180;
        return Math.abs(delta) >= 90;
    }

    public void set(double speed, double degrees){
        SmartDashboard.putNumber("targetAngle", degrees);
        //Find the least effort to rotate to "degrees"
        double currDegrees = getDegrees();
        // var optimized = SwerveModuleState.optimize(
        //     new SwerveModuleState(
        //         speed, new Rotation2d(Math.toRadians(degrees))
        //     ), 
        //     new Rotation2d(Math.toRadians(currDegrees))
        // );

        double delta = angleDiff(currDegrees, degrees);
        boolean shouldReverse = shouldReverseSpeed(currDegrees, degrees);
        // if(delta > 0){
        //     degrees = currDegrees + 20;
        // }%
        // else{
        //     degrees = currDegrees - 20;
        // }

        //if(shouldReverse) //speed *= -1;
        speed *= shouldReverse ? -1 : 1;
        degrees = currDegrees - delta;

        // if(Math.abs(delta) <= 90){}
        // else if(Math.abs(delta) <= 270){
        //     // speed *= -1;
        //     degrees += 180 * (delta > 0 ? 1 : -1);
        // }
        // else {
        //     degrees += 360 * (delta > 0 ? 1 : -1);
        // }

        SmartDashboard.putNumber("currentAngle", getDegrees());
        SmartDashboard.putNumber("setpointAngle", degrees);

        SmartDashboard.putNumber("delta", delta);

        SmartDashboard.putNumber("targetSpeed", speed);

        // setSpeed(optimized.speedMetersPerSecond);
        // setDegrees(optimized.angle.getDegrees());

        setSpeed(speed);
        setDegrees(degrees);
    }

    public double getVelocity(){
        return driver.getSelectedSensorVelocity();
    }
    public void stop(){
        driver.setVoltage(0);
        rotator.setVoltage(0);
    }
    public void resetEncoder(){
        modEnc.setPosition(0);
    }
}
