package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private WPI_TalonFX driver;
    private WPI_TalonFX rotator;
    private CANCoder modEnc;
    private PIDController pid;

    private double kp=0.05;//0.07;
    private double ki=0.1;
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

        modEnc.setPosition(0);
    }

    public void setSpeed(double speed){
        driver.set(speed);
    }

    public double getDegrees(){
        return (modEnc.getPosition() % 360 + 540) % 360 - 180;
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
        // Do not change any motor positions if speed is wheel speed is negligible.
        if(speed < 0.001){
            stop();
            return;
        }

        SmartDashboard.putNumber("targetAngle", degrees);

        //Find the least effort to rotate to "degrees"

        double currDegrees = getDegrees();

        double delta = angleDiff(currDegrees, degrees);
        boolean shouldReverse = shouldReverseSpeed(currDegrees, degrees);

        speed *= shouldReverse ? -1 : 1;
        degrees = currDegrees - delta;

        SmartDashboard.putNumber("currentAngle", getDegrees());
        SmartDashboard.putNumber("setpointAngle", degrees);

        SmartDashboard.putNumber("delta", delta);

        SmartDashboard.putNumber("targetSpeed", speed);

        setSpeed(speed);
        setDegrees(degrees);
    }

    public double getVelocity(){
        return driver.getSelectedSensorVelocity();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getDegrees()));
    }

    public void stop(){
        driver.setVoltage(0);
        rotator.setVoltage(0);
    }
    public void resetEncoder(){
        modEnc.setPosition(0);
    }
}
