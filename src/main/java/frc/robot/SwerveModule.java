package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class SwerveModule {
    private WPI_TalonFX driver;
    private WPI_TalonFX rotator;
    private PIDController pid;
    private int kp=0;
    private int ki=0;
    private int kd=0;
    public SwerveModule(int driverInd, int rotateInd){
        driver = new WPI_TalonFX(driverInd);
        rotator = new WPI_TalonFX(rotateInd);
        driver.configFactoryDefault();
        rotator.configFactoryDefault();
        driver.setNeutralMode(NeutralMode.Coast);
        rotator.setNeutralMode(NeutralMode.Coast);
        driver.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rotator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        pid = new PIDController(kp, ki, kd);
    }
    public void setSpeed(double speed){
        driver.set(speed);
    }
    public double getDegrees(){
        return ((rotator.getSelectedSensorPosition()%2048)/2048)*360;
    }
    public void setVoltage(double voltage){
        driver.setVoltage(voltage);
    }
    public void setDegrees(double degrees){
        rotator.setVoltage(pid.calculate(getDegrees(), degrees));
    }
    public double getVelocity(){
        return driver.getSelectedSensorVelocity();
    }
    public void stop(){
        driver.setVoltage(0);
        rotator.setVoltage(0);
    }
}
