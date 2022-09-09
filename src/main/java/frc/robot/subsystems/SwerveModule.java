package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

        driver.configFactoryDefault();
        rotator.configFactoryDefault();

        driver.setNeutralMode(NeutralMode.Coast);
        rotator.setNeutralMode(NeutralMode.Coast);

        driver.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        modEnc = new CANCoder(channel);

        pid = new PIDController(kp, ki, kd);

        modEnc.setPosition(0);
    }

    public double getRotationInDegrees(){
        return modToRange(modEnc.getPosition(), -180, 180);
    }

    private void setSpeed(double speed){
        driver.set(speed);
    }
    public void setDegrees(double degrees){
        rotator.setVoltage(-pid.calculate(getRotationInDegrees(), degrees));
    }

    public double getVelocity(){
        return driver.getSelectedSensorVelocity();
    }

    public double getTurningVelocity(){
        return modEnc.getVelocity();
    }

    public void resetEncoder(){
        modEnc.setPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getRotationInDegrees()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(isNegligible(state)){
            stop();
        }
        else{
            SwerveModuleState optimized = optimize(state);

            setSpeed(optimized.speedMetersPerSecond);
            setDegrees(optimized.angle.getDegrees());
        }
    }
    
    private static boolean isNegligible(SwerveModuleState state){
        return state.speedMetersPerSecond < 0.001;
    }

    private SwerveModuleState optimize(SwerveModuleState state){
        double degrees = state.angle.getDegrees();
        double speed = state.speedMetersPerSecond;

        double currDegrees = getRotationInDegrees();
        double rawDelta = currDegrees - degrees;

        double delta = modToRange(rawDelta, -90, 90);
        boolean shouldReverse = Math.abs(modToRange(rawDelta, -180, 180)) >= 90;

        double finalAngle = currDegrees - delta;
        double finalSpeed = shouldReverse ? -speed : speed;

        return new SwerveModuleState(finalSpeed, Rotation2d.fromDegrees(finalAngle));
    }

    private static double modToRange(double x, double min, double max){
        double range = max - min;
        return ((x - min) % range + range) % range + min;
    }

    public void stop(){
        driver.setVoltage(0);
        rotator.setVoltage(0);
    }
}
