package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase{
    private DriveSubsystem m_drive;
    private OI m_oi;
    double FWD, STR, RCW;
    public DriveCommand(DriveSubsystem m_drive, OI oi){
        this.m_drive =m_drive;
        m_oi = oi;
        addRequirements(m_drive);
    }
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        printValues();
        FWD = m_oi.getAxis(1, Constants.Axes.LEFT_STICK_Y);
        STR = m_oi.getAxis(1, Constants.Axes.LEFT_STICK_X);
        RCW = m_oi.getAxis(1, Constants.Axes.RIGHT_STICK_X);
        double temp = FWD*Math.cos(m_drive.getHeadingRadians())+STR*Math.sin(m_drive.getHeadingRadians());
        STR = FWD*Math.sin(m_drive.getHeadingRadians())+STR*Math.cos(m_drive.getHeadingRadians());
        FWD = temp;

        double R = Math.sqrt(Math.pow(Constants.trackwidth,2)+Math.pow(Constants.wheelbase,2));
        double A = STR-RCW*(Constants.wheelbase/R);
        double B = STR+RCW*(Constants.wheelbase/R);
        double C = FWD-RCW*(Constants.trackwidth/R);
        double D = FWD+RCW*(Constants.trackwidth/R);

        double ws1 = Math.sqrt(Math.pow(B,2)+Math.pow(C,2));
        double ws2 = Math.sqrt(Math.pow(B,2)+Math.pow(D,2));
        double ws3 = Math.sqrt(Math.pow(A,2)+Math.pow(D,2));
        double ws4 = Math.sqrt(Math.pow(A,2)+Math.pow(C,2));

        double wa1 = Math.atan2(B,C)*180/Math.PI;
        double wa2 = Math.atan2(B,D)*180/Math.PI;
        double wa3 = Math.atan2(A,D)*180/Math.PI;
        double wa4 = Math.atan2(A,C)*180/Math.PI;
        //1 is FR, 2 is FL, 3 is RL, 4 is RR

        if(wa1<0){
            wa1+=360;
        }
        if(wa2<0){
            wa2+=360;
        }
        if(wa3<0){
            wa3+=360;
        }
        if(wa4<0){
            wa4+=360;
        }

        double max = ws1;
        if(ws2>max)max=ws2; 
        if(ws3>max)max=ws3; 
        if(ws4>max)max=ws4;
        if(max>1)   {
            ws1/=max; 
            ws2/=max; 
            ws3/=max; 
            ws4/=max;
        } 
        // m_drive.setModule1(ws1, wa1);
        // m_drive.setModule2(ws2, wa2);
        // m_drive.setModule3(ws3, wa3);
        // m_drive.setModule4(ws4, wa4);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    public void printValues(){
        SmartDashboard.putNumber("Encoder1", m_drive.SwerveMod1FrontRight.getDegrees());
        SmartDashboard.putNumber("Encoder2", m_drive.SwerveMod2FrontLeft.getDegrees());
        SmartDashboard.putNumber("Encoder3", m_drive.SwerveMod3RearLeft.getDegrees());
        SmartDashboard.putNumber("Encoder4", m_drive.SwerveMod4RearRight.getDegrees());
        SmartDashboard.putNumber("RobotHeading", m_drive.getHeading());
        SmartDashboard.putNumber("LeftStickX", m_oi.getAxis(0, Constants.Axes.LEFT_STICK_X));
        SmartDashboard.putNumber("LeftStickY", m_oi.getAxis(0, Constants.Axes.LEFT_STICK_Y));
        SmartDashboard.putNumber("RightStickX", m_oi.getAxis(0, Constants.Axes.RIGHT_STICK_X));
        SmartDashboard.putNumber("RightStickY", m_oi.getAxis(0, Constants.Axes.RIGHT_STICK_Y));
    }
}
