package frc.robot.subsystems;

public class SwerveMovement{
    public double forwardMovement;
    public double strideMovement;
    public double rotationClockwise;

    public SwerveMovement(double fwd, double str, double rcw){
        forwardMovement = fwd;
        strideMovement = str;
        rotationClockwise = rcw;
    }

    public SwerveMovement(SwerveMovement s){
        this(s.forwardMovement, s.strideMovement, s.rotationClockwise);
    }
}