package frc.robot.subsystems.climber;

public interface ClimberIO {
    public void setLeftSpeed(double speed); 
    public void setRightSpeed(double speed); 
    public boolean isLeftExtended();
    public boolean isRightExtended();
    public boolean isLeftRetracted(); 
    public boolean isRightRetracted(); 
    public double getLeftEncoderPosition(); 
    public double getRightEncoderPosition();
    public boolean isLeftSideStalling(); 
    public boolean isRightSideStalling();
    public void periodicUpdate();
}
