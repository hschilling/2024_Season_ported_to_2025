package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
// import frc.utils.MotorUtil;

public class RealShooter implements ShooterIO {

    public static SparkMax shooterMotorControllerLow;
    public static SparkMax shooterMotorControllerHigh;
    public static RelativeEncoder shooterLowEncoder;
    public static RelativeEncoder shooterHighEncoder;
    public static SparkClosedLoopController shooterHighController;
    public static SparkClosedLoopController shooterLowController;

    public double feedforward = 0.011;
    public double pvalue = 0.01;
    public RealShooter()
    {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        config.encoder
                .positionConversionFactor(1) // convert to radians
                .velocityConversionFactor(1 / 60.0); // convert to rps
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(pvalue, 0, 0)
                .velocityFF(feedforward)
                .outputRange(0, 1);
        shooterMotorControllerLow = new SparkMax(ShooterConstants.SHOOT_LOW_MOTOR_ID, MotorType.kBrushless);
        shooterMotorControllerLow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterMotorControllerHigh = new SparkMax(ShooterConstants.SHOOT_HIGH_MOTOR_ID, MotorType.kBrushless);
        shooterMotorControllerHigh.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize motor encoder
        shooterLowEncoder = shooterMotorControllerLow.getEncoder();
        shooterHighEncoder = shooterMotorControllerHigh.getEncoder();
        // shooterHighEncoder.setVelocityConversionFactor(1/60.0); //convert to rps
        // shooterLowEncoder.setVelocityConversionFactor(1/60.0); //convert to rps

        //initialize PID controllers, set feedback device
        shooterHighController = shooterMotorControllerHigh.getClosedLoopController();
        shooterLowController = shooterMotorControllerLow.getClosedLoopController();
        //shooter cannot go backwards
        // shooterHighController.setOutputRange(0, 1);
        // shooterLowController.setOutputRange(0, 1);
        //set gains for PID controllers
        // shooterHighController.setFF(feedforward);
        // shooterHighController.setP(pvalue);
        // shooterLowController.setFF(feedforward);
        // shooterLowController.setP(pvalue);
    }

    //Basic shooting command
    @Override
    public void setMotor(double shootSpeed) {
        shooterHighController.setReference(shootSpeed * ShooterConstants.SHOOT_MAX_SPEED_RPS, ControlType.kVelocity);
        shooterLowController.setReference(shootSpeed * ShooterConstants.SHOOT_MAX_SPEED_RPS, ControlType.kVelocity);   
        SmartDashboard.putNumber("Shooter/shooter goal speed", shootSpeed * ShooterConstants.SHOOT_MAX_SPEED_RPS);
    }

    public double getCurrent()
    {
        return shooterMotorControllerHigh.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
        return shooterHighEncoder.getVelocity();
    }

    // @Override
    // public void setCurrentLimit(int current) {
    //     shooterMotorControllerHigh.setSmartCurrentLimit(current);        
    // }

    @Override
    public void periodicUpdate() {  
        SmartDashboard.putNumber("Shooter/shooter speed", getEncoderSpeed());
    }

}
