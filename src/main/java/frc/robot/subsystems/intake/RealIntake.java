package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class RealIntake implements IntakeIO {

    public static SparkMax leftCenteringIntakeMotorController;
    public static SparkMax rightCenteringIntakeMotorController;
    public static SparkMax intakeMotorController;
    public static RelativeEncoder leftCenteringIntakeEncoder;
    public static RelativeEncoder rightCenteringIntakeEncoder;
    public static RelativeEncoder intakeEncoder;
    public static SparkClosedLoopController intakePIDController;
    private double FEEDFORWARD = 0.01;
    private double PVALUE = 0.01;

    public RealIntake()
    {
        // leftCenteringIntakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.LEFT_CENTERING_MOTOR_ID, MotorType.kBrushless, 
        //     Constants.NEO550_CURRENT_LIMIT, true, true, slewRate);
        
        // rightCenteringIntakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.RIGHT_CENTERING_MOTOR_ID, MotorType.kBrushless, 
        //     Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);

        // intakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.INTAKE_CENTERING_ID, MotorType.kBrushless,
        // Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);


        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
            config.encoder
                .positionConversionFactor(1) // convert to radians
                .velocityConversionFactor(1 / 60.0); // convert to rps
            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(PVALUE, 0, 0)
                .velocityFF(FEEDFORWARD);
        leftCenteringIntakeMotorController = new SparkMax(ShooterConstants.SHOOT_LOW_MOTOR_ID, MotorType.kBrushless);
        leftCenteringIntakeMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.inverted(false);
        rightCenteringIntakeMotorController = new SparkMax(ShooterConstants.SHOOT_LOW_MOTOR_ID, MotorType.kBrushless);
        rightCenteringIntakeMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotorController = new SparkMax(ShooterConstants.SHOOT_LOW_MOTOR_ID, MotorType.kBrushless);
        intakeMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // initialize motor encoder
        leftCenteringIntakeEncoder = leftCenteringIntakeMotorController.getEncoder();
        rightCenteringIntakeEncoder = rightCenteringIntakeMotorController.getEncoder();
        intakeEncoder = intakeMotorController.getEncoder();
        
        // intakeEncoder.setVelocityConversionFactor(1/60.0); //convert to rps

        intakePIDController = intakeMotorController.getClosedLoopController();
        // intakePIDController.setFeedbackDevice(intakeEncoder);
        
        // intakePIDController.setFF(FEEDFORWARD);
        // intakePIDController.setP(PVALUE);

        // leftCenteringIntakeMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 32767);
        // leftCenteringIntakeMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 32767);
        // leftCenteringIntakeMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus4, 32767);
        // leftCenteringIntakeMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus5, 32767);
        // leftCenteringIntakeMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus6, 32767);
        // leftCenteringIntakeMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 32767);

        





    }

    @Override
    public void setMotor(double intakeSpeed) {
        leftCenteringIntakeMotorController.set(intakeSpeed);
        rightCenteringIntakeMotorController.set(intakeSpeed);
        intakeMotorController.set(intakeSpeed);
    }

    public double getLeftCurrent()
    {
        return rightCenteringIntakeMotorController.getOutputCurrent();
    }

    public double getRightCurrent()
    {
        return leftCenteringIntakeMotorController.getOutputCurrent();
    }

    @Override
    public double getLeftEncoderSpeed() {
        return leftCenteringIntakeEncoder.getVelocity();
    }

    @Override
    public double getRightEncoderSpeed() {
        return rightCenteringIntakeEncoder.getVelocity();
    }

    public double getIntakeEncoderSpeed() {
        return intakeEncoder.getVelocity();
    }

    @Override
    public double getLeftEncoderPosition() {
        return leftCenteringIntakeEncoder.getPosition();
    }

    @Override
    public double getRightEncoderPosition() {
        return rightCenteringIntakeEncoder.getPosition();
    }

    // @Override
    // public void setLeftCurrentLimit(int current) {
    //     leftCenteringIntakeMotorController.setSmartCurrentLimit(current);        
    // }

    // @Override
    // public void setRightCurrentLimit(int current) {
    //     rightCenteringIntakeMotorController.setSmartCurrentLimit(current);        
    // }

    @Override
    public void periodicUpdate() {
    }
}
