package frc.robot.subsystems.arm;

import org.photonvision.PhotonUtils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionConstants;
// import frc.utils.MotorUtil;

public class RealArm implements ArmIO {
    private static SparkMax armMotorControllerLeft;
    private static SparkMax armMotorControllerRight;
    public static AbsoluteEncoder armAbsoluteEncoderRight;
    public static RelativeEncoder armEncoderRight;
    public static double speedFromArmHeight;
    final static double EIGHTYSLOPE = VisionConstants.EIGHTYMODELSLOPE;
    final static double EIGHTYINTERCEPT = VisionConstants.EIGHTYMODELINTERCEPT;
    final static double HUNDREDSLOPE = VisionConstants.HUNDREDMODELSLOPE;
    final static double HUNDREDINTERCEPT = VisionConstants.HUNDREDMODELINTERCEPT;
    final static double BOUNDARY = VisionConstants.EIGHTYMODELRANGE;
    final static double STAYDOWNBOUNDARY = VisionConstants.STAYDOWNBOUNDARY;
    final static double DISTANCE_TO_CENTER_FROM_FRAME_INCHES = 15.75;

    public RealArm() {
        // make the motor controllers
        // armMotorControllerRight = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID_RIGHT, MotorType.kBrushless,
        //         Constants.NEO_CURRENT_LIMIT,
        //         true, true, 0);
        // armMotorControllerLeft = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID_LEFT, MotorType.kBrushless,
        //         Constants.NEO_CURRENT_LIMIT,
        //         false, true, 0);

        SparkMaxConfig configRight = new SparkMaxConfig();
        configRight
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        configRight.encoder
                .positionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION) // convert to radians
                .velocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60); // convert to rps
        configRight.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                // These are example gains you may need to them for your own robot!
                // .pid(pvalue, 0, 0)
                // .velocityFF(feedforward)
                // .outputRange(0, 1);
        configRight.absoluteEncoder
                .positionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION) // convert to radians
                .velocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60) // convert to rps
                .inverted(true)
                .zeroOffset(ArmConstants.ARM_ABSOLUTE_MEASURED - ArmConstants.ARM_ABSOLUTE_CAD);
        armMotorControllerRight = new SparkMax(ArmConstants.ARM_MOTOR_ID_RIGHT, MotorType.kBrushless);
        armMotorControllerRight.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig configLeft = new SparkMaxConfig();
        configRight.follow(armMotorControllerRight, true)
                .idleMode(IdleMode.kBrake)
                // .inverted(true)
                .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        armMotorControllerLeft = new SparkMax(ArmConstants.ARM_MOTOR_ID_RIGHT, MotorType.kBrushless);
        armMotorControllerLeft.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // make the encoders
        // .044
        armAbsoluteEncoderRight = armMotorControllerRight.getAbsoluteEncoder();
        armEncoderRight = armMotorControllerRight.getEncoder();

        // setting position/velocity conversion factors, offsets
        // armAbsoluteEncoderRight.setPositionConversionFactor(ArmConstants.ABSOLUTE_RADIANS_PER_REVOLUTION);
        // armAbsoluteEncoderRight.setVelocityConversionFactor(ArmConstants.ABSOLUTE_RADIANS_PER_REVOLUTION
        // / 60);
        // armAbsoluteEncoderRight.setInverted(true);
        // armAbsoluteEncoderRight.setZeroOffset(0);
        // armAbsoluteEncoderRight.setZeroOffset(ArmConstants.ARM_ABSOLUTE_MEASURED -
        // ArmConstants.ARM_ABSOLUTE_CAD);
        // armEncoderRight.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        // armEncoderRight.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION
        // / 60);
        armEncoderRight.setPosition(armAbsoluteEncoderRight.getPosition());

        // set the left motor to follow the right one, but inverted since left isn't
        // reversed and right is
        // armMotorControllerLeft.follow(armMotorControllerRight, true);
    }

    public double getAbsoluteEncoderPosition() {
        return armAbsoluteEncoderRight.getPosition();
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("arm/arm position", getEncoderPosition());
        SmartDashboard.putNumber("arm/arm absolute position", getAbsoluteEncoderPosition());
    }

    @Override
    public double getEncoderPosition() {
        return armEncoderRight.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return armEncoderRight.getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorControllerRight.set(speed);
    }

    @Override
    public double getArmCurrent() {
        return armMotorControllerRight.getOutputCurrent();
    }

    @Override
    public void setEncoderPosition(double angle) {
        armEncoderRight.setPosition(angle);
    }

    public double getDesiredArmAngle(Pose2d robotPose, Pose2d speakerPose) {
        double distToSpeaker;
        double desiredArmAngleRadians;
        distToSpeaker = PhotonUtils.getDistanceToPose(robotPose, speakerPose);
        if (distToSpeaker <= STAYDOWNBOUNDARY) {
            desiredArmAngleRadians = 0.31;
        } else if (distToSpeaker <= BOUNDARY) {
            desiredArmAngleRadians = EIGHTYSLOPE * (distToSpeaker) + EIGHTYINTERCEPT;
        } else {
            desiredArmAngleRadians = HUNDREDSLOPE * (distToSpeaker) + HUNDREDINTERCEPT;
        }
        return desiredArmAngleRadians;
    }
}
