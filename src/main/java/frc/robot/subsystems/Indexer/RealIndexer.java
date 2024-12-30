package frc.robot.subsystems.Indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
// import frc.utils.MotorUtil;

public class RealIndexer implements IndexerIO {

    public static SparkMax indexerMotorController;
    public static RelativeEncoder indexerEncoder;
    public static SparkClosedLoopController indexerController;
    // private double slewRate = 0;
    public boolean isIntooked = false;
    public boolean isSensorOverriden = false;
    // private static DigitalInput indexerSensorTop;
    private static DigitalInput indexerSensorBottom;

    public RealIndexer() {
        // indexerMotorController = MotorUtil.createSparkMAX(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless,
        //         Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);

                // inverted, isidlebreak

        indexerMotorController = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
        // config.encoder
        //         .positionConversionFactor(drivingFactor) // meters
        //         .velocityConversionFactor(drivingFactor / 60.0); // meters per second
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                // These are example gains you may need to them for your own robot!
                // .pid(0.04, 0, 0)
                // .velocityFF(drivingVelocityFeedForward)
                // .outputRange(-1, 1);


        indexerEncoder = indexerMotorController.getEncoder();
        // indexerSensorTop = new DigitalInput(IndexerConstants.INDEXER_SENSOR_CHANNEL_TOP);
        indexerSensorBottom = new DigitalInput(IndexerConstants.INDEXER_SENSOR_CHANNEL_BOTTOM);

        // config.signals.primaryEncoderPositionPeriodMs(5);

        // indexerMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 32767);
        // indexerMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 32767);
        // indexerMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus4, 32767);
        // indexerMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus5, 32767);
        // indexerMotorController.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus6, 32767);

        indexerMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void setMotor(double indexerSpeed) {
        indexerMotorController.set(indexerSpeed);
    }

    public double getCurrent() {
        return indexerMotorController.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
        return indexerEncoder.getVelocity();
    }

    @Override
    public double getEncoderPosition() {
        return indexerEncoder.getPosition();
    }

    // @Override
    // public void setCurrentLimit(int current) {
    //     indexerMotorController.setSmartCurrentLimit(current);
    // }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putBoolean("indexer/isIntooked:", isIntooked);
        SmartDashboard.putBoolean("indexer/getIsBeamBroken", getIsBeamBroken());
    }

    @Override
    public boolean getIsBeamBroken() {
        if (isSensorOverriden) {
            return false;
        } else {
            return !indexerSensorBottom.get();
        }
    }

    @Override
    public void setIsOverride() {
        isSensorOverriden = !isSensorOverriden;
    }
}
