package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModuleIO_Real implements SwerveModuleIO {
   private final SparkMax m_drivingSparkMax;
   private final SparkMax m_turningSparkMax;

   private final RelativeEncoder m_drivingEncoder;
   private final AbsoluteEncoder m_turningEncoder;

   private final SparkClosedLoopController m_drivingPIDController;
   private final SparkClosedLoopController m_turningPIDController;

   private double chassisAngularOffset;

   private String name;

   public SwerveModuleIO_Real(int drivingCANId, int turningCANId, double chassisAngularOffset,
         String name) {

      int errors = 0;

      this.name = name;

      // m_drivingSparkMax = MotorUtil.createSparkMAX(drivingCANId,
      // MotorType.kBrushless,
      // Constants.NEO_CURRENT_LIMIT, SwerveModuleConstants.kDrivingEncoderInverted,
      // true, 0);
      // m_turningSparkMax = MotorUtil.createSparkMAX(turningCANId,
      // MotorType.kBrushless,
      // Constants.NEO550_CURRENT_LIMIT, true, 0);

      SparkMaxConfig drivingConfig = new SparkMaxConfig();
      drivingConfig
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
      drivingConfig.encoder
            .positionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor) // meters
            .velocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor); // meters per second
      drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(SwerveModuleConstants.kDrivingP, SwerveModuleConstants.kDrivingI, SwerveModuleConstants.kDrivingD)
            .velocityFF(SwerveModuleConstants.kDrivingFF)
            .outputRange(SwerveModuleConstants.kDrivingMinOutput, SwerveModuleConstants.kDrivingMaxOutput);
      m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
      m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      SparkMaxConfig turningConfig = new SparkMaxConfig();

      turningConfig
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
      turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(SwerveModuleConstants.kTurningEncoderInverted)
            .positionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor) // radians
            .velocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor); // radians per second
      turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(SwerveModuleConstants.kTurningP, SwerveModuleConstants.kTurningI, SwerveModuleConstants.kTurningD)
            .outputRange(SwerveModuleConstants.kTurningMinOutput, SwerveModuleConstants.kTurningMaxOutput)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput, SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

      m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
      m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            // errors += check(m_drivingSparkMax.enableVoltageCompensation(12));
      // errors += check(m_turningSparkMax.enableVoltageCompensation(12));

      // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
      m_drivingEncoder = m_drivingSparkMax.getEncoder();
      m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
      m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
      m_turningPIDController = m_turningSparkMax.getClosedLoopController();

      // errors += check(m_drivingPIDController.setFeedbackDevice(m_drivingEncoder));
      // errors += check(m_turningPIDController.setFeedbackDevice(m_turningEncoder));

      // Apply position and velocity conversion factors for the driving encoder. The
      // native units for position and velocity are rotations and RPM, respectively,
      // but we want meters and meters per second to use with WPILib's swerve APIs.

      // errors += check(
      // m_drivingEncoder
      // .setPositionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor));
      // errors += check(
      // m_drivingEncoder.setVelocityConversionFactor(
      // (SwerveModuleConstants.kDrivingEncoderVelocityFactor)));

      // Apply position and velocity conversion factors for the turning encoder. We
      // want these in radians and radians per second to use with WPILib's swerve
      // APIs.
      // errors += check(
      //       m_turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor));
      // errors += check(
      //       m_turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor));

      // Invert the turning encoder, since the output shaft rotates in the opposite
      // direction of
      // the steering motor in the MAXSwerve Module.

      // errors += check(m_turningEncoder.setInverted(SwerveModuleConstants.kTurningEncoderInverted));
      // m_drivingSparkMax.setInverted(SwerveModuleConstants.kDrivingEncoderInverted);

      // Enable PID wrap around for the turning motor. This will allow the PID
      // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
      // to 10 degrees will go through 0 rather than the other direction which is a
      // longer route.

      // errors += check(m_turningPIDController.setPositionPIDWrappingEnabled(true));
      // errors += check(m_turningPIDController
      //       .setPositionPIDWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput));
      // errors += check(m_turningPIDController
      //       .setPositionPIDWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput));

      // Set the PID gains for the driving motor
      // errors +=
      // check(m_drivingPIDController.setP(SwerveModuleConstants.kDrivingP));
      // errors +=
      // check(m_drivingPIDController.setI(SwerveModuleConstants.kDrivingI));
      // errors +=
      // check(m_drivingPIDController.setD(SwerveModuleConstants.kDrivingD));
      // errors +=
      // check(m_drivingPIDController.setFF(SwerveModuleConstants.kDrivingFF));
      // errors +=
      // check(m_drivingPIDController.setOutputRange(SwerveModuleConstants.kDrivingMinOutput,
      // SwerveModuleConstants.kDrivingMaxOutput));

      // Set the PID gains for the turning motor
      // errors += check(m_turningPIDController.setP(SwerveModuleConstants.kTurningP));
      // errors += check(m_turningPIDController.setI(SwerveModuleConstants.kTurningI));
      // errors += check(m_turningPIDController.setD(SwerveModuleConstants.kTurningD));
      // errors += check(m_turningPIDController.setFF(SwerveModuleConstants.kTurningFF));
      // errors += check(m_turningPIDController.setOutputRange(SwerveModuleConstants.kTurningMinOutput,
      //       SwerveModuleConstants.kTurningMaxOutput));

      this.chassisAngularOffset = chassisAngularOffset;

      errors += check(m_drivingEncoder.setPosition(0));

      if (errors > 0) {
         System.out.println("Swerve Module Errors! Name: " + name + ", Amount: " + errors);
      }
   }

   public void updateInputs(SwerveModuleIOInputs inputs) {
      inputs.driveBusVoltage = getDriveBusVoltage();
      inputs.driveAppliedVolts = getDriveBusVoltage() * getDriveOutput();
      inputs.drivePositionMeters = getDriveEncoderPosition();
      inputs.driveVelocityMPS = getDriveEncoderSpeedMPS();
      inputs.turnBusVoltage = getTurnBusVoltage();
      inputs.turnAppliedVolts = getTurnBusVoltage() * getTurnOutput();
      inputs.turnPositionRad = getTurnEncoderPosition();

   }

   public void setDriveEncoderPosition(double position) {
      m_drivingEncoder.setPosition(position);
   };

   public double getDriveEncoderPosition() {
      return m_drivingEncoder.getPosition();
   };

   public void setDesiredDriveSpeedMPS(double speed) {
      m_drivingPIDController.setReference(speed, ControlType.kVelocity);
   };

   public double getDriveEncoderSpeedMPS() {
      return m_drivingEncoder.getVelocity();
   };

   public double getTurnEncoderPosition() {
      return m_turningEncoder.getPosition();
   };

   public void setDesiredTurnAngle(double angle) {
      m_turningPIDController.setReference(angle, ControlType.kPosition);
   };

   public double getDriveBusVoltage() {
      return m_drivingSparkMax.getBusVoltage();
   }

   public double getDriveOutput() {
      return m_drivingSparkMax.getAppliedOutput();
   }

   public double getTurnBusVoltage() {
      return m_turningSparkMax.getBusVoltage();
   }

   public double getTurnOutput() {
      return m_turningSparkMax.getAppliedOutput();
   }

   public String getName() {
      return name;
   }

   public double getChassisAngularOffset() {
      return chassisAngularOffset;
   }

   private int check(REVLibError err) {
      return err == REVLibError.kOk ? 0 : 1;
   }

}
