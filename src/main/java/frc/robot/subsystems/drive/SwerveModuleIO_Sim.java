package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.utils.SimEncoder;

public class SwerveModuleIO_Sim implements SwerveModuleIO {

   // Simulation Motors
   // private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), Constants.SwerveModuleConstants.kDrivingMotorReduction,
   //       0.025);
   // private DCMotorSim turnMotor = new DCMotorSim(DCMotor.getNeo550(1), Constants.SwerveModuleConstants.kTurningMotorReduction,
   //       0.025);



   private DCMotor gearbox = DCMotor.getNEO(1);
   LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(gearbox, 1, 
         Constants.SwerveModuleConstants.kDrivingMotorReduction);
   private DCMotorSim driveMotor = new DCMotorSim(plant, gearbox);
   private DCMotorSim turnMotor = new DCMotorSim(plant, gearbox);


   private SimEncoder m_turningEncoder;
   private SimEncoder m_drivingEncoder;

   // Variables to track various module information
   private String name;

   // Sim-Tuned PID Controllers
   private PIDController m_turningPIDController = new PIDController(10, 0.0, 0.0);
   private PIDController m_drivingPIDController = new PIDController(1, 0.0, 0.0);

   private double driveMotorOutput = 0.0;
   private double turnMotorOutput = 0.0;

   public SwerveModuleIO_Sim(String name) {
      m_drivingEncoder = new SimEncoder(name + " drive encoder");
      m_turningEncoder = new SimEncoder(name + " turn encoder");
      this.name = name;
   }

   public void updateInputs(SwerveModuleIOInputs inputs) {
      driveMotor.update(1.0/Constants.CodeConstants.kMainLoopFrequency);
      turnMotor.update(1.0/Constants.CodeConstants.kMainLoopFrequency);
      m_drivingEncoder.setDistance(driveMotor.getAngularPositionRotations() );
      m_drivingEncoder.setSpeed(driveMotor.getAngularVelocityRadPerSec() );
      m_turningEncoder.setDistance(turnMotor.getAngularPositionRad() );
   };

   public void setDriveEncoderPosition(double position) {
      m_drivingEncoder.setDistance(position);      
   };

   public double getDriveEncoderPosition() {
      return m_drivingEncoder.getDistance();
   };

   public double getDriveEncoderSpeedMPS() {
      return m_drivingEncoder.getSpeed();
   };

   public double getTurnEncoderPosition() {
      return m_turningEncoder.getDistance();
   };

   public void setDesiredDriveSpeedMPS(double speed) {
      driveMotorOutput = m_drivingPIDController.calculate(getDriveEncoderSpeedMPS(), speed);

      // Apply PID output
      driveMotor.setInputVoltage(MathUtil.clamp(driveMotorOutput,-12, 12));
      driveMotor.getAngularVelocityRPM();

   };

   public void setDesiredTurnAngle(double angle) {
      turnMotorOutput = m_turningPIDController.calculate(getTurnEncoderPosition(), angle);

      // Apply PID output
      turnMotor.setInputVoltage(MathUtil.clamp(turnMotorOutput, -12, 12));
   };

   public double getDriveBusVoltage() {
      return 0;
   };

   public double getDriveOutput() {
      return driveMotorOutput;
   }

   public double getTurnBusVoltage() {
      return 0;
   }

   public double getTurnOutput() {
      return turnMotorOutput;
   }

   public String getName(){
      return name;
   }

   public double getChassisAngularOffset(){
      return 0.0;
   }
}
