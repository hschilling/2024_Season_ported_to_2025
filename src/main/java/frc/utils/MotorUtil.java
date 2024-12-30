// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.utils;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// public class MotorUtil {

//     // public static SparkMax createSparkMAX(int id, MotorType motortype, int stallLimit, boolean isInverted,
//     //         boolean isIdleBreak, double slewRate) {
//     //     SparkMax sparkMAX = createSparkMAX(id, motortype, stallLimit, isIdleBreak, slewRate);
//     //     sparkMAX.setInverted(isInverted);
//     //     return sparkMAX;
//     // }

//     public static SparkMax createSparkMAX(int id, MotorType motortype, int stallLimit, boolean isInverted, 
//             boolean isIdleBreak, double slewRate) {
//         SparkMax sparkMAX = new SparkMax(id, motortype);

//         SparkMaxConfig config = new SparkMaxConfig();

//         config.signals.primaryEncoderPositionPeriodMs(5); 

//         if (isIdleBreak) {
//             config.idleMode(IdleMode.kBrake);
//         } else {
//             config.idleMode(IdleMode.kCoast);
//         }

//         config.inverted(isInverted);
//         config.encoder
//                 .positionConversionFactor(1000)
//                 .velocityConversionFactor(1000);
//         config.closedLoop
//                 .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//                 .pid(1.0, 0.0, 0.0);


//         sparkMAX.restoreFactoryDefaults();
//         sparkMAX.setSmartCurrentLimit(stallLimit);


//         sparkMAX.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//         // built in slew rate for spark max
//         sparkMAX.setOpenLoopRampRate(slewRate);

//         return sparkMAX;
//     }

//     public static void turnOffLevel3456StatusFrames() {

//     }

// }
