// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.RobotMode;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.EncoderConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFShoulderMechanism {

    CANcoder shoulderEncoder;

    Timer calibrationTimer;

    public TalonFX shoulder;
    final int SHOULDER_MOTOR_ID = 11;
    final int SHOULDER_ENCODER_ID = 11;

    double shoulderCalibratedPosition = 95; //degrees

    // to save a requested position if encoder is not calibrated
    Double requestedPositionWhileCalibrating = null;

    //public CANcoder shoulderEncoder;
    // public final VelocityVoltage -- Do I need this?

    /*
     * public static double shoulderL4;
     * public static double shoulderL3;
     * public static double shoulderL2;
     * public static double shoulderL1;
     * public static double shoulderFunnel;
     */

    final PositionVoltage shoulderRequest = new PositionVoltage(0).withSlot(0);

    public ESEFShoulderMechanism() { // Constructor sdyvgbewkhb
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, SHOULDER_MOTOR_ID, "Shoulder")
                || RobotContainer.shouldMakeAllCANDevices()) {
   
            RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, SHOULDER_ENCODER_ID, "ShoulderEncoder");
            this.shoulder = new TalonFX(SHOULDER_MOTOR_ID);
            this.shoulderEncoder = new CANcoder(SHOULDER_ENCODER_ID);
            // this.shoulderEncoder = new CANcoder(10);
            TalonFXConfiguration shoulderConfigs = new TalonFXConfiguration();

            shoulderConfigs.Slot0.kG = 0.02; // Gravity FeedForward
            shoulderConfigs.Slot0.kS = 0; // Friction FeedForward
            shoulderConfigs.Slot0.kP = 25; // an error of 1 rotation results in x Volt output
            shoulderConfigs.Slot0.kI = 0;
            shoulderConfigs.Slot0.kD = 0;

            shoulderConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

            shoulderConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
            shoulderConfigs.MotorOutput.withPeakForwardDutyCycle(0.3);
            shoulderConfigs.MotorOutput.withPeakReverseDutyCycle(-0.1);
            shoulderConfigs.Voltage.withPeakForwardVoltage(12 * 0.3);
            shoulderConfigs.Voltage.withPeakReverseVoltage(12 * -0.1);
            
            
            // This CANcoder should report absolute position from [-0.5, 0.5) rotations,
            // with a 0.26 rotation offset, with clockwise being positive
            
            CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
            
            canCoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5).withMagnetOffset(0.19).withSensorDirection(SensorDirectionValue.Clockwise_Positive);
            
            shoulderConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            shoulderConfigs.Feedback.FeedbackRemoteSensorID = shoulderEncoder.getDeviceID();
            

            shoulderEncoder.getConfigurator().apply(canCoderConfigs);
            shoulder.getConfigurator().apply(shoulderConfigs); // Applies the Config to the shoulder motor

            shoulder.setPosition(Degrees.of(90));
        }

    }

    /*
     * public enum ShoulderPosition {};
     * ShoulderPosition currentShoulderPosition;
     */



    public void periodic() {
    
        
         // only do something if we actually have a motor
   /*  if (shoulder != null) {
        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) {
            // If the robot is running, and the encoder is "not calibrated," run motor very
            // slowly towards the switch
            shoulder.set(0.03); 
            if (calibrationTimer == null) {
              // we need to calibrate and we have no timer. make one and start it
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              // we have a timer, has the motor had power long enough to spin up
              if (calibrationTimer.get() > 0.2) {
                if (Math.abs(shoulder.getVelocity().getValueAsDouble()) < 0.4) {
                  // motor is not moving, hopefully it's against the stop
                  encoderCalibrated = true;
                  shoulder.set(0);
                  shoulder.setPosition(shoulderCalibratedPosition / positionConversion);
                  
                  setShoulderPositionDegrees(shoulderCalibratedPosition);



                  // If there was a requested position while we were calibrating, go there
                  if (requestedPositionWhileCalibrating != null) {
                    setShoulderPositionDegrees(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  }

                }
              }
            }
          }
        }
    }*/
        if (shoulder != null) {
            SmartDashboard.putNumber("frc3620/Shoulder/MotorAppliedOutput", shoulder.get());
            SmartDashboard.putNumber("frc3620/Shoulder/AbsolutePosition", shoulderEncoder.getAbsolutePosition().getValue().in(Rotations));

            SmartDashboard.putNumber("frc3620/Shoulder/ActualPositionDegrees", getShoulderPosition().in(Degrees));
        }
    }

    public void setShoulderPosition(Angle position) {
        // set the shoulder to the desired position Cat
        SmartDashboard.putNumber("frc3620/Shoulder/RequestedPosition", position.in(Degrees));

        if (shoulder != null) {
            shoulder.setControl(shoulderRequest.withPosition(position));
        }
    }

    public Angle getShoulderPosition(){

        return shoulderEncoder.getAbsolutePosition().getValue();

    }

}


