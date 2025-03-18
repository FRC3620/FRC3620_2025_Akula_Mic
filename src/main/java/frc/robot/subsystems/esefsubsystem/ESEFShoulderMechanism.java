// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFShoulderMechanism {

    private final MotionMagicVoltage shoulderMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    public static final double kShoulderDefaultSetpointDegrees = 75.0;

    CANcoder shoulderEncoder;

    Timer calibrationTimer;

    public TalonFX shoulder;
    final int SHOULDER_MOTOR_ID = 11;
    final int SHOULDER_ENCODER_ID = 11;

    double shoulderCalibratedPosition = 95; //degrees

    // to save a requested position if encoder is not calibrated
    Double requestedPositionWhileCalibrating = null;

    final PositionVoltage shoulderRequest = new PositionVoltage(0).withSlot(0);

    public ESEFShoulderMechanism() { // Constructor sdyvgbewkhb
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, SHOULDER_MOTOR_ID, "Shoulder")
                || RobotContainer.shouldMakeAllCANDevices()) {
   
            RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, SHOULDER_ENCODER_ID, "ShoulderEncoder");
            this.shoulder = new TalonFX(SHOULDER_MOTOR_ID);
            this.shoulderEncoder = new CANcoder(SHOULDER_ENCODER_ID);
            // this.shoulderEncoder = new CANcoder(10);
            TalonFXConfiguration shoulderConfigs = new TalonFXConfiguration();

            shoulderConfigs.Slot0.kG = 0.04; // Gravity FeedForward
            shoulderConfigs.Slot0.kS = 0; // Friction FeedForward
            shoulderConfigs.Slot0.kP = 50; // an error of 1 rotation results in x Volt output
            shoulderConfigs.Slot0.kI = 0;
            shoulderConfigs.Slot0.kD = 0;

            shoulderConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

            shoulderConfigs.MotionMagic.MotionMagicCruiseVelocity = 75; // Max speed in Rotations per second
            shoulderConfigs.MotionMagic.MotionMagicAcceleration = 60; // Max acceleration in Rotations per second^2
            shoulderConfigs.MotionMagic.MotionMagicJerk = 200; // Smooth acceleration (optional)


            shoulderConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
            shoulderConfigs.MotorOutput.withPeakForwardDutyCycle(0.6);
            shoulderConfigs.MotorOutput.withPeakReverseDutyCycle(-0.5);
            shoulderConfigs.Voltage.withPeakForwardVoltage(12 * 0.6);
            shoulderConfigs.Voltage.withPeakReverseVoltage(12 * -0.5);
            
            
            // This CANcoder should report absolute position from [-0.5, 0.5) rotations,
            // with a 0.26 rotation offset, with clockwise being positive
            
            CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
            
            canCoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5).withMagnetOffset(0.19).withSensorDirection(SensorDirectionValue.Clockwise_Positive);
            
            shoulderConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            shoulderConfigs.Feedback.FeedbackRemoteSensorID = shoulderEncoder.getDeviceID();
            

            shoulderEncoder.getConfigurator().apply(canCoderConfigs);
            shoulder.getConfigurator().apply(shoulderConfigs); // Applies the Config to the shoulder motor

            shoulder.setPosition(Degrees.of(88));
        }

    }

    public void periodic() {
        if (shoulder != null) {
            SmartDashboard.putNumber("frc3620/Shoulder/MotorAppliedOutput", shoulder.get());
            SmartDashboard.putNumber("frc3620/Shoulder/ActualPosition", getCurrentAngle().in(Degrees));
        }
    }

    public void setSetpoint(Angle position) {
        // set the shoulder to the desired position Cat
        SmartDashboard.putNumber("frc3620/Shoulder/RequestedPosition", position.in(Degrees));

        if (shoulder != null) {
            shoulder.setControl(shoulderMotionMagicRequest.withPosition(position.in(Rotations)));
        }
    }

    public Angle getCurrentAngle(){
        if (shoulderEncoder != null) {
            return shoulderEncoder.getAbsolutePosition().getValue();
        } else {
            return Degrees.of(0);
        }
    }

}


