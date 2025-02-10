// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Revolutions;
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

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFShoulderMechanism {

    public TalonFX shoulder;
    final int SHOULDER_MOTOR_ID = 11;

    CANcoder shoulderEncoder;
    final int SHOULDER_ENCODER_ID = 11;
    
    final PositionVoltage shoulderRequest = new PositionVoltage(0).withSlot(0);

    Angle currentShoulderPosition = Rotations.of(0);

    public ESEFShoulderMechanism() { // Constructor sdyvgbewkhb
        TalonFXConfiguration shoulderConfig = null;
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, SHOULDER_MOTOR_ID, "Shoulder")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.shoulder = new TalonFX(SHOULDER_MOTOR_ID);
            // this.shoulderEncoder = new CANcoder(10);
            shoulderConfig = new TalonFXConfiguration();

            shoulderConfig.Slot0.kG = 0; // Gravity FeedForward
            shoulderConfig.Slot0.kS = 0; // Friction FeedForward
            shoulderConfig.Slot0.kP = 1000; // an error of 1 rotation results in x Volt output
            shoulderConfig.Slot0.kI = 0;
            shoulderConfig.Slot0.kD = 0;

            shoulderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

            shoulderConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
            shoulderConfig.MotorOutput.withPeakForwardDutyCycle(0.1);
            shoulderConfig.MotorOutput.withPeakReverseDutyCycle(-0.05);
            shoulderConfig.Voltage.withPeakForwardVoltage(12 * 0.1);
            shoulderConfig.Voltage.withPeakReverseVoltage(-12 * 0.05);
            // apply the configs below (after the feedback is set up)
        }

        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, SHOULDER_ENCODER_ID, "Shoulder Encoder")
                || RobotContainer.shouldMakeAllCANDevices()) {
            shoulderEncoder = new CANcoder(SHOULDER_ENCODER_ID);

            CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
            canCoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5);
            canCoderConfigs.MagnetSensor.withMagnetOffset(0.19).withSensorDirection(SensorDirectionValue.Clockwise_Positive);
            shoulderEncoder.getConfigurator().apply(canCoderConfigs);
        }

        if (shoulderConfig != null && shoulderEncoder != null) {
            shoulderConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
            shoulderConfig.Feedback.withFeedbackRemoteSensorID(SHOULDER_ENCODER_ID);
        }

        if (shoulderConfig != null) {
            shoulder.getConfigurator().apply(shoulderConfig); // Applies the Config to the shoulder motor
        }

    }

    public void periodic() {
        if (shoulderEncoder != null) {
            SmartDashboard.putString("frc3620/ESEF/Shoulder/PositionClass", shoulderEncoder.getAbsolutePosition().getValue().getClass().getCanonicalName());
            SmartDashboard.putNumber("frc3620/ESEF/Shoulder/AbsolutePosition", shoulderEncoder.getAbsolutePosition().getValueAsDouble() * 360);
            SmartDashboard.putNumber("frc3620/ESEF/Shoulder/AbsolutePosition2", shoulderEncoder.getAbsolutePosition().getValue().in(Degrees));
        }

        if (shoulder != null) {
            SmartDashboard.putNumber("frc3620/ESEF/Shoulder/MotorPosition", shoulder.getRotorPosition().getValueAsDouble() * 360);
            SmartDashboard.putNumber("frc3620/ESEF/Shoulder/MotorAppliedOutput", shoulder.get());
            SmartDashboard.putNumber("frc3620/ESEF/Shoulder/MotorAppliedVoltage", shoulder.getMotorVoltage().getValueAsDouble());
        }
    }

    public void setShoulderPositionDegrees(Angle position) {
        if (shoulder != null && shoulderEncoder != null) {
            shoulder.setControl(shoulderRequest.withPosition(position.in(Revolutions)));
        }
        // set the shoulder to the desired position Cat
        SmartDashboard.putNumber("frc3620/ESEF/Shoulder/RequestedPosition", position.in(Degrees));
    }

    /**
     * Return the angle that the shoulder position is at. 
     * 
     * DO NOT HANG ON TO THE ANGLE OBJECT THAT YOU GET FROM THIS. Extract the number you need in
     * degrees, revolutions, whatever, and hang on to that.
     * 
     * @return current shoulder position
     */
    public Angle getShoulderPosition(){
        if (shoulderEncoder != null) {
            currentShoulderPosition = shoulderEncoder.getAbsolutePosition().getValue();
        }
        return currentShoulderPosition;
    }

}