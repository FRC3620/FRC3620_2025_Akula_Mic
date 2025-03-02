// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import java.time.Period;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFEndEffectorMechanism {

    DigitalInput beambreak = new DigitalInput(8);

    TalonFX endEff;

    boolean algaeIn = false;

    //final DutyCycleOut endEffControl = new DutyCycleOut(0);

    final int ENDEFFECTORMOTORID = 12;

    public ESEFEndEffectorMechanism() {
        // constructor
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, ENDEFFECTORMOTORID,
                "End Effector")
                || RobotContainer.shouldMakeAllCANDevices()) {
            endEff = new TalonFX(ENDEFFECTORMOTORID);

            TalonFXConfiguration endEffConfigs = new TalonFXConfiguration();

            endEffConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
            endEffConfigs.MotorOutput.withPeakForwardDutyCycle(0.8);
            endEffConfigs.MotorOutput.withPeakReverseDutyCycle(-0.8);
            endEffConfigs.Voltage.withPeakForwardVoltage(12 * 0.8);
            endEffConfigs.Voltage.withPeakReverseVoltage(-12 * 0.8);

            // elevatorAConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            // elevatorAConfigs.MotorOutput.PeakForwardDutyCycle = 0.05;
            // elevatorAConfigs.MotorOutput.PeakReverseDutyCycle = 0.025;
            // elevatorAConfigs.Voltage.PeakForwardVoltage = (0.05 * 12);
            // elevatorAConfigs.Voltage.PeakReverseVoltage = (0.025 * 12);

            // elevatorA.setPosition(0);

            endEff.getConfigurator().apply(endEffConfigs);
        }

    }

    public void periodic() {
        SmartDashboard.putBoolean("frc3620/EndEffector/hasCoral", hasCoral());
        SmartDashboard.putNumber("frc3620/EndEffector/MotorVelocity", endEff.getVelocity().getValueAsDouble());
    }

    public void setEndEffSpeed(double speed) {
        if (endEff != null) {
            endEff.set(speed);
        }
    }

    public boolean hasCoral() {
        return !beambreak.get();
    }

    public double getEndEffectorVelocity() {
        return endEff.getVelocity().getValueAsDouble();
    }

}
