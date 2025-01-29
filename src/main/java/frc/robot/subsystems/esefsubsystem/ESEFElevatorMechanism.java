// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.NTPublisher;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFElevatorMechanism {

    TalonFXConfiguration elevatorAConfig = new TalonFXConfiguration();
    TalonFXConfiguration elevatorBConfig = new TalonFXConfiguration();
    public TalonFX elevatorA;
    public TalonFX elevatorB;
    // Add CANCoders

    public ESEFElevatorMechanism() { // constructor

        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 11, "Elevator Motor A")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.elevatorA = new TalonFX(11);
            // this.shoulderEncoder = new CANcoder(10);
            Slot0Configs elevatorAConfigs = new Slot0Configs();
            elevatorAConfigs.kG = 0; // Gravity FeedForward
            elevatorAConfigs.kS = 0; // Friction FeedForward
            elevatorAConfigs.kP = 1; // an error of 1 rotation results in x Volt output
            elevatorAConfigs.kI = 0;
            elevatorAConfigs.kD = 0;

            elevatorA.getConfigurator().apply(elevatorAConfigs); // Applies the Config to the motor
        }
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 12, "Elevator Motor B")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.elevatorB = new TalonFX(12);
            // this.shoulderEncoder = new CANcoder(10);
            Slot0Configs elevatorBConfigs = new Slot0Configs();
            elevatorBConfigs.kG = 0; // Gravity FeedForward
            elevatorBConfigs.kS = 0; // Friction FeedForward
            elevatorBConfigs.kP = 1; // an error of 1 rotation results in x Volt output
            elevatorBConfigs.kI = 0;
            elevatorBConfigs.kD = 0;

            elevatorB.getConfigurator().apply(elevatorBConfigs); // Applies the Config to the motor
        }

    }

    final PositionVoltage elevatorARequest = new PositionVoltage(0).withSlot(0);
    final PositionVoltage elevatorBRequest = new PositionVoltage(0).withSlot(0);

    /*
     * public enum ShoulderPosition {};
     * ShoulderPosition currentShoulderPosition;
     */

    public void periodic() {
        if (elevatorA != null) {
            SmartDashboard.putNumber("frc3620/Elevator/AMotorActualPosition",
                    elevatorA.getPosition().getValueAsDouble());
        }
        if (elevatorB != null) {
            SmartDashboard.putNumber("frc3620/Elevator/BMotorActualPosition",
                    elevatorB.getPosition().getValueAsDouble());
        }
    }

    public void setElevatorPosition(double position) {
        // set the shoulder to the desired position Cat
        SmartDashboard.putNumber("frc3620/Elevator/RequestedPosition", position);

        if (elevatorA != null) {
            elevatorA.setControl(elevatorARequest.withPosition(position));
        }
        if (elevatorB != null) {
            elevatorB.setControl(elevatorBRequest.withPosition(position));
        }
    }

}
