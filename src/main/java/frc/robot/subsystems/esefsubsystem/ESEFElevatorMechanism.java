// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFElevatorMechanism {

    TalonFXConfiguration elevatorAConfig = new TalonFXConfiguration();
    TalonFXConfiguration elevatorBConfig = new TalonFXConfiguration();
    public TalonFX elevatorA;
    public TalonFX elevatorB;
    final int ELEVATORA_MOTORID = 9; 
    final int ELEVATORB_MOTORID = 10;

    final PositionVoltage elevatorARequest = new PositionVoltage(0).withSlot(0); //check if update frequency to follower needs to be updated
    //update frequency will also affect PID

    // Add CANCoders

    public ESEFElevatorMechanism() { // constructor

        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, ELEVATORA_MOTORID, "Elevator Motor A")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.elevatorA = new TalonFX(ELEVATORA_MOTORID);
            this.elevatorB = new TalonFX(ELEVATORB_MOTORID);
            // this.shoulderEncoder = new CANcoder(10);
            Slot0Configs elevatorAConfigs = new Slot0Configs();
            elevatorAConfigs.kG = 0; // Gravity FeedForward
            elevatorAConfigs.kS = 0; // Friction FeedForward
            elevatorAConfigs.kP = 1; // an error of 1 rotation results in x Volt output
            elevatorAConfigs.kI = 0;
            elevatorAConfigs.kD = 0;

            elevatorA.getConfigurator().apply(elevatorAConfigs); // Applies the Config to the motor
            elevatorB.setControl(new Follower(ELEVATORA_MOTORID, false));
        }

    }

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
    }

}
