// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.esefsubsystem;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFShoulderMechanism {

    TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
    public TalonFX shoulder;
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
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 10, "Shoulder")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.shoulder = new TalonFX(10);
            // this.shoulderEncoder = new CANcoder(10);
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kG = 0; // Gravity FeedForward
            slot0Configs.kS = 0; // Friction FeedForward
            slot0Configs.kP = 1; // an error of 1 rotation results in x Volt output
            slot0Configs.kI = 0;
            slot0Configs.kD = 0;

            shoulder.getConfigurator().apply(slot0Configs); // Applies the Config to the shoulder motor
        }

    }

    /*
     * public enum ShoulderPosition {};
     * ShoulderPosition currentShoulderPosition;
     */

    public void periodic() {
        if (shoulder != null) {
            SmartDashboard.putNumber("frc3620/Shoulder/ActualPosition", shoulder.getPosition().getValueAsDouble());
        }
    }

    public void setShoulderPosition(double position) {
        // set the shoulder to the desired position Cat
        SmartDashboard.putNumber("frc3620/Shoulder/RequestedPosition", position);

        if (shoulder != null) {
            shoulder.setControl(shoulderRequest.withPosition(position));
        }
    }

}


