// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFEndEffectorMechanism {

    TalonFX endEff;
    TalonFXConfiguration clawConfig = new TalonFXConfiguration();

    final DutyCycleOut endEffControl = new DutyCycleOut(0);

    public ESEFEndEffectorMechanism(){ 
        //constructor
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 9, "End Effector")
                || RobotContainer.shouldMakeAllCANDevices()) {
        endEff = new TalonFX(9);
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kG = 0; // Gravity FeedForward
        slot0Configs.kS = 0; // Friction FeedForward
        slot0Configs.kP = 1; // an error of 1 rotation results in x Volt output
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        endEff.getConfigurator().apply(slot0Configs);

        }
                
    }

    public void setEndEffSpeed(double speed){
        endEff.setControl(endEffControl.withOutput(speed));
    }


}
