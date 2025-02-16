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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFEndEffectorMechanism {

    DigitalInput beambreak = new DigitalInput(8);

    TalonFX endEff;
    TalonFXConfiguration clawConfig = new TalonFXConfiguration();

    final DutyCycleOut endEffControl = new DutyCycleOut(0);

    final int ENDEFFECTORMOTORID = 12;

    public ESEFEndEffectorMechanism() {
        // constructor
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, ENDEFFECTORMOTORID, "End Effector")
                || RobotContainer.shouldMakeAllCANDevices()) {
            endEff = new TalonFX(ENDEFFECTORMOTORID);
        }

    }

    public void periodic() 
    {
        SmartDashboard.putBoolean("frc3620/EndEffector/hasCoral", hasCoral());
    }

    public void setEndEffSpeed(double speed) {
        if (endEff != null) {
            endEff.set(speed);
        }
    }

    public boolean hasCoral()
    {
        return !beambreak.get();
    }   

}
