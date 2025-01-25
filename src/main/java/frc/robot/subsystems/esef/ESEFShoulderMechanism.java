// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esef;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFShoulderMechanism {

    TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
    public TalonFX shoulder;
    AbsoluteEncoder shoulderEncoder;
    //public final VelocityVoltage -- Do I need this?

    public static double shoulderL4;
    public static double shoulderL3;
    public static double shoulderL2;
    public static double shoulderL1;
    public static double shoulderFunnel;

    public enum ShoulderPosition {};
    ShoulderPosition currentShoulderPosition;

    public void setShoulderPosition(ShoulderPosition position) {
        currentShoulderPosition = position;
        // add code to set the shoulder to the desired position
    }

}
