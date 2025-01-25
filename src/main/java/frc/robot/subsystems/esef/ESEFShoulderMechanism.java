// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.esef;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.SlotConfigs;
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
public class ESEFShoulderMechanism {

    TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
    public TalonFX shoulder;
    public CANcoder shoulderEncoder;
    //public final VelocityVoltage -- Do I need this?

    public static double shoulderL4;
    public static double shoulderL3;
    public static double shoulderL2;
    public static double shoulderL1;
    public static double shoulderFunnel;

    final PositionVoltage shoulderRequest = new PositionVoltage(0).withSlot(10);

    public ESEFShoulderMechanism(){ //Constructor
        this.shoulder = new TalonFX(15);
        this.shoulderEncoder = new CANcoder(15);
        SlotConfigs slot15Configs = new SlotConfigs();
        slot15Configs.kG = 0; //Gravity FeedForward
        slot15Configs.kS = 0; //Friction FeedForward
        slot15Configs.kP = 1; //an error of 1 rotation results in x Volt output
        slot15Configs.kI = 0;
        slot15Configs.kD = 0;

        shoulder.getConfigurator().apply(slot15Configs);

    }

    /*public enum ShoulderPosition {};
    ShoulderPosition currentShoulderPosition;*/

    public void periodic(){

    }

    public void setShoulderPosition(Double position) {
        shoulder.setControl(shoulderRequest.withPosition(position));
        // add code to set the shoulder to the desired position
    }

}
