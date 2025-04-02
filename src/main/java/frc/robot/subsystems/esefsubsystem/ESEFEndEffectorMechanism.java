// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFEndEffectorMechanism {

    DigitalInput beambreak = new DigitalInput(8);

    TalonFX endEff;

    boolean algaeIn = false;

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

            endEff.getConfigurator().apply(endEffConfigs);
            endEff.setNeutralMode(NeutralModeValue.Brake);
        }

    }

    public void periodic() {
        SmartDashboard.putBoolean("frc3620/EndEffector/HasCoral", hasCoral());
        if (endEff != null) {
            SmartDashboard.putNumber("frc3620/EndEffector/MotorVelocity", endEff.getVelocity().getValue().in(RotationsPerSecond));
            SmartDashboard.putNumber("frc3620/EndEffector/ActualPower", endEff.get());
        }
    }

    public void setEndEffSpeed(double speed) {
        SmartDashboard.putNumber("frc3620/EndEffector/RequestedPower", speed);
        if (endEff != null) {
            endEff.set(speed);
        }
    }

    public boolean hasCoral() {
        return !beambreak.get();
    }

    public double getEndEffectorVelocity() {
        return endEff.getVelocity().getValue().in(RotationsPerSecond);
    }

}
