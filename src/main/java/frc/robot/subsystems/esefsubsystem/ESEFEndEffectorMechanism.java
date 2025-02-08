// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFEndEffectorMechanism {

    TalonFX endEff;
    //SparkMaxConfig clawConfig = new SparkMaxConfig();

    final DutyCycleOut endEffControl = new DutyCycleOut(0);

    final int ENDEFFECTORMOTORID = 12;

    public ESEFEndEffectorMechanism() {
        // constructor
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, ENDEFFECTORMOTORID, "End Effector")
                || RobotContainer.shouldMakeAllCANDevices()) {
            endEff = new TalonFX(ENDEFFECTORMOTORID);
        }

    }

    public void setEndEffSpeed(double speed) {
        if (endEff != null) {
            endEff.set(speed);
        }
    }

}
