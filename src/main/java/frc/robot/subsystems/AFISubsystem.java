// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AFISubsystem extends SubsystemBase {
  /** Creates a new AFISubsystem. */
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    public TalonFX pivot;
    final PositionVoltage pivotRequest = new PositionVoltage(0).withSlot(0);


    //SparkMaxConfig rollerConfig = new SparkMaxConfig();
    public TalonFX roller;

    final int AFIPIVOTMOTORID = 14;
    final int AFIROLLERMOTORID = 15;
    
  

  public AFISubsystem() {
    //constructor

  
    //Pivot
     if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, AFIPIVOTMOTORID, "AFIPivot")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.pivot = new TalonFX(AFIPIVOTMOTORID);
            // this.shoulderEncoder = new CANcoder(10);
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kG = 0; // Gravity FeedForward
            slot0Configs.kS = 0; // Friction FeedForward
            slot0Configs.kP = 1; // an error of 1 rotation results in x Volt output
            slot0Configs.kI = 0;
            slot0Configs.kD = 0;

            pivot.getConfigurator().apply(slot0Configs);
          } // Applies the Config to the shoulder motor

      //Roller
      if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, AFIROLLERMOTORID, "AFIRoller")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.roller = new TalonFX(AFIROLLERMOTORID);
          } 
          
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     if (pivot != null) {
            SmartDashboard.putNumber("frc3620/AFI/PivotActualPosition", pivot.getPosition().getValueAsDouble());
        }
    }

    public void setPivotPosition(double position) {
        // set the shoulder to the desired position Cat
        SmartDashboard.putNumber("frc3620/AFI/PivotRequestedPosition", position);

        if (pivot != null) {
            pivot.setControl(pivotRequest.withPosition(position));
        }
    }

  public void setAFIRollerSpeed(double speed){
    roller.set(speed);
  }
}
