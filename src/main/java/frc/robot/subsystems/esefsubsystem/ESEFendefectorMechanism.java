package frc.robot.subsystems.esefsubsystem;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.RobotContainer;

public class ESEFendefectorMechanism {

    TalonFX motor;
    Slot0Configs configs = new Slot0Configs();

    ESEFendefectorMechanism() {
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 9)) {
            motor = new TalonFX(9);
        }
    }

    public void setSpeed(Double speed) {
        motor.set(speed);
    }
}
