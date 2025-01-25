package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    double pos1 = 0;
    double pos2 = 100;
    TalonFX motor;
    // in init function, set slot 0 gains
    Slot0Configs slot0Configs= new Slot0Configs();

    public ClimberSubsystem() {
        motor = new TalonFX(15);
        motor = new TalonFX(15);
        // slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        motor.getConfigurator().apply(slot0Configs);
    }

    public void setPostion(Integer cpos) {
        if (cpos == 1) {
            motor.setPosition(pos1);
        } else {
            motor.setPosition(pos2);
        }
    }

}
