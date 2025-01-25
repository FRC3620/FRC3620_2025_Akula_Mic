package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    double pos1=0;
    double pos2=100;
    TalonFX motor;



    public ClimberSubsystem(){
        motor=new TalonFX(15);
        
    }

    public void setPostion(Integer cpos){

    }

}
