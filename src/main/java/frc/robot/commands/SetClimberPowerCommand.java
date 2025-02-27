package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimberPowerCommand extends Command {
    ClimberSubsystem climberSubsystem;
    Double power;
    public SetClimberPowerCommand(Double power,ClimberSubsystem climberSubsystem){
        this.climberSubsystem=climberSubsystem;
        this.power=power;
    }
    
    
    @Override
    public void initialize() {
        climberSubsystem.setClimberPower(power);
        
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
