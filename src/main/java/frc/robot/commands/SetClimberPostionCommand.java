package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimberPostionCommand extends Command {
    ClimberSubsystem climberSubsystem;
    Double postion;
    public SetClimberPostionCommand(Double postion,ClimberSubsystem climberSubsystem){
        this.climberSubsystem=climberSubsystem;
        this.postion=postion;
    }
    
    
    @Override
    public void initialize() {
        climberSubsystem.setPostion(postion);
        
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
