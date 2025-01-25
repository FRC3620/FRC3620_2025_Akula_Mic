package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimberPostionCommand extends Command {
    public SetClimberPostionCommand(Integer postion,ClimberSubsystem climberSubsystem){
        climberSubsystem.setPostion(postion);
    }
}
