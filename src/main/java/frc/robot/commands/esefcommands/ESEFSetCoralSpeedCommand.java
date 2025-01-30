package frc.robot.commands.esefcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.esefsubsystem.ESEFEndEffectorMechanism;

public class ESEFSetCoralSpeedCommand extends Command {
    double speed;
    ESEFEndEffectorMechanism eseFendefectorMechanism;
ESEFSetCoralSpeedCommand(double speed, ESEFEndEffectorMechanism eseFendefectorMechanism){
this.eseFendefectorMechanism=eseFendefectorMechanism;
this.speed=speed;
}

@Override
public void initialize() {
   // eseFendefectorMechanism.setSpeed(speed);
}
@Override
public boolean isFinished() {
    return true;
}
}
