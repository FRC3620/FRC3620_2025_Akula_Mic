package frc.robot.commands.esefcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.esefsubsystem.ESEFendefectorMechanism;

public class ESEFSetCoralSpeedCommand extends Command {
    double speed;
    ESEFendefectorMechanism eseFendefectorMechanism;
ESEFSetCoralSpeedCommand(double speed, ESEFendefectorMechanism eseFendefectorMechanism){
this.eseFendefectorMechanism=eseFendefectorMechanism;
this.speed=speed;
}

@Override
public void initialize() {
    eseFendefectorMechanism.setSpeed(speed);
}
@Override
public boolean isFinished() {
    return true;
}
}
