package frc.robot.commandfactories;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.swervedrive.AutoAlignToAlgaeCommand;
import frc.robot.commands.swervedrive.DriveToClosestStickCommand;
import frc.robot.commands.swervedrive.SwerveDriveDiagnosticCommand;
import frc.robot.commands.swervedrive.DriveToClosestStickCommand.WhichStick;
import frc.robot.commands.swervedrive.DriveToPoseCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SwerveSubsystemCommandFactory {
  SwerveSubsystem swerveSubsystem;

  public SwerveSubsystemCommandFactory(SwerveSubsystem afiSubsystem) {
    this.swerveSubsystem = afiSubsystem;
  }

  public void setupSmartDashboardCommands() {
    // SmartDashboard.putData("Drive 10 feet",
    // swerveSubsystem.driveToDistanceCommand(Units.feetToMeters(10), 0.5));
    SmartDashboard.putData("Test Drive To Pose", swerveSubsystem.pathFinderCommand());

    SmartDashboard.putData("Drive 10 feet", swerveSubsystem.driveToDistanceCommand(Units.feetToMeters(10), 0.5));
    SmartDashboard.putData("TurnToAngle", new TurnToAngleCommand(Degrees.of(90), Meters.of(2), swerveSubsystem));
    SmartDashboard.putData("TurnToAngle-135", new TurnToAngleCommand(Degrees.of(-135), Meters.of(2), swerveSubsystem));

    SmartDashboard.putData("Drive To Nearest Tag LEFT", new DriveToClosestStickCommand(WhichStick.LEFT));
    SmartDashboard.putData("Drive To Nearest Tag RIGHT", new DriveToClosestStickCommand(WhichStick.RIGHT));

    Pose2d testPose = new Pose2d(5.1, 3.09, Rotation2d.fromDegrees(120));
    SmartDashboard.putData("Drive To TEST", swerveSubsystem.driveToPoseSlow(testPose));

    SmartDashboard.putData("Drive to Poses",
        new SequentialCommandGroup(
            new DriveToPoseCommand(swerveSubsystem, new Pose2d(2.3, 4.07, Rotation2d.fromDegrees(0))).withTimeout(2.5),
            new DriveToPoseCommand(swerveSubsystem, new Pose2d(3.3, 4.07, Rotation2d.fromDegrees(0)))));

    SmartDashboard.putData("Algae Auto Align", new AutoAlignToAlgaeCommand());

    SmartDashboard.putData(new SwerveDriveDiagnosticCommand(swerveSubsystem));

  }

}
