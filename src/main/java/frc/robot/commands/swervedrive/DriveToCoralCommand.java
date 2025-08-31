// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Camera;
import frc.robot.subsystems.VisionSubsystem.CameraType;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToCoralCommand extends Command {

  private final SwerveSubsystem swerve;
  VisionSubsystem visionSubsystem;
  Camera camera;

  double desiredTX = 0.0;
  double desiredTY = -25.25;
  double desiredTa = 3.65;

  double tx;
  double ty;
  double ta;

  //double driveX;
  double driveX;
  double driveY;
  //double SpinA;

  ProfiledPIDController txController;
  ProfiledPIDController tyController;
  ProfiledPIDController taController;

  boolean seenTarget = true;
  

  /** Creates a new DriveToCoral. */
  public DriveToCoralCommand(Camera camera, SwerveSubsystem swerve) {
    this.visionSubsystem = RobotContainer.visionSubsystem;
    this.camera = camera;
    this.swerve = RobotContainer.swerveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    seenTarget = true;
    visionSubsystem.setCameraType(camera, CameraType.ObjectDetection);

    tx = visionSubsystem.getTx(camera);
    ty = visionSubsystem.getTy(camera);
    ta = visionSubsystem.getTa(camera);

    txController = new ProfiledPIDController(0.06, 0, 0, new TrapezoidProfile.Constraints(0.3, 1));
    tyController = new ProfiledPIDController(0.15, 0, 0, new TrapezoidProfile.Constraints(0.35, 0.1));
    //taController = new ProfiledPIDController(1.0, 0, 0, new TrapezoidProfile.Constraints(0.2, 0.1));

    txController.setTolerance(0.05);
    tyController.setTolerance(0.05);

    //txController.setGoal(desiredTX);
    //tyController.setGoal(desiredTY);
    //taController.setGoal(desiredTA);

    if (visionSubsystem.countObjects(camera) < 1) {
      seenTarget = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionSubsystem.countObjects(camera) < 1) {
      seenTarget = false;
    } else {
      seenTarget = true;
    }

    tx = visionSubsystem.getTx(camera);
    ty = visionSubsystem.getTy(camera);
    ta = visionSubsystem.getTa(camera);

    //driveX = txController.calculate(tx, desiredTX);
    driveX = txController.calculate(tx, desiredTX);
    driveY = tyController.calculate(ty, desiredTY);
    //SpinA = taController.calculate(ta, desiredTa);

    //swerve.driveCommand(() -> driveX, () -> driveY, () -> SpinA);

    if (visionSubsystem.getCameraType(camera) == CameraType.ObjectDetection && seenTarget) {
      swerve.drive(new ChassisSpeeds(
        1/Math.sqrt(driveY), 0, driveX
      ));
    }
    
    SmartDashboard.putNumber("Tx", tx);
    SmartDashboard.putNumber("Ty", ty);
    SmartDashboard.putNumber("Ta", ta);
    
    SmartDashboard.putNumber("TxVelocity", txController.getSetpoint().velocity);
    SmartDashboard.putNumber("TyVelocity", tyController.getSetpoint().velocity);
    //SmartDashboard.putNumber("TaVelocity", taController.getSetpoint().velocity);

    SmartDashboard.putNumber("TxError", desiredTX-tx);
    SmartDashboard.putNumber("TyError", desiredTY-ty);

    SmartDashboard.putBoolean("seenTarget", seenTarget);
    SmartDashboard.putNumber("NumberTargetsSeen", visionSubsystem.countObjects(camera));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSubsystem.setCameraType(camera, CameraType.AprilTagDetection);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (visionSubsystem.isCloseEnough(camera) && visionSubsystem.isCentered(camera));
  }
}
