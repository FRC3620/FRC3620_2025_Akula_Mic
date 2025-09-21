// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Camera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTagCommand extends Command {

    private final SwerveSubsystem swerve;
    VisionSubsystem visionSubsystem;

    double desiredTX = 0.0;
    double desiredTY = 0.0;
    double desiredTa = 0.0;

    double tx;
    double ty;
    double ta;

    // double driveX;
    double driveX;
    double driveY;
    // double SpinA;

    ProfiledPIDController txController;
    ProfiledPIDController tyController;
    ProfiledPIDController taController;

    boolean seenTarget = true;

    public enum WhichSideOfTag {
        Left, Right
    }

    WhichSideOfTag whichSideOfTag;

    Timer timeouter = new Timer();

    /** Creates a new DriveToCoral. */
    public DriveToAprilTagCommand(SwerveSubsystem _swerve, WhichSideOfTag _whichStick) {
        this.visionSubsystem = RobotContainer.visionSubsystem;
        //this.swerve = _swerve;
        swerve = RobotContainer.swerveSubsystem;
        this.whichSideOfTag = _whichStick;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timeouter.reset();
        timeouter.stop();

        seenTarget = true;

        tx = visionSubsystem.getTx();
        ty = visionSubsystem.getTy();
        ta = visionSubsystem.getTa();
        

        txController = new ProfiledPIDController(0.04, 0, 0, new TrapezoidProfile.Constraints(1.2, 1)); // originally
                                                                                                          // 0.06, 0, 0,
        tyController = new ProfiledPIDController(0.24, 0, 0, new TrapezoidProfile.Constraints(0.7, 1)); // originally
                                                                                                           // 0.15, 0, 0
        // taController = new ProfiledPIDController(1.0, 0, 0, new
        // TrapezoidProfile.Constraints(0.2, 0.1));

        txController.setTolerance(0.2);
        tyController.setTolerance(0.2);

        // txController.setGoal(desiredTX);
        // tyController.setGoal(desiredTY);
        // taController.setGoal(desiredTA);

        if (visionSubsystem.countObjects() < 1) {
            seenTarget = false;
        }

        if (whichSideOfTag == WhichSideOfTag.Left) {
            desiredTX = 34.25;
            desiredTY = -14.25;
        } else {
            desiredTX = 4.55;
            desiredTY = -14.25;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (visionSubsystem.countObjects() < 1) {
            seenTarget = false;
        } else {
            seenTarget = true;
        }
        if (seenTarget == true) {
            tx = visionSubsystem.getTx();
            ty = visionSubsystem.getTy();
            ta = visionSubsystem.getTa();

            // driveX = txController.calculate(tx, desiredTX);
            driveX = txController.calculate(tx, desiredTX);
            driveY = tyController.calculate(ty, desiredTY);
            // SpinA = taController.calculate(ta, desiredTa);

            if (Math.abs(tx-desiredTX) < 1) {
                timeouter.start();
                swerve.drive(new ChassisSpeeds(-0.4, 0, 0));

            } else {
                // swerve.driveCommand(() -> driveX, () -> driveY, () -> SpinA);
                
                swerve.drive(new ChassisSpeeds(
                        -driveY, driveX, 0).times(1));

                //swerve.drive(new Translation2d(1/Math.sqrt(driveY), driveX), 0, false);
                // 500, 0, 0));
            }
            // Drive in straight line once in range

            SmartDashboard.putNumber("Tx", tx);
            SmartDashboard.putNumber("Ty", ty);
            SmartDashboard.putNumber("Ta", ta);

            SmartDashboard.putNumber("TxVelocity", txController.getSetpoint().velocity);
            SmartDashboard.putNumber("TyVelocity", tyController.getSetpoint().velocity);
            SmartDashboard.putNumber("Drivey", driveY);
            SmartDashboard.putNumber("Drivex", driveX);
            // SmartDashboard.putNumber("TaVelocity", taController.getSetpoint().velocity);

            SmartDashboard.putNumber("TxError", desiredTX - tx);
            SmartDashboard.putNumber("TyError", desiredTY - ty);

            SmartDashboard.putNumber("DesiredTx", desiredTX);
            SmartDashboard.putNumber("DesiredError", desiredTY);

            SmartDashboard.putBoolean("seenTarget", seenTarget);
            SmartDashboard.putBoolean("closeEnough?", visionSubsystem.isCloseEnough());
            SmartDashboard.putBoolean("centered?", visionSubsystem.isCentered());
            SmartDashboard.putNumber("NumberTargetsSeen", visionSubsystem.countObjects());
            SmartDashboard.putString("SwerveCommandkeykey", swerve.getCurrentCommand().toString());

        } else{
            timeouter.stop();
            timeouter.start();
            swerve.drive(new ChassisSpeeds(-0.4, 0, 0));
            SmartDashboard.putBoolean("seenTarget", seenTarget);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        timeouter.stop();
        timeouter.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timeouter.hasElapsed(1.5);
    }
}
