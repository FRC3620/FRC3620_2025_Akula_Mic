// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToPoseCommand extends Command {
    private final SwerveSubsystem swerve;
    Pose2d targetPose;
    double targetRotation;
    private final double maxSwerveVelocity;
    private final double maxSwerveAngularVelocity;
    private double driveKp = 5.7; //5.5
    private double turnKp = 0.12;
    //private PIDController xController;
    //private PIDController yController;
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    // Motion constraints
    private static final double MAX_ACCELERATION = 1.75; // Max linear acceleration (m/s^2)
    private static final double MAX_ANGULAR_ACCELERATION = Math.PI / 2; // Max angular acceleration (rad/s^2)
    private double xVelocity;
    private double yVelocity;
    private double angVelocity;
    private boolean targetIsSet = false;
    private int counter = 0;
    private Timer commandTimer = new Timer();
    private double COMMAND_TIMEOUT = 2.1;
    private boolean driveToPoseRunning = false;

    public DriveToPoseCommand(
            SwerveSubsystem swerve,
            Pose2d _targetPose) {
        this.swerve = swerve;
        this.targetPose = _targetPose;

        this.maxSwerveVelocity = swerve.getSwerveDrive().getMaximumChassisVelocity();
        this.maxSwerveAngularVelocity = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();

        //xController = new PIDController(driveKp, 0.0, 0.0);
        //yController = new PIDController(driveKp, 0.0, 0.0);

        // Initialize controllers with motion constraints
        xController = new ProfiledPIDController(driveKp, 0.0, 0.0, new TrapezoidProfile.Constraints(maxSwerveVelocity, MAX_ACCELERATION));
        yController = new ProfiledPIDController(driveKp, 0.0, 0.0, new TrapezoidProfile.Constraints(maxSwerveVelocity, MAX_ACCELERATION));


        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        driveToPoseRunning = true;
        SmartDashboard.putBoolean("frc3620/driveToPose/running", driveToPoseRunning);
        if(targetPose != null) {
            xController.reset(swerve.getPose().getX());
            yController.reset(swerve.getPose().getY());
    
            xController.setTolerance(0.02, 0.5);
            yController.setTolerance(0.02, 0.5); 
            counter = 0;
            targetIsSet = false;
            xController.setGoal(targetPose.getX());
            yController.setGoal(targetPose.getY());
    
            commandTimer.reset();
            commandTimer.start();  
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("target is set", targetIsSet);
        //SmartDashboard.putNumber("driveToPose counter", counter);
        SmartDashboard.putBoolean("driveToPose at setpoint", xController.atSetpoint());
        

        if(targetPose != null) {
            targetRotation = targetPose.getRotation().getDegrees();
        } else {
            targetRotation = swerve.getHeading().getDegrees();
        }
        
        xVelocity = xController.calculate(swerve.getPose().getX());
        yVelocity = yController.calculate(swerve.getPose().getY());
        angVelocity = turnKp * Math.IEEEremainder(targetRotation - swerve.getHeading().getDegrees(), 360);

        swerve.driveFieldOriented(
                new ChassisSpeeds(
                        xVelocity * maxSwerveVelocity * 0.2,
                        yVelocity * maxSwerveVelocity * 0.2,
                        angVelocity));
        SmartDashboard.putNumber("frc3620/driveToPose/xVelocity", xVelocity * maxSwerveVelocity * 0.2);
        SmartDashboard.putNumber("frc3620/driveToPose/yVelocity", yVelocity * maxSwerveVelocity * 0.2);
        SmartDashboard.putNumber("frc3620/driveToPose/angVelocity", angVelocity);
        SmartDashboard.putNumber("frc3620/driveToPose/xError", xController.getPositionError());
        SmartDashboard.putNumber("frc3620/driveToPose/yError", yController.getPositionError());
    }

    @Override
    public void end(boolean interrupted) {
        driveToPoseRunning = false;
        SmartDashboard.putBoolean("frc3620/driveToPose/running", driveToPoseRunning);
    }

    @Override
    public boolean isFinished() {

        if (commandTimer.hasElapsed(COMMAND_TIMEOUT)) {
            return true;
        } else {
            return (xController.atSetpoint() && yController.atSetpoint() && Math.abs(angVelocity) < 0.05);
        }
    }
}