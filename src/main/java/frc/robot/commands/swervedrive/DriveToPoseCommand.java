// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToPoseCommand extends Command {
    private final SwerveSubsystem swerve;
    Pose2d targetPose;
    double targetRotation;
    private final double maxSwerveVelocity;
    private final double maxSwerveAngularVelocity;
    private double driveKp = 5.5;
    private double turnKp = 0.12;
    private PIDController xController;
    private PIDController yController;
    private double xVelocity;
    private double yVelocity;
    private double angVelocity;
    private boolean targetIsSet = false;
    private int counter = 0;

    public DriveToPoseCommand(
            SwerveSubsystem swerve,
            Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;

        this.maxSwerveVelocity = swerve.getSwerveDrive().getMaximumChassisVelocity();
        this.maxSwerveAngularVelocity = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();

        xController = new PIDController(driveKp, 0.0, 0.0);
        yController = new PIDController(driveKp, 0.0, 0.0);

        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xController.setTolerance(0.05, 1.0); // 2 inches
        yController.setTolerance(0.05, 1.0); // 2 inches
        counter = 0;
        targetIsSet = false;
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("target is set", targetIsSet);
        SmartDashboard.putNumber("driveToPose counter", counter);
        SmartDashboard.putBoolean("driveToPose at setpoint", xController.atSetpoint());

        targetRotation = targetPose.getRotation().getDegrees();

        xVelocity = xController.calculate(swerve.getPose().getX());
        yVelocity = yController.calculate(swerve.getPose().getY());
        angVelocity = turnKp * Math.IEEEremainder(targetRotation - swerve.getHeading().getDegrees(), 360);

        swerve.driveFieldOriented(
                new ChassisSpeeds(
                        xVelocity * maxSwerveVelocity * 0.1,
                        yVelocity * maxSwerveVelocity * 0.1,
                        angVelocity));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && Math.abs(angVelocity) < 0.05;
    }
}