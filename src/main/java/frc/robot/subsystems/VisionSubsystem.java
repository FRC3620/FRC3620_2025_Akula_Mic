// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.NTPublisher;
import org.usfirst.frc3620.NTStructs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  String llNameFront = "limelight-front";
  String llNameBack = "limelight-back";

  String bluePoseTopicName = "peBlue";

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
  }

  @Override
  public void periodic() {
    updateFromCamera(llNameFront);
    updateFromCamera(llNameBack);
  }

  void updateFromCamera(String ll) {
    String prefix = "SmartDashboard/frc3620/vision/" + ll + "/";
    LimelightHelpers.PoseEstimate m = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
    if (m != null) {
      NTPublisher.putNumber(prefix + "targetCount", m.tagCount);
      NTStructs.publish(prefix + bluePoseTopicName, m.pose);
    }
  }

}
