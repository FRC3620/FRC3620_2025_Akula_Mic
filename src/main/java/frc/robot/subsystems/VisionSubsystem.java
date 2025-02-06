// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  String llNameFront = "limelight-front";
  String llNameBack = "limelight-back";

  String bluePoseTopicName = "peBlue";
  String redPoseTopicName = "peRed";

  Map<String, StructPublisher<? extends Object>> publishers = new TreeMap<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    makePublishers(llNameFront);
    makePublishers(llNameBack);
  }

  @Override
  public void periodic() {
    updateFromCamera(llNameFront);
    updateFromCamera(llNameBack);
  }

  void updateFromCamera(String ll) {
    String prefix = "frc3620/vision/" + ll + "/";
    LimelightHelpers.PoseEstimate m = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
    SmartDashboard.putNumber(prefix + "targetCount", m.tagCount);
    publish(ll, bluePoseTopicName, m.pose);

    m = LimelightHelpers.getBotPoseEstimate_wpiRed(ll);
    SmartDashboard.putNumber(prefix + "targetCount", m.tagCount);
    publish(ll, redPoseTopicName, m.pose);

  }

  void makePublishers(String ll) {
    makePublisher(ll, bluePoseTopicName, Pose2d.struct);
    makePublisher(ll, redPoseTopicName, Pose2d.struct);
  }

  void makePublisher(String ll, String topicName, Struct<?> s) {
    String fullTopicName = fullTopicName(ll, topicName);
    StructPublisher<? extends Object> publisher = NetworkTableInstance.getDefault().getStructTopic(fullTopicName, s).publish();
    publishers.put(fullTopicName, publisher);
  }

  @SuppressWarnings({ "rawtypes", "unchecked" })
  void publish(String ll, String topicName, Object o) {
    String fullTopicName = fullTopicName(ll, topicName);
    StructPublisher publisher = publishers.get(fullTopicName);
    if (publisher != null) {
      publisher.set(o);
    }
  }

  String fullTopicName(String ll, String topicName) {
    StringBuilder sb = new StringBuilder("SmartDashboard/frc3620/vision/");
    sb.append(ll);
    sb.append("/");
    sb.append(topicName);
    return sb.toString();
  }
}
