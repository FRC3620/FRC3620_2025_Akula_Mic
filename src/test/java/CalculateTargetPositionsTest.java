// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.WhichAlgae;
import frc.robot.subsystems.VisionSubsystem.WhichBlueStick;
import frc.robot.subsystems.VisionSubsystem.WhichRedStick;
import frc.robot.subsystems.VisionSubsystem.WhichStartingAlgae;

/** Add your docs here. */
public class CalculateTargetPositionsTest {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    @Test
    public void checkAlgaeStart() {
        checkAlgae(6, VisionSubsystem.WhichStartingAlgae.Algae6);
        checkAlgae(7, VisionSubsystem.WhichStartingAlgae.Algae7);
        checkAlgae(8, VisionSubsystem.WhichStartingAlgae.Algae8);
        checkAlgae(9, VisionSubsystem.WhichStartingAlgae.Algae9);
        checkAlgae(10, VisionSubsystem.WhichStartingAlgae.Algae10);
        checkAlgae(11, VisionSubsystem.WhichStartingAlgae.Algae11);
        System.out.println("-------------------");

        checkAlgae(17, VisionSubsystem.WhichStartingAlgae.Algae17);
        checkAlgae(18, VisionSubsystem.WhichStartingAlgae.Algae18);
        checkAlgae(19, VisionSubsystem.WhichStartingAlgae.Algae19);
        checkAlgae(20, VisionSubsystem.WhichStartingAlgae.Algae20);
        checkAlgae(21, VisionSubsystem.WhichStartingAlgae.Algae21);
        checkAlgae(22, VisionSubsystem.WhichStartingAlgae.Algae22);
        System.out.println("-------------------");
    }

    @Test
    public void checkAlgae() {
        checkAlgae(6, VisionSubsystem.WhichAlgae.Algae6);
        checkAlgae(7, VisionSubsystem.WhichAlgae.Algae7);
        checkAlgae(8, VisionSubsystem.WhichAlgae.Algae8);
        checkAlgae(9, VisionSubsystem.WhichAlgae.Algae9);
        checkAlgae(10, VisionSubsystem.WhichAlgae.Algae10);
        checkAlgae(11, VisionSubsystem.WhichAlgae.Algae11);
        System.out.println("-------------------");

        checkAlgae(17, VisionSubsystem.WhichAlgae.Algae17);
        checkAlgae(18, VisionSubsystem.WhichAlgae.Algae18);
        checkAlgae(19, VisionSubsystem.WhichAlgae.Algae19);
        checkAlgae(20, VisionSubsystem.WhichAlgae.Algae20);
        checkAlgae(21, VisionSubsystem.WhichAlgae.Algae21);
        checkAlgae(22, VisionSubsystem.WhichAlgae.Algae22);
        System.out.println("-------------------");
    }

    void checkAlgae(int tagId, WhichAlgae algae) {
        check("Algae " + tagId, aprilTagFieldLayout.getTagPose(tagId).get().toPose2d(), algae.pose);
    }

    void checkAlgae(int tagId, WhichStartingAlgae algae) {
        check("Algae " + tagId, aprilTagFieldLayout.getTagPose(tagId).get().toPose2d(), algae.pose);
    }

    void check(String what, Pose2d p1, Pose2d p2) {
        var distance = p1.getTranslation().getDistance(p2.getTranslation());
        var vector = p1.getTranslation().minus(p2.getTranslation()).getAngle().minus(p1.getRotation());
        System.out.println(what + "," + distance + "," + vector.getDegrees());
    }

    @Test
    public void checkLSticks() {
        checkStick("Left", 17, WhichBlueStick.BSTICKI);
        checkStick("Left", 18, WhichBlueStick.BSTICKG);
        checkStick("Left", 19, WhichBlueStick.BSTICKE);
        checkStick("Left", 20, WhichBlueStick.BSTICKC);
        checkStick("Left", 21, WhichBlueStick.BSTICKA);
        checkStick("Left", 22, WhichBlueStick.BSTICKK);
        System.out.println("-------------------");

        /*
        Pose2d bstickPrime1 = new Pose2d(4.98, 2.98, Rotation2d.fromDegrees(120));
        check("adjusted1" + bstickPrime1, aprilTagFieldLayout.getTagPose(22).get().toPose2d(), bstickPrime1);

        {
            Pose2d p1 = aprilTagFieldLayout.getTagPose(22).get().toPose2d();
            for (double x = 4.95; x < 5.05; x += 0.005) {
                for (double y = 2.8; y < 3.1; y += 0.005) {
                    Pose2d p2 = new Pose2d(x, y, Rotation2d.fromDegrees(120));
                    var distance = p1.getTranslation().getDistance(p2.getTranslation());
                    var vector = p1.getTranslation().minus(p2.getTranslation()).getAngle().minus(p1.getRotation());
                    var d = vector.getDegrees();
                    if (distance > 0.41 && distance < 0.43 && d > 158 && d < 162) {
                        System.out.println("" + x + " " + y + "," + distance + "," + d);
                    }
                }
            }
            System.out.println("-------------------");
        }
        */


      checkStick("Left", 6, WhichRedStick.RSTICKK);
      checkStick("Left", 7, WhichRedStick.RSTICKA);
      checkStick("Left", 8, WhichRedStick.RSTICKC);
      checkStick("Left", 9, WhichRedStick.RSTICKE);
      checkStick("Left", 10, WhichRedStick.RSTICKG);
      checkStick("Left", 11, WhichRedStick.RSTICKI);
      
      System.out.println("-------------------");
    }

    @Test
    public void checkRSticks() {
        checkStick("Right", 17, WhichBlueStick.BSTICKJ);
        checkStick("Right", 18, WhichBlueStick.BSTICKH);
        checkStick("Right", 19, WhichBlueStick.BSTICKF);
        checkStick("Right", 20, WhichBlueStick.BSTICKD);
        checkStick("Right", 21, WhichBlueStick.BSTICKB);
        checkStick("Right", 22, WhichBlueStick.BSTICKL);
        System.out.println("-------------------");

        checkStick("Right", 6, WhichRedStick.RSTICKL);
        checkStick("Right", 7, WhichRedStick.RSTICKB);
        checkStick("Right", 8, WhichRedStick.RSTICKD);
        checkStick("Right", 9, WhichRedStick.RSTICKF);
        checkStick("Right", 10, WhichRedStick.RSTICKH);
        checkStick("Right", 11, WhichRedStick.RSTICKJ);
        System.out.println("-------------------");
    }

    void checkStick(String side, int tagId, WhichRedStick stick) {
        check(side + " Redstick: " + tagId, aprilTagFieldLayout.getTagPose(tagId).get().toPose2d(), stick.pose);
    }

    void checkStick(String side, int tagId, WhichBlueStick stick) {
        check(side + " Bluestick: " + tagId, aprilTagFieldLayout.getTagPose(tagId).get().toPose2d(), stick.pose);
    }
}
