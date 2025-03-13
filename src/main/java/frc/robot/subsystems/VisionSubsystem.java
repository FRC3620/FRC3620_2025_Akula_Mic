package frc.robot.subsystems;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.usfirst.frc3620.NTStructs;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.VisionSubsystem.CameraData.MegaTagData;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);

  private Map<Translation2d, Integer> translationToTagMap = new HashMap<>();

  private Map<Integer, Translation2d> tagToTranslationMap = new HashMap<>();

  private Map<Integer, Pose2d> tagToStickPose2dLeft = new HashMap<>();

  private Map<Integer, Pose2d> tagToStickPose2dRight = new HashMap<>();

  private Translation2d centerBlueReef;

  private List<Translation2d> tagTranslations = new ArrayList<>();

  static Optional<Alliance> color;

  double maxDistanceFromCenterToBeClose = 3;// Distance in meters

  String lastLoggedError;

  public enum Camera {
    FRONT("limelight-front"), BACK("limelight-back");

    public final String limelightName;

    Camera(String limelightName) {
      this.limelightName = limelightName;
    }
  }

  public enum WhichBlueStick {
    //as of 3/12/25 using pathplanner
    BSTICKA(5.814, 3.845, Rotation2d.fromDegrees(-180)),
    BSTICKB(5.802, 4.169, Rotation2d.fromDegrees(-180)),
    BSTICKC(5.323, 5.08, Rotation2d.fromDegrees(-120)),
    BSTICKD(5.011, 5.248, Rotation2d.fromDegrees(-120)),
    BSTICKE(3.968, 5.248, Rotation2d.fromDegrees(-60)),
    BSTICKF(3.68, 5.092, Rotation2d.fromDegrees(-60)),
    BSTICKG(3.177, 4.205, Rotation2d.fromDegrees(0)),
    BSTICKH(3.105, 3.857, Rotation2d.fromDegrees(0)),
    BSTICKI(3.728, 2.994, Rotation2d.fromDegrees(60)),
    BSTICKJ(3.968, 2.874, Rotation2d.fromDegrees(60)),
    BSTICKK(5, 2.814, Rotation2d.fromDegrees(120)),
    BSTICKL(5.299, 2.97, Rotation2d.fromDegrees(120));


    public final Pose2d pose;

    WhichBlueStick(double x, double y, Rotation2d rotation) {
      pose = new Pose2d(x, y, rotation);
    }
  }

  public class CameraData {
    final String limelightName;
    public final MegaTagData megaTag1 = new MegaTagData("megaTag1");
    public final MegaTagData megaTag2 = new MegaTagData("megaTag2");
    boolean useThisCamera = true;
    int countOfSwerveUpdatesFromThisCamera = 0;

    CameraData(Camera c) {
      limelightName = c.limelightName;
      /*
       * see
       * https://docs.wpilib.org/en/stable/docs/software/networktables/listening-for-
       * change.html#using-networktableinstance-to-listen-for-changes
       * 
       * we set the haveNew* atomic(s) when the corresponding network entry is
       * updated, and in periodic() we only go through all the work of reading
       * the network entry if it had been updated.
       */
      DoubleArrayEntry entry = LimelightHelpers.getLimelightDoubleArrayEntry(c.limelightName, "botpose_wpiblue");
      inst.addListener(
          entry,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> megaTag1.haveNewPose.set(true));

      entry = LimelightHelpers.getLimelightDoubleArrayEntry(c.limelightName, "botpose_orb_wpiblue");
      inst.addListener(
          entry,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> megaTag2.haveNewPose.set(true));
    }

    public String getLimelightName() {
      return limelightName;
    }

    public CameraData withUseThisCamera(boolean b) {
      useThisCamera = b;
      return this;
    }

    public boolean shouldUseThisCamera() {
      return useThisCamera;
    }

    public int bumpCountOfSwerveUpdatesFromThisCamera() {
      return ++countOfSwerveUpdatesFromThisCamera;
    }

    public class MegaTagData {
      // gets set whenever we have new data
      AtomicBoolean haveNewPose = new AtomicBoolean(false);

      final String megaTagName;

      PoseEstimate poseEstimate;

      double distanceToClosestSeenTarget;

      MegaTagData(String megaTagName) {
        this.megaTagName = megaTagName;
      }

      public String getMegaTagName() {
        return megaTagName;
      }

      public PoseEstimate getPoseEstimate() {
        return poseEstimate;
      }

      public String getLimelightName() {
        return limelightName;
      }

      public double getDistanceToClosestSeenTarget() {
        return distanceToClosestSeenTarget;
      }
    }
  }

  Map<Camera, CameraData> allCameraData = new TreeMap<>();
  Set<CameraData> allCameraDataAsSet;

  public VisionSubsystem() {
    allCameraData.put(Camera.FRONT, new CameraData(Camera.FRONT));
    allCameraData.put(Camera.BACK, new CameraData(Camera.BACK).withUseThisCamera(false));
    allCameraData = Map.copyOf(allCameraData); // make immutable
    allCameraDataAsSet = Set.copyOf(allCameraData.values());

    setUpTagMaps();
  }

  void processMegaTag(MegaTagData megaTagData, Supplier<PoseEstimate> supplier, Pose2d currentPose) {
    if (megaTagData.haveNewPose.getAndSet(false)) {
      PoseEstimate m = supplier.get();

      if (m != null) {
        megaTagData.poseEstimate = m;

        var prefix = "frc3620/vision/" + megaTagData.getLimelightName() + "/" + megaTagData.megaTagName
            + "/";

        SmartDashboard.putNumber(prefix + "targetCount", m.tagCount);
        NTStructs.publishToSmartDashboard(prefix + "poseEstimate", m.pose);
        // it doesn't seem that poses published to NT make it into the
        // wpilog file via NetworkTableInstance.startEntryDataLog, so let's be
        // explicit
        DogLog.log(prefix + "poseEstimate", m.pose);
        
        if (currentPose != null) {
          SmartDashboard.putNumber(prefix + "distanceFromSwervePose",
              currentPose.getTranslation().getDistance(m.pose.getTranslation()));
        }

        if (aprilTagFieldLayout != null) {
          List<Pose3d> targetPoses = new ArrayList<>();

          double distanceToClosestSeenTarget = 100000;

          for (var fiducial : m.rawFiducials) {
            Optional<Pose3d> aprilTagPose = aprilTagFieldLayout.getTagPose(fiducial.id);
            if (aprilTagPose.isPresent()) {
              targetPoses.add(aprilTagPose.get());
              double distanceToThisTag = m.pose.getTranslation()
                  .getDistance(aprilTagPose.get().getTranslation().toTranslation2d());
              if (distanceToThisTag < distanceToClosestSeenTarget) {
                distanceToClosestSeenTarget = distanceToThisTag;
              }
              megaTagData.distanceToClosestSeenTarget = distanceToClosestSeenTarget;
              SmartDashboard.putNumber(prefix + "distance to closest seen tag", distanceToClosestSeenTarget);
            }
          }
          NTStructs.publish(prefix + "targets", targetPoses.toArray(new Pose3d[0]));
        }
      }
    }
  }

  void setUpTagMaps() {

    for (int tagID = 1; tagID <= 22; tagID++) {

      Translation2d translation = RobotContainer.aprilTagFieldLayout.getTagPose(tagID).get().getTranslation()
          .toTranslation2d();

      translationToTagMap.put(translation, tagID);
      tagToTranslationMap.put(tagID, translation);

      tagTranslations.add(translation);

    }

    tagToStickPose2dLeft.put(17, WhichBlueStick.BSTICKI.pose);
    tagToStickPose2dLeft.put(18, WhichBlueStick.BSTICKG.pose);
    tagToStickPose2dLeft.put(19, WhichBlueStick.BSTICKE.pose);
    tagToStickPose2dLeft.put(20, WhichBlueStick.BSTICKC.pose);
    tagToStickPose2dLeft.put(21, WhichBlueStick.BSTICKA.pose);
    tagToStickPose2dLeft.put(22, WhichBlueStick.BSTICKK.pose);

    tagToStickPose2dRight.put(17, WhichBlueStick.BSTICKJ.pose);
    tagToStickPose2dRight.put(18, WhichBlueStick.BSTICKH.pose);
    tagToStickPose2dRight.put(19, WhichBlueStick.BSTICKF.pose);
    tagToStickPose2dRight.put(20, WhichBlueStick.BSTICKD.pose);
    tagToStickPose2dRight.put(21, WhichBlueStick.BSTICKB.pose);
    tagToStickPose2dRight.put(22, WhichBlueStick.BSTICKL.pose);

    centerBlueReef = tagToTranslationMap.get(17).plus(tagToTranslationMap.get(20)).div(2);

  }

  @Override
  public void periodic() {

    // gets alliance color
    color = DriverStation.getAlliance();

    // added this to handle case where color is not yet set, otherwise we blow up in
    // the simulator
    if (color.isEmpty())
      return;

    double yaw = 0;
    double yawRate = 0;
    double pitch = 0;
    SwerveDrive sd = null;
    Pose2d currentSwervePose = null;
    if (RobotContainer.swerveSubsystem != null) {
      sd = RobotContainer.swerveSubsystem.getSwerveDrive();
      yaw = sd.getYaw().getDegrees();
      pitch = sd.getPitch().getDegrees();
      // need to convert this to degrees / s.
      // yawRate = sd.getGyro().getYawAngularVelocity();
      currentSwervePose = sd.getPose();
    }

    for (var cameraData : allCameraData.values()) {
      var sdPrefix = "frc3620/vision/" + cameraData.getLimelightName() + "/";

      LimelightHelpers.SetRobotOrientation(cameraData.limelightName, yaw, yawRate, pitch, 0, 0, 0);
      processMegaTag(cameraData.megaTag1, () -> LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraData.limelightName),
          currentSwervePose);
      processMegaTag(cameraData.megaTag2,
          () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraData.limelightName), currentSwervePose);

      // update robot odometry from vision

      String error = "";
      if (! cameraData.shouldUseThisCamera()) {
        error = "Ignoring this camera";
      } if (Math.abs(yawRate) > 720) {
        // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        error = "Angular Velocity";
      } else if (cameraData.megaTag2.poseEstimate == null) {
        error = "megaTag2Pose = null";
      } else if (cameraData.megaTag2.poseEstimate.tagCount == 0) {
        error = "No Visible Tags";
      } else if (cameraData.megaTag2.getDistanceToClosestSeenTarget() > 4.75) {
        error = "Closest Tag Too Far";
      } else if (sd == null) {
        error = "No Swerve Drive";
      } else if (cameraData.megaTag2.poseEstimate.pose.getX() >= 17) {
        error = "Pose has too much X";
      } else if (cameraData.megaTag2.poseEstimate.pose.getX() <= 0) {
        error = "Pose has too little X";
      } else if (cameraData.megaTag2.poseEstimate.pose.getY() >= 8.5) {
        error = "Pose has too much Y";
      } else if (cameraData.megaTag2.poseEstimate.pose.getY() <= 0) {
        error = "Pose has too little Y";
      }
      if (error.length() == 0) {
        double distanceError = sd.getPose().getTranslation()
            .getDistance(cameraData.megaTag2.poseEstimate.pose.getTranslation());

        double translationStdDev = Math.min(50.0, Math.max(0.4, distanceError * 2.0));
        // double rotationStdDev = Math.min(50.0, Math.max(0.3, distanceError * 0.3));

        // sd.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));//originally
        // .7, .7, 9999999
        sd.setVisionMeasurementStdDevs(VecBuilder.fill(translationStdDev, translationStdDev, 9999999));

        sd.addVisionMeasurement(cameraData.megaTag2.poseEstimate.pose,
            cameraData.megaTag2.poseEstimate.timestampSeconds);

        int updateCount = cameraData.bumpCountOfSwerveUpdatesFromThisCamera();
        SmartDashboard.putNumber(sdPrefix + "swervePoseUpdates", updateCount);
      }
      if (error != lastLoggedError) {
        // log if it changed

        // and remember!
        lastLoggedError = error;
      }
      SmartDashboard.putString(sdPrefix + "rejectionMessage", error);
    }

    // we are not using this, so commented out to try to speed up code a little
    /*
    if (RobotContainer.swerveSubsystem != null) {
      SmartDashboard.putNumber("frc3620/vision/nearestTagID",
          getNearestTagID(RobotContainer.swerveSubsystem.getPose()));
    }
    */

  }

  public CameraData getCameraData(Camera camera) {
    return allCameraData.get(camera);
  }

  public Set<CameraData> getAllCameraData() {
    return allCameraDataAsSet;
  }

  public int getNearestTagID(Pose2d pose) {
    Translation2d translation = pose.getTranslation();
    Translation2d nearestTagTranslation = translation.nearest(tagTranslations);

    if (translation.getDistance(centerBlueReef) < maxDistanceFromCenterToBeClose) {
      return translationToTagMap.get(nearestTagTranslation);
    } else {
      return -1;
    }
  }

  public Pose2d getNearestLeftStickPose(int tagID) {
    if (tagID == -1) {
      return RobotContainer.swerveSubsystem.getPose();
    } else {
      return tagToStickPose2dLeft.get(tagID);
    }
  }

  public Pose2d getNearestRightStickPose(int tagID) {
    if (tagID == -1) {
      return RobotContainer.swerveSubsystem.getPose();
    } else {
      return tagToStickPose2dRight.get(tagID);
    }
  }

}
