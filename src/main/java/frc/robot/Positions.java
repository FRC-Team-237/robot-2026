package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Positions {
  public static final Translation2d hubBluePos = new Translation2d(Inches.of(182.11), Inches.of(158.84));
  public static final Translation2d hubRedPos = new Translation2d(Inches.of(469.11), Inches.of(158.84));

  public static Translation2d myHubPosition(SwerveDriveState driveState) {
    var baseHubPos = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? Positions.hubBluePos
        : Positions.hubRedPos;

    var speed = new Translation2d(
        Meters.of(driveState.Speeds.vxMetersPerSecond),
        Meters.of(driveState.Speeds.vyMetersPerSecond));

    var position = driveState.Pose.getTranslation();

    var distance = baseHubPos.minus(position).getNorm();

    // var multiplier = (2 * distance + 1) / 3;

    return baseHubPos.plus(speed.rotateBy(Rotation2d.k180deg).times(2));
  }

  public static Rotation2d angleToHub(SwerveDriveState driveState) {
    var position = driveState.Pose.getTranslation();
    var targetPos = Positions.myHubPosition(driveState);

    var originAdjustedTarget = targetPos.minus(position);
    var targetAngle = originAdjustedTarget.getAngle();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return targetAngle.plus(Rotation2d.k180deg);
    }
    return targetAngle;
  }
}
