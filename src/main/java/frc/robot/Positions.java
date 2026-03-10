package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Positions {
  public static final Translation2d hubBluePos = new Translation2d(Inches.of(182.11), Inches.of(158.84));
  public static final Translation2d hubRedPos = new Translation2d(Inches.of(469.11), Inches.of(158.84));

  public static Translation2d myHubPosition() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? Positions.hubBluePos
        : Positions.hubRedPos;
  }

  public static Rotation2d angleToHub(Translation2d position) {
    var targetPos = Positions.myHubPosition();

    var originAdjustedTarget = targetPos.minus(position);
    var targetAngle = originAdjustedTarget.getAngle();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return targetAngle.plus(Rotation2d.k180deg);
    }
    return targetAngle;
  }
}
