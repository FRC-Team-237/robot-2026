package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  VictorSPX frontIntakeMotor = new VictorSPX(31);
  SparkMax bumperIntakeMotor = new SparkMax(33, MotorType.kBrushless);

  SparkMax hopperIntake = new SparkMax(32, MotorType.kBrushless);

  TalonFX shooterBack = new TalonFX(21);
  TalonFX shooterFeeder = new TalonFX(22);
  TalonFX shooterFront = new TalonFX(23);

  private static final double FEEDER_DUTY_CYCLE = 1.0;
  private static final double HOPPER_SPEED = 0.25;
  private static final Translation2d hubBluePos = new Translation2d(Inches.of(182.11), Inches.of(158.84));
  private static final Translation2d hubRedPos = new Translation2d(Inches.of(469.11), Inches.of(158.84));

  public ShooterSubsystem() {
    this.shooterFront.getConfigurator().apply(
        new Slot0Configs().withKV(0.1111111).withKP(0.02).withKA(0.01));
    this.shooterBack.getConfigurator().apply(
        new Slot0Configs().withKV(0.1111111).withKP(0.02));
  }

  public Command spoolShootCommand(Supplier<Translation2d> robotPosition) {
    return Commands.run(() -> {
      var latestPos = robotPosition.get();
      var targetPos = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
          ? hubBluePos
          : hubRedPos;
      var distanceToTarget = Feet.convertFrom(latestPos.getDistance(targetPos), Meters);

      double targetSpeedFront = frontSpinSpeed(distanceToTarget);
      double targetSpeedBack = backspinSpeed(distanceToTarget);
      SmartDashboard.putNumber("FrontSpeed", targetSpeedFront);
      SmartDashboard.putNumber("BackSpeed", targetSpeedBack);
      SmartDashboard.putNumber("Distance", distanceToTarget);

      this.shooterFront.setControl(new VelocityVoltage(targetSpeedFront));
      shooterBack.setControl(new VelocityVoltage(-targetSpeedBack));
    }).finallyDo(() -> {
      shooterFront.set(0);
      shooterBack.set(0);
    });
  }

  public Command shootCommand = Commands.run(() -> {
    shooterFeeder.set(-FEEDER_DUTY_CYCLE);
  }).finallyDo(() -> {
    shooterFeeder.set(0);
  });

  public Command intakeCommand = Commands.run(() -> {
    frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 1);
    bumperIntakeMotor.set(1);
    hopperIntake.set(-HOPPER_SPEED);
  }).finallyDo(() -> {
    frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    bumperIntakeMotor.set(0);
    hopperIntake.set(0);
  });

  public double backspinSpeed(double dist) {
    double m = 2.2942;
    double b = 13.27157;
    return m * (dist - 2) + b;
  }

  public double frontSpinSpeed(double dist) {
    double m = 0.684583;
    double b = 49.47808;
    return m * (dist - 2) + b;
  }
}
