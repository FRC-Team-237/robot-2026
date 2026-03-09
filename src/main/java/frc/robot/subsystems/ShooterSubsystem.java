package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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

  private CommandSwerveDrivetrain drivetrain;
  private boolean shouldIntake = false;
  private boolean overOverideIntake = false;

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    var config = new TalonFXConfiguration();
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kV = 0.1316;
    config.Slot0.kP = 0.02;
    config.Slot0.kA = 0.01;

    this.shooterFront.getConfigurator().apply(config);
    this.shooterBack.getConfigurator().apply(config);
  }

  final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(false);

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

      var voltageAdjustmentRatio = voltageAjuster();

      SmartDashboard.putNumber("VoltageAdjustment", voltageAdjustmentRatio);

      this.shooterFront.setControl(velocityControl.withVelocity(targetSpeedFront * voltageAdjustmentRatio));
      shooterBack.setControl(velocityControl.withVelocity(-targetSpeedBack * voltageAdjustmentRatio));
    }).finallyDo(() -> {
      shooterFront.set(0);
      shooterBack.set(0);
    });
  }

  public Command shootCommand = Commands.run(() -> {
    shooterFeeder.set(-FEEDER_DUTY_CYCLE);
    hopperIntake.set(-HOPPER_SPEED);
  }).finallyDo(() -> {
    shooterFeeder.set(0);
    hopperIntake.set(0);
  });

  public Command reverseShootCommand = Commands.run(() -> {
    shooterFeeder.set(FEEDER_DUTY_CYCLE);
  }).finallyDo(() -> {
    shooterFeeder.set(0);
  });

  public Command intakeCommand = Commands.run(() -> {
    shouldIntake = true;
  }).finallyDo(() -> {
    shouldIntake = false;
  });

  public Command stopIntakeCommand = Commands.run(() -> {
    overOverideIntake = true;
  }).finallyDo(() -> {
    overOverideIntake = false;
  });

  public Command reverseIntakeCommand = Commands.run(() -> {
    frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, -1);
    bumperIntakeMotor.set(-1);
    hopperIntake.set(HOPPER_SPEED);
  }).finallyDo(() -> {
    frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    bumperIntakeMotor.set(0);
    hopperIntake.set(0);
  });

  public double backspinSpeed(double dist) {
    double m = 2.42159;
    double b = -2.20966;
    return (m * (dist) + b);
  }

  public double frontSpinSpeed(double dist) {
    double m = 1.93656;
    double b = 47.73898;
    return (m * (dist) + b);
  }

  public double voltageAjuster() {
    double voltage = RobotController.getBatteryVoltage();
    double center = 11.5;
    double steepness = 8;
    double maxBoost = 0.15;
    double denominator = 1 + (Math.pow(Math.E, steepness * (voltage - center)));
    return maxBoost / denominator + 1;

  }

  @Override
  public void periodic() {
    var robotPose = this.drivetrain.getState().Pose;
    var robotXInches = Inches.convertFrom(robotPose.getX(), Meters);
    boolean overrideShouldIntake;
    overrideShouldIntake = robotXInches > 182.11 && robotXInches < 469.11;
    if (overOverideIntake) {
      overrideShouldIntake = false;
    }
    boolean newShouldIntake;

    if (overrideShouldIntake) {
      newShouldIntake = true;
    } else {
      newShouldIntake = shouldIntake;
    }

    if (newShouldIntake) {
      frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 1);
      bumperIntakeMotor.set(1.0);
      hopperIntake.set(-HOPPER_SPEED);
    } else {
      frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
      bumperIntakeMotor.set(0);
      hopperIntake.set(0);
    }
  }
}