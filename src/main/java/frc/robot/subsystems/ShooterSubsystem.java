package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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

  private static final double SHOOTER_SPEED_FRONT = 50;
  private static final double SHOOTER_SPEED_BACK = 100;
  private static final double FEEDER_DUTY_CYCLE = 1.0;
  private static final double HOPPER_SPEED = 0.25;

  public ShooterSubsystem() {
    SmartDashboard.putNumber("Shooter/TargetSpeedFront", SHOOTER_SPEED_FRONT);
    SmartDashboard.putNumber("Shooter/TargetSpeedBack", SHOOTER_SPEED_BACK);
    this.shooterFront.getConfigurator().apply(
        new Slot0Configs().withKV(0.1111111).withKP(0.02));
    this.shooterBack.getConfigurator().apply(
        new Slot0Configs().withKV(0.1111111).withKP(0.02));
  }

  public Command spoolShootCommand(double speed) {
    return Commands.run(() -> {
      var targetSpeedFront = SmartDashboard.getNumber("Shooter/TargetSpeedFront", 0.0);
      var targetSpeedBack = SmartDashboard.getNumber("Shooter/TargetSpeedBack", 0.0);
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

  public Command hopperLoadCommand = Commands.run(() -> {
    hopperIntake.set(-HOPPER_SPEED);
  }).finallyDo(() -> {
    hopperIntake.set(0);
  });

  public Command floorPickupCommand = Commands.run(() -> {
    frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 1);
    bumperIntakeMotor.set(1);
  }).finallyDo(() -> {
    frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    bumperIntakeMotor.set(0);
  });
}
