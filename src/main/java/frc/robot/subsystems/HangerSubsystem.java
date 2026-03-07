// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangerSubsystem extends SubsystemBase {

  SparkMax hangerMotor = new SparkMax(41, MotorType.kBrushless);
  public Command hangerUpCommand = Commands.run(() -> {
    hangerMotor.set(1);
  }).finallyDo(() -> {
    hangerMotor.set(0);
  });
  public Command hangerDownCommand = Commands.run(() -> {
    hangerMotor.set(-1);
  }).finallyDo(() -> {
    hangerMotor.set(0);
  });

}
