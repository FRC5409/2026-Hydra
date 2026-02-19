// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class ExtendCommand extends Command {
  private final Hopper hopper;
  private final Intake intake;
  private double distance;

  private static final Distance MIN_GAP = IntakeConstants.Extension.KILLSWITCH_TOLERANCE;

  public ExtendCommand(Hopper hopper, Intake intake) {
    this.hopper = hopper;
    this.intake = intake;
    addRequirements(hopper, intake);
  }

  @Override
  public void execute() {
    hopper.fullExtend();
    intake.extend();
  }

  @Override
  public boolean isFinished() {
    double hopperPos = hopper.getPosition().in(Inches);
    double intakePos = intake.getPosition().in(Inches);
    distance = hopperPos - intakePos;
    return distance < MIN_GAP.in(Inches);
  }

  @Override
  public void end(boolean interrupted) {
    hopper.stopMotor();
    intake.stopMotor();
  }
}