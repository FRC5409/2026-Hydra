// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class RetractWPulseCommand extends SequentialCommandGroup {

  private static Distance currentSetpoint = Inches.of(IntakeConstants.Extension.INITIAL_SETPOINT.in(Inches));

  public RetractWPulseCommand(Intake intake, Hopper hopper) {
    super(

      Commands.parallel(
          intake.move(currentSetpoint),
          hopper.setSetpoint(currentSetpoint)
      ),

      Commands.waitSeconds(IntakeConstants.Extension.WAIT_TIME.in(Seconds)),
      hopper.setSetpoint(currentSetpoint.plus(Inches.of(IntakeConstants.Extension.EXTEND_INCREMENT.in(Inches)))),
      Commands.waitSeconds(IntakeConstants.Extension.WAIT_TIME.in(Seconds)),
      Commands.runOnce(() -> currentSetpoint = currentSetpoint.minus(Inches.of(IntakeConstants.Extension.RETRACT_INCREMENT.in(Inches))))
    
    );
    repeatedly().until(() -> 
      Math.abs(intake.getPosition().in(Inches) - hopper.getPosition().in(Inches)) <= IntakeConstants.Extension.KILLSWITCH_TOLERANCE.in(Inches) || currentSetpoint.in(Inches) <= IntakeConstants.Extension.EXTENSION_MIN_DISTANCE.in(Inches)
    );
    addRequirements(intake, hopper);

  }
}