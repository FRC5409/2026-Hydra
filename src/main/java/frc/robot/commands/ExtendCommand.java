// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendCommand extends ParallelDeadlineGroup {
  private static final double MIN_GAP = IntakeConstants.Extension.KILLSWITCH_TOLERANCE.in(Inches);
  private static final double STARTING_GAP = HopperConstants.STARTING_GAP_TO_INTAKE.in(Inches);

  /** Creates a new ExtendCommand. */
  public ExtendCommand(Hopper hopper, Intake intake) {

    super(
      Commands.waitUntil(() -> 
          (hopper.getPosition().in(Inches)+STARTING_GAP) - intake.getPosition().in(Inches) <= MIN_GAP ||
          hopper.getPosition().isNear(HopperConstants.HOPPER_MAX_EXTENSION, Inches.of(0.02))
      ),
      hopper.fullExtend(),
      intake.extend()
    );

  }
}
