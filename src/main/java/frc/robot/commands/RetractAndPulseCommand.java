// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.Intake;

public class RetractAndPulseCommand extends ParallelDeadlineGroup {
  private static final Distance MIN_GAP = HopperConstants.MIN_GAP_TO_INTAKE;
  private static final Distance INCREASE = HopperConstants.MAX_GAP_TO_INTAKE;
  private static final Distance TOLERANCE = HopperConstants.PULSE_TOLERANCE;
    
    /** Creates a new RetractCommand. */
    public RetractAndPulseCommand(Hopper hopper, Intake intake) {
      this(hopper, intake, true);
    }


    public RetractAndPulseCommand(Hopper hopper, Intake intake, boolean pulseHopper) {
      super(
        Commands.waitUntil(() ->
                hopper.getPosition().isNear(Inches.of(0.0), TOLERANCE) && 
                intake.getPosition().isNear(Inches.of(0.0), TOLERANCE)
        ),
        
        intake.retract(),

        pulseHopper 
          ? Commands.either(
              hopper.pulseWhileRetracting(
                          intake.getPosition().plus(MIN_GAP), 
                          intake.getPosition().plus(INCREASE)
              ),
              hopper.stopMotor(),
              () -> !hopper.getPosition().isNear(intake.getPosition(), MIN_GAP)
          ).repeatedly()

          : Commands.either(
            hopper.fullRetract(),
            hopper.stopMotor(),
            () -> !hopper.getPosition().isNear(Inches.of(0.0), TOLERANCE)
          ).repeatedly()

      );
      addRequirements(hopper, intake);

   }
}