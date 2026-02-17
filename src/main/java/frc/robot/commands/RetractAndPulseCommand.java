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
    public RetractAndPulseCommand(Hopper sys_hopper, Intake sys_intake) {
      this(sys_hopper, sys_intake, true);
    }


    public RetractAndPulseCommand(Hopper sys_hopper, Intake sys_intake, boolean pulseHopper) {
      super(
        Commands.waitUntil(() ->
                sys_hopper.getPosition().isNear(Inches.of(0.0), TOLERANCE) && 
                sys_intake.getPosition().isNear(Inches.of(0.0), TOLERANCE)
        ),
        
        sys_intake.retract(),

        pulseHopper 
          ? Commands.either(
              sys_hopper.pulseWhileRetracting(
                          sys_intake.getPosition().plus(MIN_GAP), 
                          sys_intake.getPosition().plus(INCREASE)
              ),
              sys_hopper.stopMotor(),
              () -> !sys_hopper.getPosition().isNear(sys_intake.getPosition(), MIN_GAP)
          ).repeatedly()

          : Commands.either(
            sys_hopper.fullRetract(),
            sys_hopper.stopMotor(),
            () -> !sys_hopper.getPosition().isNear(Inches.of(0.0), TOLERANCE)
          ).repeatedly()

      );
      addRequirements(sys_hopper, sys_intake);

   }
}