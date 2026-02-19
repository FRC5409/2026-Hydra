package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class FullRetractCommand extends SequentialCommandGroup {

    public FullRetractCommand(Intake intake, Hopper hopper) {

        super(
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    intake.retract(),
                    hopper.fullRetract()
                ),

                new WaitUntilCommand(() ->
                    Math.abs(intake.getPosition().in(Inches) - hopper.getPosition().in(Inches)) <= IntakeConstants.Extension.KILLSWITCH_TOLERANCE.in(Inches)
                )
            )
        );

        addRequirements(intake, hopper);
    }
}
