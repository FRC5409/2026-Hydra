// src/main/java/frc/robot/commands/AutoClimb.java

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.kAutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

/**
 * Auto climb sequence:
 * 1) In parallel: align to prep pose + raise climber (elevator) to prep height
 * 2) Align to final climbing pose
 * 3) Lower climber (elevator) to min height
 */
public class AutoClimb extends SequentialCommandGroup {

  public AutoClimb(
      Drive sys_drive,
      Elevator sys_elevator,
      Supplier<Pose2d> prepPoseSupplier,
      Supplier<Pose2d> climbPoseSupplier) {

    addCommands(
        Commands.parallel(
            DriveCommands.alignToPoint(
                sys_drive,
                prepPoseSupplier,
                () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
                () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
                kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
                kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
                kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP),
            sys_elevator.elevatorGo(ElevatorConstants.kSetpoints.ELEVATOR_UP)
        ),

        DriveCommands.alignToPoint(
            sys_drive,
            climbPoseSupplier,
            () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB,
            () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
        ),

        sys_elevator.elevatorGo(ElevatorConstants.kSetpoints.ELEVATPR_DOWN)
    );
  }
}