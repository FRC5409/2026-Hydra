package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kBump;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoPath;
import frc.robot.util.FieldConstants.LinesHorizontal;
import frc.robot.util.FieldConstants.Tower;

public class Autos {

    public static ArrayList<AutoPath> getAutoPaths(Drive drive, Vision vision){
        ArrayList<AutoPath> autoPaths = new ArrayList<>();

        autoPaths.add(
            new AutoPath(
                "LeftBump-Intake-Score-LeftClimb",
                // Starting Pose: 
                new Pose2d(3.470,5.090, Rotation2d.k180deg),
                // Alliance -> neutral zone 
                DriveCommands.crossBump(drive, vision, () -> Rotation2d.k180deg, DriveCommands.getBumpSpeed(drive), kBump.SETTLING_TIME.in(Milliseconds)),
                // confirm position
                // DriveCommands.alignToHeading(
                //     drive,
                //     () -> new Rotation2d(Degrees.of(13))
                // ),
                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(6.200,5.090, new Rotation2d(Degrees.of(-146.651))), 
                    () -> MetersPerSecond.of(2.0), 
                    () -> MetersPerSecondPerSecond.of(8.0)
                ),
                // Follow path from center of neutral zone to left of field
                AutoPath.followPath("Left-Bump-IntakeEnd"),
                // Align back to bump known position
                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(6.200,(LinesHorizontal.leftBumpEnd + LinesHorizontal.leftBumpStart) / 2, new Rotation2d(Degrees.of(-50.711))), 
                    () -> MetersPerSecond.of(2.0), 
                    () -> MetersPerSecondPerSecond.of(8.0)
                ),
                // cross bump back into alliance zone
                DriveCommands.crossBump(drive, vision, () -> new Rotation2d(Degrees.of(-50.711)), DriveCommands.getBumpSpeed(drive), kBump.SETTLING_TIME.in(Milliseconds)),
                // Commands.runOnce(() ->Logger.recordOutput("Path/running: ", "wait") ),

                // Score
                Commands.waitTime(Seconds.of(7)),
                // Align to climber prep
                // Commands.runOnce(() ->Logger.recordOutput("Path/running: ", "prep") ),
                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(new Translation2d(Meters.of(Tower.leftUpright.getX()), Meters.of(5.00)), Rotation2d.k180deg), 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
                    kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
                    kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
                    kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
                ),
                // Commands.runOnce(() -> Logger.recordOutput("Path/running: ", "climb")),
                // Align to climb
                DriveCommands.alignToPoint(
                    drive, 
                    () -> (new Pose2d( 
                            new Translation2d( 
                                Meters.of(Tower.leftUpright.getX()), 
                                Meters.of( Tower.leftUpright.getY() + (DriveConstants.ROBOT_WIDTH.in(Meters) / 2) + kAutoAlign.CLIMBER_DISTANCE_FROM_UPRIGHT.in(Meters))
                            ), 
                            Rotation2d.k180deg
                        )
                    ), 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
                )
            )
        );


        autoPaths.add(
            new AutoPath(
                "RightBump-Intake-Score-RightClimb",
                // Starting Pose: 
                new Pose2d(3.470,3.071, Rotation2d.k180deg),
                // Alliance -> neutral zone 
                DriveCommands.crossBump(drive, vision, () -> Rotation2d.k180deg, DriveCommands.getBumpSpeed(drive), kBump.SETTLING_TIME.in(Milliseconds)),
                // confirm position
                // DriveCommands.alignToHeading(
                //     drive,
                //     () -> new Rotation2d(Degrees.of(13))
                // ),
                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(6.265,3.071, new Rotation2d(Degrees.of(3.071))), 
                    () -> MetersPerSecond.of(2.0), 
                    () -> MetersPerSecondPerSecond.of(8.0)
                ),
                // Follow path from center of neutral zone to left of field
                AutoPath.followPath("Right-Bump-IntakeEnd"),
                // Align back to bump known position
                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(6.200,(LinesHorizontal.rightBumpEnd + LinesHorizontal.rightBumpEnd) / 2, new Rotation2d(Degrees.of(-37))), 
                    () -> MetersPerSecond.of(2.0), 
                    () -> MetersPerSecondPerSecond.of(8.0)
                ),
                // cross bump back into alliance zone
                DriveCommands.crossBump(drive, vision, () -> drive.getRotation(), DriveCommands.getBumpSpeed(drive), kBump.SETTLING_TIME.in(Milliseconds)),
                // Commands.runOnce(() ->Logger.recordOutput("Path/running: ", "wait") ),
                // Score
                Commands.waitTime(Seconds.of(7)),
                // Align to climber prep
                // Commands.runOnce(() ->Logger.recordOutput("Path/running: ", "prep") ),
                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(new Translation2d(Meters.of(Tower.rightUpright.getX()), Meters.of(2.450)), Rotation2d.kZero), 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
                    kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
                    kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
                    kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
                ),
                // Commands.runOnce(() -> Logger.recordOutput("Path/running: ", "climb")),
                // Align to climb
                DriveCommands.alignToPoint(
                    drive, 
                    () -> (new Pose2d(
                        new Translation2d(
                            Meters.of(Tower.rightUpright.getX()), 
                            Meters.of(
                                Tower.rightUpright.getY() - (DriveConstants.ROBOT_WIDTH.in(Meters) / 2) - kAutoAlign.CLIMBER_DISTANCE_FROM_UPRIGHT.in(Meters)
                            )
                        ), 
                        Rotation2d.kZero)
                    ), 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
                )
            )
        );

        return autoPaths;
    }
    
}
