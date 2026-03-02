package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.util.FieldConstants.Hub;
import frc.robot.util.FieldConstants.LinesHorizontal;
import frc.robot.util.FieldConstants.Tower;

public class Autos {

	public static ArrayList<AutoPath> getAutoPaths(Drive drive, Vision vision){
		ArrayList<AutoPath> autoPaths = new ArrayList<>();

		// LEFT SIDE AUTOS:
		autoPaths.add(
			new AutoPath(
				"LeftBump-Intake-CloseFar-Score-LeftClimb",

				// Angled Starting pose:
				// new Pose2d(3.470,5.400, new Rotation2d(Degrees.of(-146.651))),
				new Pose2d(3.470,5.400, new Rotation2d(Degrees.of(38.572))),

				// Starting Pose: 
				// new Pose2d(3.470,5.400, Rotation2d.k180deg),

				// Alliance -> neutral zone 
				DriveCommands.crossBump(
					drive, 
					vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED), 
					kBump.SETTLING_TIME
				),

				// confirm position
				DriveCommands.alignToPoint(
					drive,
					() -> new Pose2d(6.200,5.400, new Rotation2d(Degrees.of(38.572))), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

				// Follow path from center of neutral zone to left of field
				AutoPath.followPath("Left-Bump-Intake-CloseFar"),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(6.200,(LinesHorizontal.leftBumpEnd + LinesHorizontal.leftBumpStart) / 2, Rotation2d.k180deg), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					drive, 
					vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
					kBump.SETTLING_TIME
				),

				// Score
				DriveCommands.alignToHeading(
					drive, 
					() -> DriveCommands.getRotation2d(
						drive, 
						new Pose2d(
						new Translation2d(Hub.topCenterPoint.getMeasureX(), Hub.topCenterPoint.getMeasureY()), 
						Rotation2d.kZero
						)
					).plus(Rotation2d.k180deg)
				),
				Commands.waitTime(Seconds.of(5)),

				// Align to climber prep
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(new Translation2d(Meters.of(Tower.leftUpright.getX()), Meters.of(5.00)), Rotation2d.kZero), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
					kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
					kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
					kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
				),

				// Align to climb
				DriveCommands.alignToPoint(
					drive, 
					() -> (new Pose2d( 
							new Translation2d( 
								Meters.of(Tower.leftUpright.getX()), 
								Meters.of( Tower.leftUpright.getY() + (DriveConstants.ROBOT_WIDTH.in(Meters) / 2) + kAutoAlign.CLIMBER_DISTANCE_FROM_UPRIGHT.in(Meters))
							), 
							Rotation2d.kZero
						)
					), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
				)
			)
		);

		autoPaths.add(
			new AutoPath(
				"LeftBump-Intake-FarClose-Score-LeftClimb",
				// Angled Start:
				new Pose2d(3.470,5.801, new Rotation2d(Degrees.of(-43.361))),            
				// Starting Pose: 
				// new Pose2d(3.470,5.801, Rotation2d.k180deg),

				// Alliance -> neutral zone 
				DriveCommands.crossBump(
					drive, 
					vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED), 
					kBump.SETTLING_TIME
				),

				// confirm position
				DriveCommands.alignToPoint(
					drive,
					() -> new Pose2d(6.187,5.969, new Rotation2d(Degrees.of(-43.361))), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

				// Follow path from center of neutral zone to left of field
				AutoPath.followPath("Left-Bump-Intake-FarClose"),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(6.200,(LinesHorizontal.leftBumpEnd + LinesHorizontal.leftBumpStart) / 2, Rotation2d.k180deg), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					drive, 
					vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
					kBump.SETTLING_TIME
				),

				// Score
				DriveCommands.alignToHeading(
					drive, 
					() -> DriveCommands.getRotation2d(
						drive, 
						new Pose2d(
						new Translation2d(Hub.topCenterPoint.getMeasureX(), Hub.topCenterPoint.getMeasureY()), 
						Rotation2d.kZero
						)
					).plus(Rotation2d.k180deg)
				),
				Commands.waitTime(Seconds.of(5)),

				// Align to climber prep
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(new Translation2d(Meters.of(Tower.leftUpright.getX()), Meters.of(5.00)), Rotation2d.kZero), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
					kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
					kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
					kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
				),

				// Align to climb
				DriveCommands.alignToPoint(
					drive, 
					() -> (new Pose2d( 
							new Translation2d( 
								Meters.of(Tower.leftUpright.getX()), 
								Meters.of( Tower.leftUpright.getY() + (DriveConstants.ROBOT_WIDTH.in(Meters) / 2) + kAutoAlign.CLIMBER_DISTANCE_FROM_UPRIGHT.in(Meters))
							), 
							Rotation2d.kZero
						)
					), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
				)
			)
		);

		// RIGHT SIDE AUTOS:
		autoPaths.add(
			new AutoPath(
				"RightBump-Intake-CloseFar-Score-RightClimb",

				// Starting Pose: 
				// new Pose2d(3.470,2.750, Rotation2d.k180deg),

				// Angled Starting Pose
				new Pose2d(3.470,2.750, new Rotation2d(Degrees.of(-41.689))),
				
				// Alliance -> neutral zone
				DriveCommands.crossBump(
					drive, 
					vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED), 
					kBump.SETTLING_TIME
				),

				// confirm position
				DriveCommands.alignToPoint(
					drive, 

					() -> new Pose2d(6.265,2.750, new Rotation2d(Degrees.of(-41.689))), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),
				// Follow path from center of neutral zone to left of field
				AutoPath.followPath("Right-Bump-Intake-CloseFar"),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(6.200,(LinesHorizontal.rightBumpStart + LinesHorizontal.rightBumpEnd) / 2, Rotation2d.k180deg), 
					() -> MetersPerSecond.of(2.0), 
					() -> MetersPerSecondPerSecond.of(8.0)
				),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					drive, vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
					kBump.SETTLING_TIME
				),

				// Score
				DriveCommands.alignToHeading(
					drive, 
					() -> DriveCommands.getRotation2d(
						drive, 
						new Pose2d(
						new Translation2d(Hub.topCenterPoint.getMeasureX(), Hub.topCenterPoint.getMeasureY()), 
						Rotation2d.kZero
						)
					).plus(Rotation2d.k180deg)
				),
				Commands.waitTime(Seconds.of(5)),

				// Align to climber prep
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(new Translation2d(Meters.of(Tower.rightUpright.getX()), Meters.of(2.450)), Rotation2d.k180deg), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
					kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
					kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
					kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
				),

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
						Rotation2d.k180deg)
					), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
				)
			)
		);

		// Right Side
		// Bump
		// Intake from the edge of the field to the center
		// Go back and score
		// Climb
		autoPaths.add(
			new AutoPath(
				"RightBump-Intake-FarClose-Score-RightClimb",

				// Starting Pose: 
				// new Pose2d(3.470,2.750, Rotation2d.k180deg),

				// Angled Starting Pose
				new Pose2d(3.470,2.282, new Rotation2d(Degrees.of(53.181))),
				
				// Alliance -> neutral zone
				DriveCommands.crossBump(
					drive, 
					vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED), 
					kBump.SETTLING_TIME
				),

				// confirm position
				DriveCommands.alignToPoint(
					drive, 

					() -> new Pose2d(6.200,2.282, new Rotation2d(Degrees.of(53.181))), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),
				// Follow path from center of neutral zone to left of field
				AutoPath.followPath("Right-Bump-Intake-FarClose"),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(6.200,(LinesHorizontal.rightBumpStart + LinesHorizontal.rightBumpEnd) / 2, Rotation2d.k180deg), 
					() -> MetersPerSecond.of(2.0), 
					() -> MetersPerSecondPerSecond.of(8.0)
				),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					drive, vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
					kBump.SETTLING_TIME
				),

				// Score
				DriveCommands.alignToHeading(
					drive, 
					() -> DriveCommands.getRotation2d(
						drive, 
						new Pose2d(
						new Translation2d(Hub.topCenterPoint.getMeasureX(), Hub.topCenterPoint.getMeasureY()), 
						Rotation2d.kZero
						)
					).plus(Rotation2d.k180deg)
				),
				Commands.waitTime(Seconds.of(5)),

				// Align to climber prep
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(new Translation2d(Meters.of(Tower.rightUpright.getX()), Meters.of(2.450)), Rotation2d.k180deg), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
					kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
					kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
					kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
				),

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
						Rotation2d.k180deg)
					), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
				)
			)
		);

		autoPaths.add(
			new AutoPath(
				"Test-Path",
				new Pose2d(2,7,Rotation2d.k180deg),
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(2,7,Rotation2d.kZero), 
					() -> MetersPerSecond.of(1), 
					() -> MetersPerSecondPerSecond.of(2)
				),
				AutoPath.followPath("TestPath")
			)
		);

		return autoPaths;
	}
	
}
