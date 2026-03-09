package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Objects;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbingPositions;
import frc.robot.Constants.GameCommandsConstants;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kBump;
import frc.robot.Constants.kField;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.serializer.SerializerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoPath;
import frc.robot.util.FieldConstants.LinesHorizontal;

public class Autos {

    // TODO: CLEAN UP TO USE GAME COMMANDS INSTEAD OF EVERYTHING MANUALLY
	public static ArrayList<AutoPath> getAutoPaths(Drive drive, Vision vision, Launcher launcher, Feeder feeder, Intake intake, Hopper hopper, Elevator elevator, RobotContainer robotContainer){
		ArrayList<AutoPath> autoPaths = new ArrayList<>();

		// LEFT SIDE AUTOS:
		autoPaths.add(
			new AutoPath(
				"LeftBump-Intake-CloseFar-Score-LeftClimb",

				// Angled Starting pose:
				new Pose2d(3.565,5.400, new Rotation2d(Degrees.of(38.572))),

				// Starting Pose: 
				// new Pose2d(3.565,5.400, Rotation2d.k180deg),

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

                Objects.requireNonNull(AutoPath.followPath("Left-Bump-Intake-CloseFar"))
                // TODO: REPLACE WITH GAME COMMANDS: START INTAKING
                    .alongWith(robotContainer.startIntaking()),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(6.200,(LinesHorizontal.leftBumpEnd + LinesHorizontal.leftBumpStart) / 2, Rotation2d.k180deg), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

                // TODO: DETERMINE IF INTAKE ROLLERS NEED TO BE STOPPED
                Commands.print("Intake rollers stop?"),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					drive, 
					vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
					kBump.SETTLING_TIME
				),

				// TODO: Score -> call GameCommands.launchFuel() make sure that includes steps 1 to 5
				// 1. align to hub
                Commands.parallel(
                    DriveCommands.alignToHeading(
                        drive, 
                        () -> DriveCommands.getRotation2d(
                            drive, 
                            kField.BLUE_HUB
                        ).plus(Rotation2d.k180deg)
                    ),
                    // 2. prepare launcher
                    launcher.launchFuel(() -> DriveCommands.distToHub(drive), feeder)
                ),

                // 3. wait to make sure aligned and spun up
                Commands.parallel(
                    Commands.waitUntil(DriveCommands::isAligned),
                    // TODO: REPLACE WITH WAIT FOR LAUNCHER IS SPUN UP
                    Commands.waitTime(Seconds.of(2))
                ),

				// 4. feed fuel into launcher
				robotContainer.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

				robotContainer.agitateIntake(IntakeConstants.Roller.AGITATE_VOLTAGE),

				Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),

				// 5. after 5s, stop all launcher systems
                Commands.parallel(
                    launcher.stopLauncher(),
                    feeder.stopMotor(),
                    robotContainer.sys_serializer.stopMotor(),
                    intake.stopMotor(),
                    intake.stopRoller()
                )

				// TODO: Align to climber prep -> Call GameCommands.climb()
				// DriveCommands.alignToPoint(
				// 	drive,
				// 	ClimbingPositions.LEFT_PREP::getPose,
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
				// 	kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
				// 	kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
				// 	kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
				// ),

				// // Align to climb
				// DriveCommands.alignToPoint(
				// 	drive,
				// 	ClimbingPositions.LEFT::getPose,
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
				// )
			)
		);

		autoPaths.add(
			new AutoPath(
				"LeftBump-Intake-FarClose-Score-LeftClimb",
				// Angled Start:
				new Pose2d(3.565,5.801, new Rotation2d(Degrees.of(-43.361))),            
				// Starting Pose: 
				// new Pose2d(3.565,5.801, Rotation2d.k180deg),

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
                // TODO: REPLACE WITH GAME COMMANDS INTAKE 
                Objects.requireNonNull(AutoPath.followPath("Left-Bump-Intake-FarClose"))
                    .alongWith(robotContainer.startIntaking()),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(6.200,(LinesHorizontal.leftBumpEnd + LinesHorizontal.leftBumpStart) / 2, Rotation2d.k180deg), 
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

                // TODO: DETERMINE IF WE NEED TO STOP INTAKE ROLLERS
                Commands.print("Stop intake rollers?"),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					drive, 
					vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
					kBump.SETTLING_TIME
				),

				// TODO: Score -> call GameCommands.launchFuel() make sure that includes steps 1 to 5

				// 1. align to hub
                Commands.parallel(
                    DriveCommands.alignToHeading(
                        drive, 
                        () -> DriveCommands.getRotation2d(
                            drive, 
                            kField.BLUE_HUB
                        ).plus(Rotation2d.k180deg)
                    ),
                    // 2. prepare launcher
                    launcher.launchFuel(() -> DriveCommands.distToHub(drive), feeder)
                ),

                // 3. wait to make sure aligned and spun up
                Commands.parallel(
                    Commands.waitUntil(DriveCommands::isAligned),
                    // TODO: REPLACE WITH WAIT FOR LAUNCHER IS SPUN UP
                    Commands.waitTime(Seconds.of(2))
                ),

				// 4. feed fuel into launcher
				robotContainer.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

				robotContainer.agitateIntake(IntakeConstants.Roller.AGITATE_VOLTAGE),

				Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),

				// 5. after 5s, stop all launcher systems
                Commands.parallel(
                    launcher.stopLauncher(),
                    feeder.stopMotor(),
                    robotContainer.sys_serializer.stopMotor(),
                    intake.stopMotor(),
                    intake.stopRoller()
                )

				// TODO: Align to climber prep -> call GameCommands.climb() 
//				DriveCommands.alignToPoint(
//					drive,
//					ClimbingPositions.LEFT_PREP::getPose,
//					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
//					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
//					kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
//					kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
//					kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
//				),
//
//				// Align to climb
//				DriveCommands.alignToPoint(
//					drive,
//					ClimbingPositions.LEFT::getPose,
//					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB,
//					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
//				)
			)
		);

		// RIGHT SIDE AUTOS:
		autoPaths.add(
			new AutoPath(
				"RightBump-Intake-CloseFar-Score-RightClimb",

				// Starting Pose: 
				// new Pose2d(3.565,2.750, Rotation2d.kZero),

				// Angled Starting Pose
				new Pose2d(3.565,2.750, new Rotation2d(Degrees.of(-41.689))),
				
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
				// Follow path from center of neutral zone to right of field
				Objects.requireNonNull(AutoPath.followPath("Right-Bump-Intake-CloseFar"))
                // TODO: REPLACE WITH GAME COMMANDS START INTAKING COMMAND
                .alongWith(robotContainer.startIntaking()),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(6.200,(LinesHorizontal.rightBumpStart + LinesHorizontal.rightBumpEnd) / 2, Rotation2d.k180deg), 
					() -> MetersPerSecond.of(2.0), 
					() -> MetersPerSecondPerSecond.of(8.0)
				),

                // TODO: DETERMINE IF NEEDED TO STOP INTAKING
                Commands.print("Stop intake rollers?"),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					drive, vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
					kBump.SETTLING_TIME
				),

				// TODO: Score -> call GameCommands.launchFuel() make sure that includes steps 1 to 5

				// 1. align to hub
                Commands.parallel(
                    DriveCommands.alignToHeading(
                        drive, 
                        () -> DriveCommands.getRotation2d(
                            drive, 
                            kField.BLUE_HUB
                        ).plus(Rotation2d.k180deg)
                    ),
                    // 2. prepare launcher
                    launcher.launchFuel(() -> DriveCommands.distToHub(drive), feeder)
                ),

                // 3. wait to make sure aligned and spun up
                Commands.parallel(
                    Commands.waitUntil(DriveCommands::isAligned),
                    // TODO: REPLACE WITH WAIT FOR LAUNCHER IS SPUN UP
                    Commands.waitTime(Seconds.of(2))
                ),

				// 4. feed fuel into launcher
				robotContainer.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

				robotContainer.agitateIntake(IntakeConstants.Roller.AGITATE_VOLTAGE),

				Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),

				// 5. after 5s, stop all launcher systems
                Commands.parallel(
                    launcher.stopLauncher(),
                    feeder.stopMotor(),
                    robotContainer.sys_serializer.stopMotor(),
                    intake.stopMotor(),
                    intake.stopRoller()
                )

				// TODO: Align to climber prep -> Should be calling GameCommands.climb()
				// DriveCommands.alignToPoint(
				// 	drive,
				// 		ClimbingPositions.RIGHT_PREP::getPose,
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
				// 	kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
				// 	kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
				// 	kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
				// ),

				// // Align to climb
				// DriveCommands.alignToPoint(
				// 	drive,
				// 		ClimbingPositions.RIGHT::getPose,
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
				// )
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
				// new Pose2d(3.565,2.750, Rotation2d.k180deg),

				// Angled Starting Pose
				new Pose2d(3.565,2.282, new Rotation2d(Degrees.of(53.181))),
				
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
				// Follow path from right of field to center of neutral zone 
				Objects.requireNonNull(AutoPath.followPath("Right-Bump-Intake-FarClose"))
                // TODO: REPLACE WITH GAME COMMANDS START INTAKING COMMAND
                .alongWith(robotContainer.startIntaking()),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					drive, 
					() -> new Pose2d(6.200,(LinesHorizontal.rightBumpStart + LinesHorizontal.rightBumpEnd) / 2, Rotation2d.k180deg), 
					() -> MetersPerSecond.of(2.0), 
					() -> MetersPerSecondPerSecond.of(8.0)
				),

                // TODO: DETERMINE IF THERE IS A NEED TO STOP INTAKE ROLLERS
                Commands.print("Stop intake rollers?"),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					drive, vision,
					drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
					kBump.SETTLING_TIME
				),

				// TODO: Score -> call GameCommands.launchFuel() make sure that includes steps 1 to 5

				// 1. align to hub
                Commands.parallel(
                    DriveCommands.alignToHeading(
                        drive, 
                        () -> DriveCommands.getRotation2d(
                            drive, 
                            kField.BLUE_HUB
                        ).plus(Rotation2d.k180deg)
                    ),
                    // 2. prepare launcher
                    launcher.launchFuel(() -> DriveCommands.distToHub(drive), feeder)
                ),

                // 3. wait to make sure aligned and spun up
                Commands.parallel(
                    Commands.waitUntil(DriveCommands::isAligned),
                    // TODO: REPLACE WITH WAIT FOR LAUNCHER IS SPUN UP
                    Commands.waitTime(Seconds.of(2))
                ),

				// 4. feed fuel into launcher
				robotContainer.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

				robotContainer.agitateIntake(IntakeConstants.Roller.AGITATE_VOLTAGE),

				Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),

				// 5. after 5s, stop all launcher systems
                Commands.parallel(
                    launcher.stopLauncher(),
                    feeder.stopMotor(),
                    robotContainer.sys_serializer.stopMotor(),
                    intake.stopMotor(),
                    intake.stopRoller()
                )

				//TODO:  Align to climber prep -> Should be calling GameCommands.climb()
//				DriveCommands.alignToPoint(
//					drive,
//					ClimbingPositions.RIGHT_PREP::getPose,
//					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
//					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
//					kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
//					kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
//					kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
//				)

				// Align to climb
//				DriveCommands.alignToPoint(
//					drive,
//					ClimbingPositions.RIGHT::getPose,
//					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB,
//					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
//				)
			)
		); 

        if (Constants.IS_TUNING)
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
        

        autoPaths.add(
            new AutoPath(
                "Leave-Shoot", 
                new Pose2d(3.565,4.011,Rotation2d.k180deg), 
                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(3.127,4.011,Rotation2d.k180deg), 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION),

                // TODO: Score -> call GameCommands.launchFuel() make sure that includes steps 1 to 5

				// 1. align to hub
                Commands.parallel(
                    DriveCommands.alignToHeading(
                        drive, 
                        () -> DriveCommands.getRotation2d(
                            drive, 
                            kField.BLUE_HUB
                        ).plus(Rotation2d.k180deg)
                    ),
                    // 2. prepare launcher
                    launcher.launchFuel(() -> DriveCommands.distToHub(drive), feeder)
                ),

                // 3. wait to make sure aligned and spun up
                Commands.parallel(
                    Commands.waitUntil(DriveCommands::isAligned),
                    // TODO: REPLACE WITH WAIT FOR LAUNCHER IS SPUN UP
                    Commands.waitTime(Seconds.of(2))
                ),

				// 4. feed fuel into launcher
				robotContainer.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                // Commands.waitTime(GameCommands.WAIT_TIME_BEFORE_AGITATE),

				robotContainer.agitateIntake(IntakeConstants.Roller.AGITATE_VOLTAGE),

				Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),

				// 5. after 5s, stop all launcher systems
                Commands.parallel(
                    launcher.stopLauncher(),
                    feeder.stopMotor(),
                    robotContainer.sys_serializer.stopMotor(),
                    intake.stopMotor(),
                    intake.stopRoller()
                )
            )
        );

        autoPaths.add(
            new AutoPath(
                "Depot-Shoot-LeftClimb", 
                new Pose2d(3.565,5.958,Rotation2d.k180deg),

                // Go from starting point to depot
                Objects.requireNonNull(AutoPath.followPath("Start-Depot"))
                // TODO: USE GAME COMMANDS START INTAKING COMMAND
                .alongWith(robotContainer.startIntaking()),

                // Align to scoring point
                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(1.390, 4.887, Rotation2d.k180deg), 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
                ),

                // TODO: Score -> call GameCommands.launchFuel() make sure that includes steps 1 to 5

				// 1. align to hub
                Commands.parallel(
                    DriveCommands.alignToHeading(
                        drive, 
                        () -> DriveCommands.getRotation2d(
                            drive, 
                            kField.BLUE_HUB
                        ).plus(Rotation2d.k180deg)
                    ),
                    // 2. prepare launcher
                    launcher.launchFuel(() -> DriveCommands.distToHub(drive), feeder)
                ),

                // 3. wait to make sure aligned and spun up
                Commands.parallel(
                    Commands.waitUntil(DriveCommands::isAligned),
                    // TODO: REPLACE WITH WAIT FOR LAUNCHER IS SPUN UP
                    Commands.waitTime(Seconds.of(2))
                ),

				// 4. feed fuel into launcher
				robotContainer.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

				robotContainer.agitateIntake(IntakeConstants.Roller.AGITATE_VOLTAGE),

				Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),

				// 5. after 5s, stop all launcher systems
                Commands.parallel(
                    launcher.stopLauncher(),
                    feeder.stopMotor(),
                    robotContainer.sys_serializer.stopMotor(),
                    intake.stopMotor(),
                    intake.stopRoller()
                )

                // TODO: Align to climber prep -> call GameCommands.climb() 
				// DriveCommands.alignToPoint(
				// 	drive,
				// 	ClimbingPositions.LEFT_PREP::getPose,
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
				// 	kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
				// 	kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
				// 	kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
				// ),

				// // Align to climb
				// DriveCommands.alignToPoint(
				// 	drive,
				// 	ClimbingPositions.LEFT::getPose,
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
				// )
            )
        );

        autoPaths.add(
            new AutoPath(
                "Outpost-Shoot-RightClimb", 
                // Start at edge of bump
                new Pose2d(3.565,2.076,Rotation2d.k180deg), 
                // Start at trench
                // new Pose2d(3.565,0.719,Rotation2d.k180deg),
                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(0.473, 0.670, Rotation2d.k180deg), 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION  
                )
                // TODO: REPLACE WITH GAME COMMANDS: START INTAKING
                .alongWith(robotContainer.startIntaking()),
                // Time to wait for outpost dump
                Commands.waitSeconds(2),

                DriveCommands.alignToPoint(
                    drive, 
                    () -> new Pose2d(0.902, 0.670, Rotation2d.k180deg), 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
                ),
                
                // TODO: Score -> call GameCommands.launchFuel() make sure that includes steps 1 to 5

				// 1. align to hub
                Commands.parallel(
                    DriveCommands.alignToHeading(
                        drive, 
                        () -> DriveCommands.getRotation2d(
                            drive, 
                            kField.BLUE_HUB
                        ).plus(Rotation2d.k180deg)
                    ),
                    // 2. prepare launcher
                    launcher.launchFuel(() -> DriveCommands.distToHub(drive), feeder)
                ),

                // 3. wait to make sure aligned and spun up
                Commands.parallel(
                    Commands.waitUntil(DriveCommands::isAligned),
                    // TODO: REPLACE WITH WAIT FOR LAUNCHER IS SPUN UP
                    Commands.waitTime(Seconds.of(2))
                ),

				// 4. feed fuel into launcher
				robotContainer.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

				robotContainer.agitateIntake(IntakeConstants.Roller.AGITATE_VOLTAGE),

				Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),

				// 5. after 5s, stop all launcher systems
                Commands.parallel(
                    launcher.stopLauncher(),
                    feeder.stopMotor(),
                    robotContainer.sys_serializer.stopMotor(),
                    intake.stopMotor(),
                    intake.stopRoller()
                )

                // TODO: Align to climber prep -> call GameCommands.climb() 
				// DriveCommands.alignToPoint(
				// 	drive,
				// 	ClimbingPositions.RIGHT_PREP::getPose,
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
				// 	kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
				// 	kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
				// 	kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
				// ),

				// // Align to climb
				// DriveCommands.alignToPoint(
				// 	drive,
				// 	ClimbingPositions.RIGHT::getPose,
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
				// 	() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
				// )
            )
        );

		return autoPaths;
	}
	
}
