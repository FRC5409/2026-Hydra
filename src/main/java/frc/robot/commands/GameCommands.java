// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.serializer.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.Constants.*;


/** Add your docs here. */
public class GameCommands {

    public static Command autoLaunch(Supplier<Distance> distanceSupplier, Drive drive, Launcher launcher, Feeder feeder, Serializer serializer, Intake intake){
        return Commands.sequence(
            Commands.parallel(
                DriveCommands.alignToHeading(
                    drive,
                    () -> DriveCommands.getRotationToHub(drive)
                ),
                // 2) Then launch
                launcher.launchFuel(distanceSupplier, feeder)
            ).until(
                ()-> DriveCommands.isAligned() 
                // TODO: IMPLEMENT WAITFOR LAUNCHER SPIN UP
                // && launcher.isGood
            ),

            serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

            Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

            GameCommands.agitate(intake)

        );
    }

    public static Command manualLaunch(Supplier<Distance> distance, Launcher launcher, Feeder feeder, Serializer serializer, Intake intake ){
        return Commands.sequence(
            
                // 2) Then launch
            launcher.launchFuel(distance, feeder),
            // TODO: UPDATE WITH LAUNCHER WAIT TILL SPIN UP
            // Commands.waitUntil(launcher.isready)
            
            serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

            Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

            GameCommands.agitate(intake)
        );
    }


    /**
     * Drive aligns to face target manually
     * @param distanceSupplier
     * @param launcher
     * @param feeder
     * @param serializer
     * @param intake
     * @return
     */
    public static Command manualPass(Launcher launcher, Feeder feeder, Serializer serializer, Intake intake){
        return Commands.sequence(
            Commands.parallel(
                launcher.runVelocity(() -> GameCommandsConstants.PASSING_RPS),
                launcher.setHoodExtension(() -> GameCommandsConstants.PASSING_HOOD_ANGLE)
                
            ),
            // TODO: IMPLEMENT WAIT FOR LAUNCHER SPINUP
            // Commands.waitUntil(launcher.isReady),

            serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

            Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

            GameCommands.agitate(intake)

        );
    }

    public static Command startIntake(Intake intake, Hopper hopper){
        return Commands.sequence(
            hopper.fullExtend(),
            Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_INTAKE_EXTENSION),
            Commands.parallel(
                intake.extend(),
                intake.setRollerVoltage(IntakeConstants.Roller.INTAKE_VOLTAGE)
            )
        );
    }

    public static Command retract(Intake intake, Hopper hopper){
        return Commands.parallel(
          intake.retract(),
          hopper.fullRetract(),
          intake.stopRoller()  
        );
    }

    public static Command agitate(Intake intake){
        return Commands.parallel(
            intake.setRollerVoltage(IntakeConstants.Roller.AGITATE_VOLTAGE),
            Commands.repeatingSequence(
                intake.move(() -> GameCommandsConstants.RETRACT_POINT),
                Commands.waitUntil(() -> intake.getPosition().isNear(GameCommandsConstants.RETRACT_POINT, Centimeters.of(1.0))),
                intake.move(() -> GameCommandsConstants.EXTEND_POINT),
                Commands.waitUntil(() -> intake.getPosition().isNear(GameCommandsConstants.EXTEND_POINT, Centimeters.of(1.0)))
            )
        );
    }

    public static Command autoClimb(Drive drive, Elevator elevator, Supplier<Pose2d> prepPose, Supplier<Pose2d> climbPose){
        return Commands.sequence(
            Commands.parallel(
                DriveCommands.alignToPoint(
                    drive,
                    prepPose,
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
                    kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
                    kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
                    kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
                ),
                elevator.elevatorGo(ElevatorConstants.kSetpoints.ELEVATOR_UP,0)
            ),

            DriveCommands.alignToPoint(
                drive,
                climbPose,
                () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB,
                () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
            ),

            elevator.elevatorGo(ElevatorConstants.kSetpoints.ELEVATOR_DOWN,0)
        );
    }

    /**
     * Stops launcher, feeder and calls {@link GameCommands#stopSerializing(Serializer, Intake)}
     * @param launcher
     * @param feeder
     * @param serializer
     * @param intake
     * @return
     */
    public static Command stopLaunching(Launcher launcher, Feeder feeder, Serializer serializer, Intake intake){
        return Commands.parallel(
            launcher.stopLauncher(),
            feeder.stopMotor(),
            stopSerializing(serializer, intake)
        );
    }

    public static Command stopSerializing(Serializer serializer, Intake intake){
        return Commands.parallel(
            serializer.stopMotor(),
            intake.stopMotor(),
            intake.stopRoller()
        );
    }

    private static Distance totalIncrement = Meters.of(0.0);
    public static Command retractAndAgitate(Intake intake, Hopper hopper) {
        return Commands.sequence(
                Commands.runOnce(() -> totalIncrement = Meters.of(0.0)),
                startIntake(intake, hopper),
                Commands.repeatingSequence(
                    Commands.runOnce(() -> {
                            totalIncrement = totalIncrement.plus(GameCommandsConstants.incrementBy);
                    }),
                    Commands.parallel(
                        hopper.setSetpoint(() -> {
                            Distance extension = HopperConstants.HOPPER_MAX_EXTENSION;
                            if (extension.minus(totalIncrement).lt(HopperConstants.HOPPER_MIN_EXTENSION))
                                    return HopperConstants.HOPPER_MIN_EXTENSION;

                            return extension.minus(totalIncrement);
                        }),
                        intake.move(() -> {
                            Distance extension = IntakeConstants.Extension.EXTENSION_MAX_DISTANCE;
                            if (extension.minus(totalIncrement).lt(IntakeConstants.Extension.EXTENSION_MIN_DISTANCE))
                                    return IntakeConstants.Extension.EXTENSION_MIN_DISTANCE;

                            return extension.minus(totalIncrement);
                        })
                    ),
                    Commands.waitUntil(() -> hopper.getPosition().isNear(hopper.getSetpoint(), Centimeters.of(0.5))),
                    hopper.setSetpoint(() -> HopperConstants.HOPPER_MAX_EXTENSION.minus(totalIncrement).plus(Inches.of(3)))),
                    Commands.waitUntil(() -> hopper.getPosition().isNear(hopper.getSetpoint(), Centimeters.of(0.01))
                ).until(() -> intake.getPosition().isNear(IntakeConstants.Extension.EXTENSION_MIN_DISTANCE, Centimeters.of(1)))
        );
    }

    // TESTING THIS COMMAND
    public static Command agitateIntakeAndHopper(Intake intake, Hopper hopper) {
        return Commands.parallel(
            intake.setRollerVoltage(IntakeConstants.Roller.AGITATE_VOLTAGE),
            Commands.repeatingSequence(
                    intake.move(() -> GameCommandsConstants.RETRACT_POINT),
                    Commands.waitUntil(() -> intake.getPosition().isNear(GameCommandsConstants.RETRACT_POINT, Centimeters.of(1.0))),
                    Commands.waitUntil(() -> hopper.getPosition().isNear(GameCommandsConstants.RETRACT_POINT, Centimeters.of(1.0))),
                    Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_INTAKE_EXTENSION),
                    intake.move(() -> GameCommandsConstants.EXTEND_POINT),
                    Commands.waitUntil(() -> intake.getPosition().isNear(GameCommandsConstants.EXTEND_POINT, Centimeters.of(1.0))),
                    Commands.waitUntil(() -> hopper.getPosition().isNear(GameCommandsConstants.EXTEND_POINT, Centimeters.of(1.0)))
            ),
            Commands.repeatingSequence(
                    hopper.setSetpoint(() -> GameCommandsConstants.RETRACT_POINT),
                    Commands.waitUntil(() -> intake.getPosition().isNear(GameCommandsConstants.RETRACT_POINT, Centimeters.of(1.0))),
                    Commands.waitUntil(() -> hopper.getPosition().isNear(GameCommandsConstants.RETRACT_POINT, Centimeters.of(1.0))),
                    hopper.setSetpoint(() -> GameCommandsConstants.EXTEND_POINT),
                    Commands.waitUntil(() -> intake.getPosition().isNear(GameCommandsConstants.EXTEND_POINT, Centimeters.of(1.0))),
                    Commands.waitUntil(() -> hopper.getPosition().isNear(GameCommandsConstants.EXTEND_POINT, Centimeters.of(1.0)))
            )
        );
    }

    public static Command agitateThenRetract(Intake intake, Hopper hopper) {
                return Commands.sequence(
                        agitateIntakeAndHopper(intake, hopper).withTimeout(3),
                        Commands.parallel(
                                intake.setExtensionVoltage(-2),
                                hopper.setVoltage(2)
                        ).until(() -> intake.getPosition().isNear(Centimeters.of(0.0), Centimeters.of(2)))

                );
        }

    public static Distance retractStep;
    public static Command retractIntakeInSteps(Intake intake) {
                retractStep = GameCommandsConstants.EXTEND_POINT.minus(Inches.of(3.0));
                return Commands.parallel(
                        intake.setRollerVoltage(IntakeConstants.Roller.AGITATE_VOLTAGE),
                        Commands.repeatingSequence(
                                intake.move(() -> retractStep),
                                Commands.waitUntil(() -> intake.getPosition().isNear(retractStep, Centimeters.of(0.5))),
                                Commands.runOnce(() -> retractStep = retractStep.minus(Centimeters.of(3)))
                        )
                );
    }

    public static Command runIntakeIn(Intake intake){
        return Commands.sequence(
            intake.setExtensionVoltage(2),
            intake.setRollerVoltage(IntakeConstants.Roller.AGITATE_VOLTAGE),
            Commands.waitUntil(() -> intake.getPosition().isNear(Centimeters.of(0), Centimeters.of(1))),
            intake.setExtensionVoltage(0),
            intake.setRollerVoltage(0)
        );
    }



    
}
