// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.launcher.interpolator.LaunchStrategy;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerConstants;
import frc.robot.subsystems.serializer.SerializerIO;
import frc.robot.subsystems.serializer.SerializerIOSim;
import frc.robot.subsystems.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.AutoPath;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Constants.*;

import java.util.ArrayList;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    // TODO: make these protected later when gamecommands are here
    public final Drive      sys_drive;
    public final Vision     sys_vision;
    public final Intake     sys_intake;
    public final Serializer sys_serializer;
    public final Feeder     sys_feeder;
    public final Hopper     sys_hopper;
   
    public final Launcher   sys_launcher;
    public final   Elevator   sys_elevator;

    public static SwerveDriveSimulation simConfig;

    private PassingPositions selectedPassingPosition = PassingPositions.MIDDLE;
    private ClimbingPositions selectedClimbingPosition = ClimbingPositions.LEFT;
    private ClimbingPositions selectedClimbingPrepPosition = ClimbingPositions.LEFT_PREP;

    private Distance manualLaunchDistance = Meters.of(2);

    private boolean shouldPass = false;

    // Controllers
    private final CommandXboxController primaryController   = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final CommandXboxController tertiaryController = new CommandXboxController(2);

    private final Alert primaryDisconnectedAlert   = new Alert(
            "Primary Controller Disconnected!",
            AlertType.kError
    );
    private final Alert secondaryDisconnectedAlert = new Alert(
            "Secondary Controller Disconnected!",
            AlertType.kError
    );

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            // Real robot, instantiate hardware IO implementations
            case REAL -> {
                sys_hopper = new Hopper(
                        new HopperIOTalonFX(DeviceID.HOPPER_MOTOR_ID));
                sys_intake = new Intake(new IntakeIOTalonFX(DeviceID.INTAKE_ROLLER_MOTOR, DeviceID.INTAKE_EXTENSION_MOTOR));

                sys_serializer = new Serializer(
                        new SerializerIOTalonFX(DeviceID.SERIALIZER_MOTOR, DeviceID.FEEDER_MOTOR_BOTTOM));
                sys_feeder = new Feeder(new FeederIOTalonFX(DeviceID.FEEDER_MOTOR_TOP));
                sys_vision = new Vision(new VisionIOLimelight());
                sys_elevator = new Elevator(new ElevatorIOTalonFX(DeviceID.CLIMBER_MOTOR));

                sys_drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        sys_vision
                );

                sys_launcher = new Launcher(new LauncherIOTalonFX(
                        DeviceID.LAUNCHER_CANCODER,
                        DeviceID.LAUNCHER_MOTOR_1,
                        DeviceID.LAUNCHER_MOTOR_2,
                        DeviceID.LAUNCHER_ULTRASONIC_CHANNEL,
                        DeviceID.LAUNCHER_HOOD_SERVO_1,
                        DeviceID.LAUNCHER_HOOD_SERVO_2));
            }
            // Sim robot, instantiate physics sim IO implementations
            case SIM -> {
                final DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig
                        .Default()
                        .withGyro(COTS.ofPigeon2())
                        .withRobotMass(DriveConstants.ROBOT_FULL_MASS)
                        .withTrackLengthTrackWidth(Meters.of(0.578), Meters.of(0.578))
                        .withBumperSize(Meters.of(0.881), Meters.of(0.881))
                        .withSwerveModule(
                                COTS.ofMark4i(
                                        DCMotor.getKrakenX60(1),
                                        DCMotor.getKrakenX44(1),
                                        DriveConstants.WHEEL_COF,
                                        1));

                simConfig = new SwerveDriveSimulation(driveConfig, new Pose2d(3, 3, Rotation2d.kZero));

                SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
                SimulatedArena.getInstance().addDriveTrainSimulation(simConfig);
                SimulatedArena.getInstance().resetFieldForAuto();

                sys_vision = new Vision(new VisionIOSim(simConfig));
                sys_drive = new Drive(
                        new GyroIOSim(simConfig.getGyroSimulation()),
                        new ModuleIOSim(simConfig.getModules()[0]),
                        new ModuleIOSim(simConfig.getModules()[1]),
                        new ModuleIOSim(simConfig.getModules()[2]),
                        new ModuleIOSim(simConfig.getModules()[3]),
                        sys_vision);
                sys_elevator = new Elevator(new ElevatorIOSim());
                sys_intake = new Intake(new IntakeIOSim());
                sys_serializer = new Serializer(new SerializerIOSim());
                sys_feeder = new Feeder(new FeederIOSim());
                sys_hopper = new Hopper(new HopperIOSim());

                sys_launcher = new Launcher(new LauncherIOSim());
            }
            // Replayed robot, disable IO implementations
            default -> {
                sys_vision = new Vision(new VisionIO() {});
                sys_drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        sys_vision);
                sys_hopper = new Hopper(new HopperIO() {});
                sys_intake = new Intake(new IntakeIO() {});
                sys_serializer = new Serializer(new SerializerIO() {});
                sys_elevator = new Elevator(new ElevatorIO() {});
                sys_feeder = new Feeder(new FeederIO() {});
                sys_launcher = new Launcher(new LauncherIO() {});
            }
        }

        // Set up auto routines
        autoChooser = buildAutoChooser();
        buildLaunchStrategyChooser();

        // Configure the button bindings
        configureButtonBindings();

        SmartDashboard.putData("Reset", Commands.runOnce(this::resetPose).ignoringDisable(true));

        new Trigger(() -> !primaryController.isConnected()).onChange(
                Commands.runOnce(() -> primaryDisconnectedAlert.set(!primaryController.isConnected()))
                        .ignoringDisable(true)
        );

        new Trigger(() -> !secondaryController.isConnected()).onChange(
                Commands.runOnce(() -> secondaryDisconnectedAlert.set(!secondaryController.isConnected()))
                        .ignoringDisable(true)
        );

        // When DS connects check joystick connections
        new Trigger(DriverStation::isDSAttached).onTrue(
                Commands.waitSeconds(1.0)
                        .andThen(
                                Commands.runOnce(() -> {
                                            primaryDisconnectedAlert.set(!primaryController.isConnected());
                                            secondaryDisconnectedAlert.set(!secondaryController.isConnected());

                                            resetPose();
                                        })
                                        .ignoringDisable(true)
                        )
        );
    }

    private void resetPose() {
        if (autoChooser.get() instanceof AutoPath path) {
            sys_drive.setPose(path.getStartingPose());
            if (Constants.CURRENT_MODE == Mode.SIM)
                simConfig.setSimulationWorldPose(path.getStartingPose());
        }
        if (autoChooser.get() instanceof PathPlannerAuto auto){
            sys_drive.setPose(auto.getStartingPose());
            if (Constants.CURRENT_MODE == Mode.SIM)
                simConfig.setSimulationWorldPose(auto.getStartingPose());
        }
    }

    /**
     * builds the dashboard command chooser ({@link LoggedDashboardChooser}) for picking launch strategies.
     *
     * @return the logged dashboard chooser
     */
    public LoggedDashboardChooser<Command> buildLaunchStrategyChooser() {
        LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>("Launch Strategy");

        for (LaunchStrategy strategy: LaunchStrategy.getLaunchStrategies())
            chooser.addOption(strategy.getName(), Commands.runOnce(() -> sys_launcher.setStrategy(strategy)));

        chooser.onChange(CommandScheduler.getInstance()::schedule);

        return chooser;
    }

    /**
     * Updates sim positions of algae, coral and robot poses
     */
    public void updateSim() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("Simulation/RobotPose", simConfig.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    private LoggedDashboardChooser<Command> buildAutoChooser() {
        LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>("Auto Choices");
        chooser.addDefaultOption("None", Commands.none());
        ArrayList<AutoPath> autoPaths = Autos.getAutoPaths(
            sys_drive, 
            sys_vision,
            sys_launcher,
            sys_feeder,
            sys_intake,
            sys_hopper,
            sys_elevator,
            this
        );

        autoPaths.forEach(autoPath -> chooser.addOption(autoPath.getName(), autoPath));

        for (String auto: AutoBuilder.getAllAutoNames()){
            chooser.addOption(auto, new PathPlannerAuto(auto));
        }

        if (Constants.IS_TUNING) {
            chooser.addOption(
                    "Drive Wheel Radius Characterization",
                    DriveCommands.wheelRadiusCharacterization(sys_drive));
            chooser.addOption(
                    "Drive Simple FF Characterization",
                    DriveCommands.feedforwardCharacterization(sys_drive));
            chooser.addOption(
                    "Drive SysId (Quasistatic Forward)",
                    sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            chooser.addOption(
                    "Drive SysId (Quasistatic Reverse)",
                    sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            chooser.addOption(
                    "Drive SysId (Dynamic Forward)",
                    sys_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            chooser.addOption(
                    "Drive SysId (Dynamic Reverse)",
                    sys_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        chooser.onChange(_cmd -> resetPose());
        return chooser;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        SmartDashboard.putNumber("Launcher Speed Offset [rps]", Launcher.getSpeedOffset().in(RotationsPerSecond));
        
        SmartDashboard.putData("Update Offset Now", Commands.runOnce(() -> Launcher.setSpeedOffset(
                RotationsPerSecond.of(SmartDashboard.getNumber("Launcher Speed Offset [rps]", 0.0)))));

        sys_drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        sys_drive,
                        () -> -primaryController.getLeftY(),
                        () -> -primaryController.getLeftX(),
                        () -> -(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis())
                )
        );

        primaryController.start()
                         .and(primaryController.back())
                         .onTrue(
                                 Commands.runOnce(() -> sys_drive.setPose(new Pose2d(0, 0, Rotation2d.k180deg)))
                                         .ignoringDisable(true)
                         );

        // TODO: GAME COMMANDS: LAUNCH FUEL or PASS FUEL
        new Trigger(() -> kField.NEUTRAL_ZONE.contains(sys_drive.getPose().getTranslation()))
            .onTrue(Commands.runOnce(() -> shouldPass = true))
            .onFalse(Commands.runOnce(() -> shouldPass = false));

        primaryController.rightBumper()
                        .whileTrue(
                            !shouldPass 
                            // TODO: IF LAUNCHING, USE LAUNCHING GAME COMMAND
                            ? Commands.parallel(
                                DriveCommands.alignToHeading(sys_drive, () -> DriveCommands.getRotation2d(sys_drive, kField.BLUE_HUB).plus(Rotation2d.k180deg)),
                                sys_launcher.launchFuel(() -> DriveCommands.distToHub(sys_drive), sys_feeder),
                                Commands.sequence(
                                    Commands.waitUntil(DriveCommands::isAligned),
                                    // TODO: WAIT UNTIL LAUNCHER IS SPUN UP TO RIGHT SPEED
                                    // .alongWith(Commands.waitUntil(sys_launcher.)),
                                    sys_serializer.setVoltage(10),
                                    Commands.waitTime(Seconds.of(3)),
                                    agitateIntake(IntakeConstants.Roller.AGITATE_VOLTAGE)
                                    
                                )
                            )
                            // TODO: IF PASSING USE PASSING GAME COMMAND
                            : Commands.parallel(
                                // TODO: GET PASSING DISTANCE
                                Commands.sequence(
                                    sys_launcher.launchFuel(()-> Meters.of(2.0), sys_feeder),
                                    // TODO: WAIT UNTIL LAUNCHER IS SPUN UP
                                    // Commands.waitUntil(null),
                                    sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),
                                    Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE)
                                )
                            )
                        )
                        .onFalse(Commands.parallel(
                            sys_launcher.stopLauncher(),
                            sys_feeder.setVoltage(0),
                            sys_serializer.stopMotor(),
                            sys_intake.stopMotor(),
                            sys_intake.stopRoller()
                        ));

        //TODO:  GAME COMMANDS: START INTAKING
        primaryController.leftBumper()
                        .onTrue(
                            startIntaking()
                        );

        // TODO: GAME COMMANDS: CLIMB (Auto Align or No Auto Align)
        primaryController.y()
                        .whileTrue(
                            Commands.print("CLIMB GAME COMMAND HERE")
                        );

        // TODO: GAME COMMANDS: RETRACT INTAKE AND HOPPER INSTANTLY, both at the same time
        primaryController.x()
                        .onTrue(
                            Commands.parallel(
                                sys_intake.retract(),
                                sys_hopper.fullRetract()
                            )
                        );

        primaryController.a()
                         .onTrue(Commands.runOnce(() -> DriveCommands.setSpeed(kBump.BUMP_SPEED_MODIFIER)))
                         .onFalse(Commands.runOnce(() -> DriveCommands.setSpeed(1.0)));

        // TODO: GAME COMMANDS: MANUAL LAUNCH 
        primaryController.povDown()
                        .whileTrue(
                            Commands.sequence(
                                sys_launcher.launchFuel(() -> manualLaunchDistance, sys_feeder),
                                // WAIT FOR LAUNCHER SPIN UP
                                // Commands.waitUntil(() -> ),
                                sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),
                                Commands.waitTime(Milliseconds.of(500)),
                                agitateIntake(IntakeConstants.Roller.AGITATE_VOLTAGE)
                            )
                        )
                        .onFalse(
                            Commands.parallel(
                                sys_launcher.stopLauncher(),
                                sys_feeder.stopMotor(),
                                sys_serializer.stopMotor(),
                                sys_intake.stopMotor(),
                                sys_intake.stopRoller()
                            )
                        );

        // TODO: GAME COMMANDS: RETRACT INTAKE AND HOPPER INSTANTLY, both at the same time
        secondaryController.x()
                        .onTrue(
                            Commands.parallel(
                                sys_intake.retract(),
                                sys_hopper.fullRetract()
                            )
                        );

        secondaryController.povUp()
                        .onTrue(Launcher.incrementSpeedOffset(RotationsPerSecond.of(1)));
                        
        secondaryController.povDown()
                        .onTrue(Launcher.incrementSpeedOffset(RotationsPerSecond.of(-1)));

        secondaryController.povLeft()
                        .onTrue(prepClimberPositionCommand(ClimbingPositions.LEFT));

        secondaryController.povRight()
                        .onTrue(prepClimberPositionCommand(ClimbingPositions.RIGHT));

        // TODO: GET MANUAL LAUNCH DISTANCE THAT WE WANT TO USE
        new Trigger(() -> secondaryController.getLeftX() > 0.5)
                    .onTrue(prepManualLaunchDistance(Meters.of(2.0)));

        // TODO: GET MANUAL LAUNCH DISTANCE THAT WE WANT TO USE
        new Trigger(() -> secondaryController.getLeftX() < 0.5)
                    .onTrue(prepManualLaunchDistance(Meters.of(4.0)));

        // PITS TEST CONTROLLER BUTTONS:
        // TODO: BOOLEAN IF IN PITS THEN USE THESE BUTTONS OTHERWISE DON't (TO STOP DRIVERSTATION ERRORS)

        // tertiaryController.x()
        //     .onTrue(sys_elevator.goTillSpike(-1));

        // tertiaryController.povUp()
        // .onTrue(sys_elevator.startManualMove(1.0))
        // .onFalse(sys_elevator.startManualMove(0));

        // tertiaryController.povDown()
        // .onTrue(sys_elevator.startManualMove(-1.0))
        // .onFalse(sys_elevator.startManualMove(0));

        // BUTTONS TO TEST CODE
        // TODO: remove some of these when merging to main, or maybe make a DebugCommand interface
        
        SmartDashboard.putNumber("LAUNCHER DISTANCE [m]", 5);
        SmartDashboard.putData(
                "LAUNCH FUEL (DST)", sys_launcher.launchFuel(
                        () -> Meters.of(SmartDashboard.getNumber("LAUNCHER DISTANCE [m]", 0)), sys_feeder));

        SmartDashboard.putData("STOP LAUNCHER", sys_launcher.stopLauncher());

        SmartDashboard.putNumber("Hood Angle [mm]", 0);
        SmartDashboard.putData(
                "Set Hood Angle", sys_launcher.setHoodExtension(() ->
                        Millimeter.of(
                                                                            SmartDashboard.getNumber(
                                                                                    "Hood Angle [mm]",
                                                                                    0))));
        final double[] launchSpeed = {50};
        Logger.recordOutput("Launcher/SpeedSetpointManual", launchSpeed[0]);

        tertiaryController.povUp()
            .onTrue(Commands.runOnce(() -> {
                launchSpeed[0] += 0.5;
                Logger.recordOutput("Launcher/SpeedSetpointManual", launchSpeed[0]);
            }));

        tertiaryController.povDown()
                .onTrue(Commands.runOnce(() -> {
                    launchSpeed[0] -= 0.5;
                    Logger.recordOutput("Launcher/SpeedSetpointManual", launchSpeed[0]);
                }));

        // LAUNCHER TESTING

        // tertiaryController.a()
        //                  .onTrue(sys_serializer.setVoltage(8))
        //                  .onTrue(sys_intake.setRollerVoltage(10))
        //                  .onFalse(sys_serializer.setVoltage(0))
        //                  .onFalse(sys_intake.setRollerVoltage(0));

        // tertiaryController.rightTrigger()
        //                    .onTrue(sys_launcher.launchFuel(() -> DriveCommands.distToHub(sys_drive), sys_feeder));

        // tertiaryController.x()
        //         .onTrue(sys_launcher.runVelocity(() -> RotationsPerSecond.of(launchSpeed[0])))
        //         .onTrue(sys_feeder.runRPS(() -> RotationsPerSecond.of(launchSpeed[0])));

        // tertiaryController.b()
        //          .onTrue(sys_feeder.stopMotor())
        //          .onTrue(sys_launcher.stopLauncher());

        // tertiaryController.y()
        //         .onTrue(sys_launcher.setHoodExtension(() -> Millimeter.of(SmartDashboard.getNumber("Hood Angle [mm]", 0))));

        // BUMP TESTING
        tertiaryController.a()
                    .onTrue(
                        Commands.runOnce(() -> DriveCommands.setSpeed(kBump.BUMP_SPEED_MODIFIER))
                    )
                    .onFalse(Commands.runOnce(() -> DriveCommands.setSpeed(1)));

        tertiaryController.x()
                .whileTrue(
                        DriveCommands.crossBump(
                                sys_drive,
                                sys_vision,
                                sys_drive::getRotation,
                                () -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED),
                                Seconds.of(1)
                        )
                );

        tertiaryController.b()
                .whileTrue(
                        DriveCommands.crossBump(
                                sys_drive,
                                sys_vision,
                                sys_drive::getRotation,
                                () -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)),
                                Seconds.of(1)
                        )
                );

        tertiaryController.y()
                .whileTrue(
                    DriveCommands.crossBumpDeadline(
                        sys_drive, 
                        () -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED)
                    )
                );
        
        SmartDashboard.putData("Hopper/Coast", sys_hopper.coastMode().ignoringDisable(true)); //TODO remove when main
        SmartDashboard.putData("Hopper/Brake", sys_hopper.brakeMode().ignoringDisable(true)); //TODO remove when main

        SmartDashboard.putData("Intake/Coast", sys_intake.coastMode().ignoringDisable(true)); // TODO: REMOVE WHEN MAIN
        SmartDashboard.putData("Intake/Brake", sys_intake.brakemode().ignoringDisable(true)); // TODO: REMOVE WHEN MAIN
      }

    private Command prepClimberPositionCommand(ClimbingPositions climbingPosition) {
        return Commands.runOnce(
                () -> {
                    if (climbingPosition == ClimbingPositions.LEFT)
                        selectedClimbingPrepPosition = ClimbingPositions.LEFT_PREP;
                    else
                        selectedClimbingPrepPosition = ClimbingPositions.RIGHT_PREP;

                    Logger.recordOutput("ClimbingPosition", climbingPosition);

                    selectedClimbingPosition = climbingPosition;

                    Logger.recordOutput("ClimbingSelectedPose", selectedClimbingPosition.pose);
                }
        );
    }

    private Command prepManualLaunchDistance(Distance distance){
        return Commands.runOnce(
            () -> {
                manualLaunchDistance = distance;
                Logger.recordOutput("ManualLaunchDistance", manualLaunchDistance);
            }
        );
    }

    // TODO: DETERMINE IF NEEDED, IF NOT DELETE
    /** 
     * Command to extend both intake and hopper subsystems, with crash avoidance
     * @author Jaden Rajan, team 5409
     * @author John Chen, team 5409
     */
    private Command extendIntakeAndHopper() {
        return Commands.repeatingSequence(
                Commands.either(
                sys_intake.stopMotor(),
                sys_intake.extend(), 
                () -> sys_hopper.getPositionIntakeZero().minus(sys_intake.getPosition()).lt(IntakeConstants.Extension.KILLSWITCH_TOLERANCE) 
                        && !(sys_hopper.getPosition().isNear(HopperConstants.HOPPER_MAX_EXTENSION, HopperConstants.AGITATE_TOLERANCE))
                ).alongWith(sys_hopper.fullExtend())
        ).until(() -> {return sys_intake.getPosition().isNear(IntakeConstants.Extension.EXTENSION_DISTANCE, HopperConstants.AGITATE_TOLERANCE);})
                .andThen(sys_hopper.fullExtend());
    }

    // TODO: MOVE TO GAME COMMANDS
    /** 
     * Extends Hopper, waits {@link GameCommands#WAIT_TIME_BEFORE_INTAKE_EXTENSION}, 
     * then extends intake and starts intake roller at {@link IntakeConstants.Roller#INTAKE_VOLTAGE} volts
     * 
     */ 
    public Command startIntaking(){
        return Commands.sequence(
                        sys_hopper.fullExtend(),
                        Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_INTAKE_EXTENSION),
                        Commands.parallel(
                            sys_intake.extend(),
                            sys_intake.setRollerVoltage(IntakeConstants.Roller.INTAKE_VOLTAGE)
                        )
                    );
    }

    // TODO: DETERMINE IF NEEDED, IF NOT DELETE
    /** 
     * Command to retract both intake and hopper subsystems, with crash avoidance
     * @author Jaden Rajan, team 5409
     * @author John Chen, team 5409
     */
    private Command retractIntakeAndHopper() {
        return Commands.repeatingSequence(
                Commands.either(
                        sys_hopper.stopMotor(), 
                        sys_hopper.setSetpoint(() -> HopperConstants.HOPPER_MIN_EXTENSION), 
                        () -> sys_hopper.getPositionIntakeZero().minus(sys_intake.getPosition()).lt(IntakeConstants.Extension.KILLSWITCH_TOLERANCE) 
                        && !(sys_intake.getPosition().isNear(IntakeConstants.Extension.EXTENSION_MIN_DISTANCE, HopperConstants.AGITATE_TOLERANCE))
                ).alongWith(sys_intake.retract())
        ).until(() -> {return sys_intake.getPosition().isNear(IntakeConstants.Extension.EXTENSION_MIN_DISTANCE, HopperConstants.AGITATE_TOLERANCE);})
                .andThen(sys_hopper.fullRetract());
        
    }

    Distance intakeSetpoint;
    Distance hopperSetpoint;

    // TODO: DETERMINE IF NEEDED, IF NOT DELETE
    /** 
     * Command to retract both intake and hopper subsystems, while agitating hopper back and forth to help with launching fuel
     * @author Jaden Rajan, team 5409
     * @author John Chen, team 5409
     */
    private Command retractAndAgitate() {
        return Commands.repeatingSequence(
                Commands.runOnce(() -> 
                        intakeSetpoint = sys_intake.getPosition().minus(IntakeConstants.Extension.RETRACT_INCREMENT)),
                Commands.runOnce(() -> 
                        hopperSetpoint = intakeSetpoint.plus(IntakeConstants.Extension.KILLSWITCH_TOLERANCE)
                                                        .minus(HopperConstants.STARTING_GAP_TO_INTAKE)),
                sys_intake.move(() -> intakeSetpoint),
                Commands.either(
                        sys_hopper.stopMotor(),
                        sys_hopper.setSetpoint(() -> hopperSetpoint),
                        () -> (sys_hopper.getPositionIntakeZero().minus(sys_intake.getPosition()))
                                        .lt(IntakeConstants.Extension.KILLSWITCH_TOLERANCE)
                ).repeatedly().until(() -> sys_hopper.getPosition().isNear(
                        hopperSetpoint, HopperConstants.AGITATE_TOLERANCE)),
                sys_hopper.setSetpoint(() -> hopperSetpoint.plus(HopperConstants.EXTEND_INCREMENT)),
                Commands.waitUntil(() -> 
                        sys_hopper.getPosition().isNear(hopperSetpoint.plus(HopperConstants.EXTEND_INCREMENT), 
                                                        HopperConstants.AGITATE_TOLERANCE))
        ).until(() -> sys_intake.getPosition().isNear(IntakeConstants.Extension.EXTENSION_MIN_DISTANCE, HopperConstants.AGITATE_TOLERANCE))
                .andThen(sys_hopper.fullRetract());
    }

    // TODO: DETERMINE IF NEEDED, IF NOT DELETE
    private Command agitate(){
        return
//                Commands.sequence(
//                sys_hopper.setSetpoint(() -> Meters.of(2.6)),
//                sys_intake.move(Meters.of(3.35)),
//                Commands.waitUntil(
//                                () -> sys_hopper.getPosition().isNear(sys_hopper.getSetpoint(), HopperConstants.AGITATE_TOLERANCE)
//                                        && sys_intake.getPosition().isNear(sys_intake.getSetpoint(), HopperConstants.AGITATE_TOLERANCE)
//                ),
                Commands.repeatingSequence(
                        Commands.parallel(
                                sys_intake.move(() -> sys_intake.getPosition().minus(Centimeters.of(2.5))),
                                sys_hopper.setSetpoint(() -> sys_hopper.getPosition().minus(Centimeters.of(2.5))),
                                Commands.print("pulling in")
                        ),
                        Commands.print("Waiting"),
                        Commands.waitUntil(
                                () -> sys_hopper.getPosition().isNear(sys_hopper.getSetpoint(), HopperConstants.AGITATE_TOLERANCE)
                                        && sys_intake.getPosition().isNear(sys_intake.getSetpoint(), HopperConstants.AGITATE_TOLERANCE)
                        ),
                        Commands.print("done waiting"),
                        Commands.parallel(
                                sys_hopper.setSetpoint(() -> sys_hopper.getPosition().plus(Centimeters.of(1.0))),
                                sys_intake.move(() -> sys_intake.getPosition().plus(Centimeters.of(1.0))),
                                Commands.print("pushing out")
                        ),
                        Commands.print("waiting 2"),
                        Commands.waitUntil(
                                () -> sys_hopper.getPosition().isNear(sys_hopper.getSetpoint(), HopperConstants.AGITATE_TOLERANCE)
                                        && sys_intake.getPosition().isNear(sys_intake.getSetpoint(), HopperConstants.AGITATE_TOLERANCE)
                        ),
                        Commands.print("done waiting")
//                )
        );
    }

        Distance extendPoint = Centimeters.of(23.5);
        Distance retractPoint = extendPoint.minus(Centimeters.of(7.5));

        // TODO: MOVE TO GAME COMMANDS
        public Command agitateIntake(double rollerVoltage) {
                return Commands.parallel(
                    sys_intake.setRollerVoltage(rollerVoltage),
                    Commands.repeatingSequence(
                        sys_intake.move(() -> retractPoint),
                        Commands.waitUntil(() -> sys_intake.getPosition().isNear(retractPoint, Centimeters.of(1.0))),
                        sys_intake.move(() -> extendPoint),
                        Commands.waitUntil(() -> sys_intake.getPosition().isNear(extendPoint, Centimeters.of(1.0)))
                    )
                );
        }

        // TODO: DETERMINE IF NEEDED, IF NOT DELETE, IF NEEDED MOVE TO GAME COMMANDS
        private Command runIntakeIn(double voltage){
            return Commands.sequence(
                sys_intake.setExtensionVoltage(voltage),
                Commands.waitUntil(() -> sys_intake.getPosition().isNear(Centimeters.of(1), Centimeters.of(1))),
                sys_intake.setExtensionVoltage(0)
            );
        }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
