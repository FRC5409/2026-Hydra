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
import frc.robot.Constants.ClimbingPositions;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PassingPositions;
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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Constants.DeviceID;

import java.util.ArrayList;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    protected final Drive      sys_drive;
    protected final Vision     sys_vision;
    protected final Intake     sys_intake;
    protected final Serializer sys_serializer;
    protected final Feeder     sys_feeder;
    protected final Hopper     sys_hopper;
   
    protected final Launcher   sys_launcher;
    private final   Elevator   sys_elevator;

    public static SwerveDriveSimulation simConfig;

    private PassingPositions selectedPassingPosition = PassingPositions.MIDDLE;
    private ClimbingPositions selectedClimbingPosition = ClimbingPositions.LEFT;
    private ClimbingPositions selectedClimbingPrepPosition = ClimbingPositions.LEFT_PREP;


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
        ArrayList<AutoPath> autoPaths = Autos.getAutoPaths(sys_drive, sys_vision);

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
        // Default command, normal field-relative drive
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

        if (Constants.IS_TUNING){
            LoggedNetworkNumber driveAngleSetpoint = new LoggedNetworkNumber("DriveTuning/angleSetpoint", 0.0);
            LoggedNetworkNumber driveTurnVelocitySetpoint = new LoggedNetworkNumber("DriveTuning/driveTurnVelocitySetpoint", 0.0);
            LoggedNetworkNumber driveTurnVoltageSetpoint = new LoggedNetworkNumber("DriveTuning/driveTurnVoltageSetpoint", 0.0);

            SmartDashboard.putData(
                "Run Turn Setpoint",
                Commands.run(
                    () -> sys_drive.runTurnSetpoint(new Rotation2d(Degrees.of(driveAngleSetpoint.get()))), 
                    sys_drive
                )
            );

            SmartDashboard.putData(
                "Run turn velocity",
                Commands.run(
                    () -> sys_drive.runTurnVelocity(RadiansPerSecond.of(driveTurnVelocitySetpoint.get())), 
                    sys_drive
                )
            );

            SmartDashboard.putData(
                "Run turn voltage",
                Commands.run(
                    () -> sys_drive.runTurnVoltage(driveTurnVoltageSetpoint.get()), 
                    sys_drive
                )
            );            
        }
        SmartDashboard.putNumber("Launcher Speed Offset [rps]", Launcher.getSpeedOffset().in(RotationsPerSecond));
        SmartDashboard.putData("Update Offset Now", Commands.runOnce(() -> Launcher.setSpeedOffset(
                RotationsPerSecond.of(SmartDashboard.getNumber("Launcher Speed Offset [rps]", 0.0)))));

        // TEST CODE FOR LAUNCHER PROTOTYPES
        // launch fuel w distance
        // TODO: remove some of these when merging to main, or maybe make a DebugCommand interface
        SmartDashboard.putNumber("LAUNCHER DISTANCE [m]", 5);
        SmartDashboard.putData(
                "LAUNCH FUEL (DST)", sys_launcher.launchFuel(
                        () -> Meters.of(SmartDashboard.getNumber("LAUNCHER DISTANCE [m]", 0)), sys_feeder));

        SmartDashboard.putData("STOP LAUNCHER", sys_launcher.stopLauncher());

        // launch fuel w speed
        SmartDashboard.putNumber("LAUNCHER SPEED [rps]", 50);
        SmartDashboard.putData(
                "LAUNCH FUEL (SPD)", sys_launcher.runVelocity(
                        () -> RotationsPerSecond.of(SmartDashboard.getNumber("LAUNCHER SPEED [rps]", 0))));

        primaryController.y()
                        .onTrue(sys_launcher.runVelocity(
                                () -> RotationsPerSecond.of(SmartDashboard.getNumber("LAUNCHER SPEED [rps]", 0))))
                        .onFalse(sys_launcher.stopLauncher());

        primaryController.a()
                .onTrue(sys_serializer.setVoltage(8))
                .onTrue(sys_intake.setRollerVoltage(10))
                .onFalse(sys_serializer.setVoltage(0))
                .onFalse(sys_intake.setRollerVoltage(0));

//        primaryController.povUp()
//                        .onTrue(sys_launcher.setHoodExtension(() -> Millimeter.of(100)));
        // primaryController.povUp()
//                        .onFalse(Commands.runOnce(() -> SmartDashboard.putNumber("LAUNCHER SPEED [rps]", SmartDashboard.getNumber("LAUNCHER SPEED [rps]") + 5)));
//        primaryController.povDown()
//                .onTrue(sys_launcher.setHoodExtension(() -> Millimeter.of(0)));

        primaryController.x()
                .onTrue(sys_launcher.runVelocity(() -> RotationsPerSecond.of(SmartDashboard.getNumber("LAUNCHER SPEED [rps]", 0))))
                .onTrue(sys_feeder.runRPS(() -> RotationsPerSecond.of(SmartDashboard.getNumber("LAUNCHER SPEED [rps]", 0))))
                .onFalse(sys_feeder.stopMotor())
                .onFalse(sys_launcher.stopLauncher());

        primaryController.b()
                .onTrue(sys_launcher.runVelocity(() -> RotationsPerSecond.of(20)))
                .onFalse(sys_launcher.stopLauncher());

        SmartDashboard.putData("STOP LAUNCHER", sys_launcher.stopLauncher());

        

        SmartDashboard.putNumber("Hood Angle [mm]", 0);
        SmartDashboard.putData(
                "Set Hood Angle", sys_launcher.setHoodExtension(() ->
                        Millimeter.of(
                                                                            SmartDashboard.getNumber(
                                                                                    "Hood Angle [mm]",
                                                                                    0))));

        // Switch to X pattern when X button is pressed
        SmartDashboard.putNumber("SerializerVoltage", 0.0);
        // primaryController.x()
        //                 .onTrue(sys_serializer.setVoltage(5))
        //                 .onFalse(sys_serializer.setVoltage(0));

        // Switch To Bump Speed Modifier
        // primaryController.a()
        //                  .onTrue(Commands.runOnce(() -> DriveCommands.setSpeed(kBump.BUMP_SPEED_MODIFIER)))
        //                  .onFalse(Commands.runOnce(() -> DriveCommands.setSpeed(1.0)));

        // primaryController.x()
        //     .onTrue(sys_elevator.goTillSpike(-1));

        // primaryController.povUp()
        // .onTrue(sys_elevator.startManualMove(1.0))
        // .onFalse(sys_elevator.startManualMove(0));

        // primaryController.povDown()
        // .onTrue(sys_elevator.startManualMove(-1.0))
        // .onFalse(sys_elevator.startManualMove(0));

        // primaryController.a()
        //         .onTrue(sys_elevator.elevatorGo(Meters.of(1.0),0));
        
        // primaryController.y()
        //         .onTrue(sys_elevator.elevatorGo(Meters.of(0.05),0));

        // primaryController.povUp()
        //         .onTrue(sys_hopper.setVoltage(-2))
        //         .onFalse(sys_hopper.setVoltage(0));

        // primaryController.povDown()
        //         .onTrue(sys_hopper.setVoltage(2))
        //         .onFalse(sys_hopper.setVoltage(0));

        // primaryController.povLeft()
        //         .onTrue(sys_intake.setExtensionVoltage(-2))
        //         .onFalse(sys_intake.setExtensionVoltage(0));
        
        // primaryController.povRight()
        //         .onTrue(sys_intake.setExtensionVoltage(2))
        //         .onFalse(sys_intake.setExtensionVoltage(0));


		
//		primaryController.y()
//                .onTrue(sys_hopper.setSetpoint(() -> Meters.of(0.26)))
//                .onFalse(sys_hopper.setVoltage(0));
        // primaryController.a()
        //         .onTrue(sys_hopper.setSetpoint(() -> Meters.of(0.01)))
        //         .onFalse(sys_hopper.setVolta`ge(0));
        // primaryController.a()
        //         .onTrue(agitate())
        //         .onFalse(Commands.parallel(sys_hopper.setVoltage(0), sys_intake.setExtensionVoltage(0)));

        // primaryController.b()
        //         .onTrue(sys_intake.move(Meters.of(0.22)))
        //         .onFalse(sys_intake.setExtensionVoltage(0));

        // primaryController.x()
        //         .onTrue(sys_intake.move(Meters.of(0.01)))
        //         .onFalse(sys_intake.setExtensionVoltage(0));

        // secondaryController.b()
        //         .onTrue(getAutonomousCommand());

        



        // secondaryController.povLeft()
        //         .onTrue(sys_hopper.setVoltage(-2))
        //         .onFalse(sys_hopper.setVoltage(0));

        // secondaryController.povRight()
        //         .onTrue(sys_hopper.setVoltage(2))
        //         .onFalse(sys_hopper.setVoltage(0));

        



        // tertiaryController.povRight()
        //         .onTrue(sys_intake.setExtensionVoltage(1))
        //         .onFalse(sys_intake.setExtensionVoltage(0));

        // tertiaryController.povLeft()
        //         .onTrue(sys_intake.setExtensionVoltage(-1))
        //         .onFalse(sys_intake.setExtensionVoltage(0));

        //         // Expected extension and retract values
        

        tertiaryController.y()
                .onTrue(sys_intake.setRollerVoltage(6))
                .onFalse(sys_intake.setRollerVoltage(0));

        tertiaryController.a()
                .onTrue(sys_intake.setRollerVoltage(-6))
                .onFalse(sys_intake.setRollerVoltage(0));


        // primaryController.povUp().onTrue(sys_feeder.setVoltage(3))
        //         .onFalse(sys_feeder.setVoltage(0));
            
        // primaryController.a().onTrue(sys_feeder.runRPS(() -> sys_feeder.targetRPS))
        // .onFalse(sys_feeder.setVoltage(0));

        SmartDashboard.putData("Hopper/Coast", sys_hopper.coastMode().ignoringDisable(true)); //TODO remhoppeove when main
        SmartDashboard.putData("Hopper/Brake", sys_hopper.brakeMode().ignoringDisable(true)); //TODO remhoppeove when main

        SmartDashboard.putData("Intake/Coast", sys_intake.coastMode().ignoringDisable(true));
        SmartDashboard.putData("Intake/Brake", sys_intake.brakemode().ignoringDisable(true));
  

        SmartDashboard.putData("extend", sys_intake.extend()); //TODO remove when main
        SmartDashboard.putData("retract", sys_intake.retract());
        SmartDashboard.putData("Start Roller", sys_intake.setRollerVoltage(12.0));
        SmartDashboard.putData("Stop Roller", sys_intake.setRollerVoltage(0.0));

        // SmartDashboard.putData(("sys_drive"));
    }

    private Command prepClimberPositionCommand(ClimbingPositions climbingPosition) {
        return Commands.runOnce(
                () -> {
                    if (climbingPosition == ClimbingPositions.LEFT)
                        selectedClimbingPrepPosition = ClimbingPositions.LEFT_PREP;
                    else
                        selectedClimbingPrepPosition = ClimbingPositions.RIGHT_PREP;

                    Logger.recordOutput("Climbing Position", climbingPosition);

                    selectedClimbingPosition = climbingPosition;

                    Logger.recordOutput("Climbing Selected Pose", selectedClimbingPosition.pose);
                }
        );
    }

    private Command prepPassingPositionCommand(PassingPositions passingPosition) {
        return Commands.runOnce(
                () -> {
                    Logger.recordOutput("Passing Position", passingPosition);

                    selectedPassingPosition = passingPosition;

                    Logger.recordOutput("Passing Selected Pose", selectedPassingPosition.pose);
                }
        );
    }

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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
