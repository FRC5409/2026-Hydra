// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimbingPositions;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PassingPositions;
import frc.robot.Constants.kBump;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.AutoPath;
import frc.robot.util.FieldConstants.Hub;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.ArrayList;

import static edu.wpi.first.units.Units.Meters;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    protected final Drive  sys_drive;
    private final boolean hasDrive = true;

    protected final Vision   sys_vision;
    private final boolean hasVision = false;

    private final   Elevator sys_elevator;
    private final boolean hasElevator = true;


    public static SwerveDriveSimulation simConfig;

    private PassingPositions  selectedPassingPosition      = PassingPositions.MIDDLE;
    private ClimbingPositions selectedClimbingPosition     = ClimbingPositions.LEFT;
    private ClimbingPositions selectedClimbingPrepPosition = ClimbingPositions.LEFT_PREP;
//     private LinearVelocity bumpSpeed = MetersPerSecond.of(0.0);

    // Controllers
    private final CommandXboxController primaryController   = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);

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
                if (hasVision)
                    sys_vision = new Vision(new VisionIOLimelight());
                else
                    sys_vision = new Vision(new VisionIO() {});

                if (hasDrive){
                    sys_drive = new Drive(
                            new GyroIOPigeon2(),
                            new ModuleIOTalonFX(TunerConstants.FrontLeft),
                            new ModuleIOTalonFX(TunerConstants.FrontRight),
                            new ModuleIOTalonFX(TunerConstants.BackLeft),
                            new ModuleIOTalonFX(TunerConstants.BackRight),
                            sys_vision);
                } else {
                    sys_drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        sys_vision);
                }

                if (hasElevator)
                    sys_elevator = new Elevator(new ElevatorIOTalonFX(Constants.DeviceID.CLIMBER_MOTOR));
                else
                    sys_elevator = new Elevator(new ElevatorIO() {});
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
                sys_elevator = new Elevator(new ElevatorIO() {});
            }
        }

        // Set up auto routines
        autoChooser = buildAutoChooser();

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
        // ArrayList<AutoPath> autoPaths = Autos.getAutoPaths(sys_drive, sys_vision);

        // autoPaths.forEach(autoPath -> chooser.addOption(autoPath.getName(), autoPath));

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

        // primaryController.povUp()
        //                 .onTrue(sys_elevator.startManualMove(2))
        //                 .onFalse(sys_elevator.startManualMove(0));

        // primaryController.povDown()
        //                 .onTrue(sys_elevator.startManualMove(-2))
        //                 .onFalse(sys_elevator.startManualMove(0));

        // primaryController.povLeft()
        //                 .onTrue(sys_elevator.zeroEncoder());

        // primaryController.a()
        //                 .onTrue(sys_elevator.goTillSpike(-2));
        
        // primaryController.b()
        //                 .onTrue(sys_elevator.elevatorGo(Meters.of(3)));
        // primaryController.x()
        //                 .onTrue(sys_elevator.elevatorGo(Meters.of(0.5)));
        

        // primaryController.x()
        //         .onTrue(

        //         );

        // Switch to X pattern when X button is pressed
        // primaryController.x()
        //                  .onTrue(Commands.runOnce(sys_drive::stopWithX, sys_drive));

        // Switch To Bump Speed Modifier
        // primaryController.a()
        //                  .onTrue(Commands.runOnce(() -> DriveCommands.setSpeed(kBump.BUMP_SPEED_MODIFIER)))
        //                  .onFalse(Commands.runOnce(() -> DriveCommands.setSpeed(1.0)));

        // primaryController.rightBumper()
        //                  .whileTrue(
        //                          DriveCommands.alignToHeading(
        //                                  sys_drive,
        //                                  () -> DriveCommands.getRotation2d(
        //                                          sys_drive,
        //                                          new Pose2d(
        //                                                  new Translation2d(
        //                                                          Hub.topCenterPoint.getMeasureX(),
        //                                                          Hub.topCenterPoint.getMeasureY()),
        //                                                  Rotation2d.kZero
        //                                          )
        //                                  )
        //                          )
        //                  );

        // primaryController.leftBumper()
        //                 .whileTrue(
        //                   DriveCommands.joystickDriveAtAngle(
        //                     sys_drive,
        //                     () -> -primaryController.getLeftY(),
        //                     () -> -primaryController.getLeftX(),
        //                     () -> DriveCommands.getRotation2d(sys_drive, selectedPassingPosition.pose)
        //                   )
        //                 );

        // primaryController.x()
        //                 .whileTrue(
        //                     Commands.sequence(
        //                       DriveCommands.alignToPoint(
        //                         sys_drive, 
        //                         () -> selectedClimbingPrepPosition.pose,
        //                         () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
        //                         () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
        //                         kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
        //                         kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
        //                         kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP

        //                       ),
        //                       DriveCommands.alignToPoint(
        //                         sys_drive, 
        //                         () -> selectedClimbingPosition.pose, 
        //                         () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
        //                         () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
        //                       )
        //                     )
        //                 );

        // primaryController.x()
        //                  .whileTrue(

        //                          DriveCommands.crossBump(
        //                                  sys_drive,
        //                                  sys_vision,
        //                                  () -> Rotation2d.k180deg,
        //                                  DriveCommands.getBumpSpeed(sys_drive),
        //                                  500
        //                          )
        //                  );

        // secondaryController.x()
        //                    .onTrue(prepPassingPositionCommand(PassingPositions.LEFT));
        // secondaryController.b()
        //                    .onTrue(prepPassingPositionCommand(PassingPositions.RIGHT));
        // secondaryController.a()
        //                    .onTrue(prepPassingPositionCommand(PassingPositions.MIDDLE));

        // secondaryController.povLeft()
        //                    .onTrue(prepClimberPositionCommand(ClimbingPositions.LEFT));
        // secondaryController.povRight()
        //                    .onTrue(prepClimberPositionCommand(ClimbingPositions.RIGHT));

        // secondaryController.povUp()
        //                 .onTrue(Commands.runOnce(() -> Robot.rebuiltTimer.addFuel(5)));
        // secondaryController.povDown()
        //                 .onTrue(Commands.runOnce(() -> Robot.rebuiltTimer.addFuel(-5)));

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

    ;

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
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
