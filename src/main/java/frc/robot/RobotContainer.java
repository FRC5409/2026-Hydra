// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConstants;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOTalonFX;
import frc.robot.subsystems.serializer.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;

import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
//    protected final Drive      sys_drive;
//    protected final Vision     sys_vision;
    protected final Serializer sys_serializer;
    protected final Feeder     sys_feeder;
    protected final Launcher   sys_launcher;

    public static SwerveDriveSimulation simConfig;

//    private PassingPositions selectedPassingPosition = PassingPositions.MIDDLE;
//    private ClimbingPositions selectedClimbingPosition = ClimbingPositions.LEFT;
//    private ClimbingPositions selectedClimibingPrepPosition = ClimbingPositions.LEFT_PREP;

    // Controllers
   private final CommandXboxController primaryController   = new CommandXboxController(0);
//    private final CommandXboxController secondaryController = new CommandXboxController(1);

    // Dashboard inputs
//    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            // Real robot, instantiate hardware IO implementations
            case REAL -> {
                sys_serializer = new Serializer(
                        new SerializerIOTalonFX(SerializerConstants.SERIALIZER_ID));
                sys_feeder = new Feeder(new FeederIOTalonFX(FeederConstants.FEEDER_ID));
//                sys_vision = new Vision(new VisionIOLimelight());
//
//                sys_drive = new Drive(
//                        new GyroIOPigeon2(),
//                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
//                        new ModuleIOTalonFX(TunerConstants.FrontRight),
//                        new ModuleIOTalonFX(TunerConstants.BackLeft),
//                        new ModuleIOTalonFX(TunerConstants.BackRight),
//                        sys_vision
//                );

                sys_launcher = new Launcher(new LauncherIOTalonFX(
                    LauncherConstants.Launcher.LAUNCHER_CAN_ID,
                    // LauncherConstants.Launcher.LAUNCHER_SENSOR_ID,
                    LauncherConstants.Launcher.FOLLOWER_LAUNCHER_CAN_ID,
                    // LauncherConstants.Hood.HOOD_CAN_ID,
                    LauncherConstants.Hood.HOOD_PWM_CHANNEL_1,
                    LauncherConstants.Hood.HOOD_PWM_CHANNEL_2
                //     LauncherConstants.Hood.HOOD_SENSOR_ID
                //     LauncherConstants.Ultrasonic.DIGITAL_OUTPUT,
                //     LauncherConstants.Ultrasonic.DIGITAL_INPUT
                ));

            }
            // Sim robot, instantiate physics sim IO implementations
            case SIM -> {
                sys_serializer = new Serializer(new SerializerIOSim());
                sys_feeder = new Feeder(new FeederIOSim());

                final DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig
                        .Default()
                        .withGyro(COTS.ofPigeon2())
                        .withRobotMass(DriveConstants.ROBOT_FULL_MASS)
                        .withTrackLengthTrackWidth(Meters.of(0.578), Meters.of(0.578))
                        .withBumperSize(Meters.of(0.881), Meters.of(0.881))
                        .withSwerveModule(
                                COTS.ofMark4i(
                                        DCMotor.getKrakenX60(1),
                                        DCMotor.getKrakenX60(1),
                                        DriveConstants.WHEEL_COF,
                                        1
                                )
                        );

                simConfig = new SwerveDriveSimulation(
                        driveConfig,
                        new Pose2d(3, 3, Rotation2d.kZero)
                );

                SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
                SimulatedArena.getInstance().addDriveTrainSimulation(simConfig);
                SimulatedArena.getInstance().resetFieldForAuto();

//                sys_vision = new Vision(new VisionIOSim(simConfig));
//
//                sys_drive = new Drive(
//                        new GyroIOSim(simConfig.getGyroSimulation()),
//                        new ModuleIOSim(simConfig.getModules()[0]),
//                        new ModuleIOSim(simConfig.getModules()[1]),
//                        new ModuleIOSim(simConfig.getModules()[2]),
//                        new ModuleIOSim(simConfig.getModules()[3]),
//                        sys_vision
//                );

                // TODO: disabled sim IO because i don't have time for ts
                sys_launcher = new Launcher(new LauncherIO() {});
            }
            // Replayed robot, disable IO implementations
            default -> {
//                sys_vision = new Vision(new VisionIO() {});
//                sys_drive = new Drive(
//                        new GyroIO() {},
//                        new ModuleIO() {},
//                        new ModuleIO() {},
//                        new ModuleIO() {},
//                        new ModuleIO() {},
//                        sys_vision);
                sys_serializer = new Serializer(new SerializerIO() {});
                sys_feeder = new Feeder(new FeederIO() {});
                sys_launcher = new Launcher(new LauncherIO() {});
            }
        }

        // Set up auto routines
//        autoChooser = buildAutoChooser();

        // Configure the button bindings
        configureButtonBindings();
    }

//    /**
//     * builds the dashboard command chooser ({@link LoggedDashboardChooser}) for picking autonomous routines.
//     *
//     * @return the logged dashboard chooser
//     */
//    private LoggedDashboardChooser<Command> buildAutoChooser() {
//        LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>(
//                "Auto Choices", AutoBuilder.buildAutoChooser());
//
//        // Set up SysId routines
//        chooser.addOption(
//                "Drive Wheel Radius Characterization",
//                DriveCommands.wheelRadiusCharacterization(sys_drive));
//        chooser.addOption(
//                "Drive Simple FF Characterization",
//                DriveCommands.feedforwardCharacterization(sys_drive));
//        chooser.addOption(
//                "Drive SysId (Quasistatic Forward)",
//                sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        chooser.addOption(
//                "Drive SysId (Quasistatic Reverse)",
//                sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//        chooser.addOption(
//                "Drive SysId (Dynamic Forward)",
//                sys_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        chooser.addOption(
//                "Drive SysId (Dynamic Reverse)",
//                sys_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
//
//        return chooser;
//    }

    /**
     * Updates sim positions of algae, coral and robot poses
     */
    public void updateSim() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("Simulation/RobotPose", simConfig.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
//        sys_drive.setDefaultCommand(
//                DriveCommands.joystickDrive(
//                        sys_drive,
//                        () -> -primaryController.getLeftY(),
//                        () -> -primaryController.getLeftX(),
//                        () -> -(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis())
//                )
//        );

        // TEST CODE FOR LAUNCHER PROTOTYPES
        // launch fuel w distance
        SmartDashboard.putNumber("LAUNCHER DISTANCE [m]", 5);
        SmartDashboard.putData("LAUNCH FUEL (DST)", sys_launcher.launchFuel(
              () -> Meters.of(SmartDashboard.getNumber("LAUNCHER DISTANCE [m]", 0))));

        SmartDashboard.putData("STOP LAUNCHER", sys_launcher.stopLauncher());

        // launch fuel w speed
        SmartDashboard.putNumber("LAUNCHER SPEED [rps]", 50);
        SmartDashboard.putData("LAUNCH FUEL (SPD)", sys_launcher.runRPS(
              () -> RotationsPerSecond.of(SmartDashboard.getNumber("LAUNCHER SPEED [rps]", 0))));

        SmartDashboard.putData("STOP LAUNCHER", sys_launcher.stopLauncher());

        // sequentially run every distance from 0.5 m to 10.0 m
        SmartDashboard.putData("LAUNCHER RUN ALL", new SequentialCommandGroup(
               DoubleStream.iterate(0, d -> d + 0.5)
                           .limit((int)(10 / 0.5) + 1)
                           .boxed()
                           .flatMap(d -> Stream.of(
                                   sys_launcher.launchFuel(() -> Meters.of(d)),
                                   new WaitCommand(0.5)))
                           .toArray(Command[]::new)
        ));

        SmartDashboard.putNumber("Hood Extension [mm]", 0);
        SmartDashboard.putData("Set Hood Extension", Commands.runOnce(() -> sys_launcher.setHoodPos(
                Millimeters.of(SmartDashboard.getNumber("Hood Extension [mm]", 0)))));

        SmartDashboard.putNumber("Hood Angle [deg]", 0);
        SmartDashboard.putData("Set Hood Angle", Commands.runOnce(() -> sys_launcher.setHoodPos(
                // linear regression (deg to mm)
                Millimeters.of(0.296*SmartDashboard.getNumber("Hood Angle [deg]", 0)+15.2))));

        SmartDashboard.putNumber("Feeder Voltage", 0);
        SmartDashboard.putNumber("Serializer Voltage", 0);


        primaryController.y()
                .onTrue(sys_launcher.runRPS(
                    () -> RotationsPerSecond.of(SmartDashboard.getNumber("LAUNCHER SPEED [rps]", 0)))
                );


        primaryController.a()
            .onTrue(sys_launcher.stopLauncher());


        primaryController.povUp()
            .onTrue(sys_feeder.runRPS(() -> SmartDashboard.getNumber("LAUNCHER SPEED [rps]", 0)))
            .onTrue(sys_serializer.setVoltage(8));


        primaryController.povDown()
            .onTrue(sys_feeder.setVoltage(0))
            .onTrue(sys_serializer.setVoltage(0));


        primaryController.povRight()
            .onTrue(sys_serializer.setVoltage(5));

        primaryController.povLeft()
            .onTrue(sys_serializer.setVoltage(0));


      // score fuel in hub by using odometry
//      var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(Vision.PRIMARY_CAM_NAME).pose;
//      Logger.recordOutput("Vision/Estimate", pose);
//      SmartDashboard.putData("SCORE FUEL IN HUB", sys_launcher.launchFuel(
//              () -> Meters.of(Hub.topCenterPoint.toTranslation2d().getDistance(pose.getTranslation()))));

    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TODO: temp. code for prototype
        return null;
    }
}
