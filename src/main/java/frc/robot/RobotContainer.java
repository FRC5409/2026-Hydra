// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.launcher.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static edu.wpi.first.units.Units.Centimeter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
//   private final Drive drive;

  protected final Launcher sys_launcher;
  // protected final Serializer sys_serializer;

  // Controller
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);

  // Dashboard inputs
//   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(),
        //         new ModuleIOTalonFX(TunerConstants.FrontLeft),
        //         new ModuleIOTalonFX(TunerConstants.FrontRight),
        //         new ModuleIOTalonFX(TunerConstants.BackLeft),
        //         new ModuleIOTalonFX(TunerConstants.BackRight));

        sys_launcher = new Launcher(new LauncherTalonFX(
                LauncherConstants.LAUNCHER_CAN_ID,
                LauncherConstants.LAUNCHER_SENSOR_ID,
                // LauncherConstants.HOOD_CAN_ID,
                // LauncherConstants.HOOD_SENSOR_ID,
                LauncherConstants.FOLLOWER_LAUNCHER_CAN_ID
                // LauncherConstants.FOLLOWER_LAUNCHER_SENSOR_ID
                )
            );

        // sys_serializer = new Serializer(
        //         new SerializerIOSparkMax(
        //             SerializerConstants.ORTONA_INDEXER_MOTOR_CANID, 
        //             SerializerConstants.ORTONA_FEEDER_MOTOR_CANID
        //         )
        //     );

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIOSim(TunerConstants.FrontLeft),
        //         new ModuleIOSim(TunerConstants.FrontRight),
        //         new ModuleIOSim(TunerConstants.BackLeft),
        //         new ModuleIOSim(TunerConstants.BackRight));

        sys_launcher =  new Launcher(new LauncherSim());

        // sys_serializer = new Serializer(new SerializerIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {});

        sys_launcher = new Launcher(new LauncherIO() {});
        // sys_serializer = new Serializer(new SerializerIO() {});

        break;
    }

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -primaryController.getLeftY(),
    //         () -> -primaryController.getLeftX(),
    //         () -> -(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis())
    //     )
    // );

    LoggedNetworkNumber voltageSetpoint = new LoggedNetworkNumber("Launcher Voltage Setpoint", 0);
    LoggedNetworkNumber velocitySetpoint = new LoggedNetworkNumber("Launcher Velocity Setpoint", 0);

    
    primaryController.povUp()
                     .onTrue(sys_launcher.setVoltage(voltageSetpoint))
                     .onFalse(sys_launcher.stop());

    primaryController.a()
                    .onTrue(sys_launcher.runVelocity(20));

    primaryController.y()
                    .onTrue(sys_launcher.stop());

    primaryController.x()
                    .onTrue(sys_launcher.launchFuel(Centimeter.of(640)));

    // primaryController.x()
    //     .whileTrue(sys_launcher.runVelocity(1));

    // primaryController.y()
    //     .onTrue(sys_serializer.runIndexerVoltage(8));

    // primaryController.a()
    //     .onTrue(sys_serializer.runFeederVoltage(5));
    
    // primaryController.povDown()
    //     .onTrue(sys_serializer.runFeederVoltage(0));

    // primaryController.povUp()
    //     .onTrue(sys_serializer.runIndexerVoltage(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//   public Command getAutonomousCommand() {
//     return autoChooser.get();
//   }
}