// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.FieldConstants.Hub;
import frc.robot.util.LimelightHelpers;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
//  private final Drive drive;
  // private final Intake sys_intake;

  protected final Launcher sys_launcher;
  // protected final Serializer sys_serializer;
  protected final Vision sys_vision;

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
                LauncherConstants.FOLLOWER_LAUNCHER_CAN_ID
                // LauncherConstants.HOOD_CAN_ID,
                // LauncherConstants.HOOD_SENSOR_ID)
                ));

        sys_vision = new Vision(new VisionIOLimelight());

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
        // sys_intake = new Intake(new IntakeIOSim());
        // Sim robot, instantiate physics sim IO implementations
//        drive =
//            new Drive(
//                new GyroIO() {},
//                new ModuleIOSim(TunerConstants.FrontLeft),
//                new ModuleIOSim(TunerConstants.FrontRight),
//                new ModuleIOSim(TunerConstants.BackLeft),
//                new ModuleIOSim(TunerConstants.BackRight));

        sys_launcher =  new Launcher(new LauncherSim());
        sys_vision = new Vision(new VisionIO() {});

        break;

      default:
        // sys_intake = new Intake(new IntakeIO(){});
        // Replayed robot, disable IO implementations
//        drive =
//            new Drive(
//                new GyroIO() {},
//                new ModuleIO() {},
//                new ModuleIO() {},
//                new ModuleIO() {},
//                new ModuleIO() {});

        sys_launcher = new Launcher(new LauncherIO() {});
        sys_vision = new Vision(new VisionIO() {});

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
                    .onTrue(sys_launcher.runVelocity(RotationsPerSecond.of(20)));

                    
    primaryController.y()
                    .onTrue(sys_launcher.stop());

    primaryController.x()
                    .onTrue(sys_launcher.launchFuel(() -> Centimeter.of(640)));

      // launch fuel w distance
      SmartDashboard.putNumber("LAUNCHER DISTANCE [m]", 5);
      SmartDashboard.putData("LAUNCH FUEL (DST)", sys_launcher.launchFuel(
              () -> Meters.of(SmartDashboard.getNumber("LAUNCHER DISTANCE [m]", 0))));

      SmartDashboard.putData("STOP LAUNCHER", sys_launcher.stop());

      // launch fuel w speed
      SmartDashboard.putNumber("LAUNCHER SPEED [rps]", 50);
      SmartDashboard.putData("LAUNCH FUEL (SPD)", sys_launcher.runVelocity(
              () -> RotationsPerSecond.of(SmartDashboard.getNumber("LAUNCHER SPEED [rps]", 0))));

      SmartDashboard.putData("STOP LAUNCHER", sys_launcher.stop());

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

       // score fuel in hub by using odometry
       var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(Vision.PRIMARY_CAM_NAME).pose;
       Logger.recordOutput("Vision/Estimate", pose);
        SmartDashboard.putData("SCORE FUEL IN HUB", sys_launcher.launchFuel(
                () -> Meters.of(Hub.topCenterPoint.toTranslation2d().getDistance(pose.getTranslation()))));


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

//  /**
//   * Use this to pass the autonomous command to the main {@link Robot} class.
//   *
//   * @return the command to run in autonomous
//   */
//  public Command getAutonomousCommand() {
//    return autoChooser.get();
//  }
}