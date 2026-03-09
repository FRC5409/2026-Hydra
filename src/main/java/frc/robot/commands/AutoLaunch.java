package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.util.FieldConstants.Hub;

public class AutoLaunch extends SequentialCommandGroup {

    public AutoLaunch(
        Drive sys_drive,
        Launcher sys_launcher,
        Feeder sys_feeder,
        Supplier<Distance> distanceSupplier
    ) {
        addCommands(
            // 1) Align to hub
            DriveCommands.alignToHeading(
                sys_drive,
                () -> DriveCommands.getRotation2d(
                    sys_drive,
                    new Pose2d(
                        new Translation2d(
                            Hub.topCenterPoint.getMeasureX(),
                            Hub.topCenterPoint.getMeasureY()
                        ),
                        Rotation2d.kZero
                    )
                )
            ),

            // 2) Then launch
            sys_launcher.launchFuel(distanceSupplier, sys_feeder)
        );
    }
}