package frc.robot.util;

import java.util.Arrays;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPath extends SequentialCommandGroup {
    private final String autoName;
    private final Pose2d startingPose;

    public static Command followPath(String pathName){
        try{
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e){
            System.out.println("Path Error " + e);
            return null;
        }
    }

    public AutoPath(String autoName, Pose2d startingPose, Command... commands){
        super(
            Stream.concat(
                Stream.of(Commands.runOnce(() -> AutoBuilder.resetOdom(startingPose))),
                Arrays.stream(commands)
            ).toArray(Command[]::new)
        );        
        this.autoName = autoName;
        if (AutoBuilder.shouldFlip())
            this.startingPose = FlippingUtil.flipFieldPose(startingPose);
        else
            this.startingPose = startingPose;
    }
    
    public String getName(){
        return autoName;
    }

    public Pose2d getStartingPose(){
        return startingPose;
    }
}
