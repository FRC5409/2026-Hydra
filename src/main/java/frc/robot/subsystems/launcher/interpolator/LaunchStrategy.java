package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.units.measure.Distance;

import java.util.ArrayList;

public abstract class LaunchStrategy {
    private static final ArrayList<LaunchStrategy> LAUNCH_STRATEGIES = new ArrayList<>();

    public LaunchStrategy() {
        LAUNCH_STRATEGIES.add(this);
    }

    /**
     * Interpolates the fastest angular velocity and shoot angle for the launcher based on the displacement to fire the
     * fuel.
     *
     * @param displacement total straight-line displacement to shoot fuel at
     *
     * @return {@link LaunchConfig}, containing angular velocity and angle to shoot at
     *
     * @apiNote Uses the active {@link LaunchStrategy}
     */
    public abstract LaunchConfig interpolate(Distance displacement);

    public abstract String getName();

    public static LaunchStrategy[] getLaunchStrategies() {
        return LAUNCH_STRATEGIES.toArray(LaunchStrategy[]::new);
    }
}
