package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.units.measure.Distance;

public abstract class LaunchStrategy {
    public LaunchStrategy() {
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

    /**
     * Gets a list of all the available {@link LaunchStrategy} that the user can choose from.
     */
    public static LaunchStrategy[] getLaunchStrategies() {
        return new LaunchStrategy[]{
                new BilinearStrategy(),
                new MatrixStrategy()
        };
    }
}
