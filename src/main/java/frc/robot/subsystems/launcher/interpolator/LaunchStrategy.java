package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.units.measure.Distance;

public interface LaunchStrategy {
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
    LaunchConfig interpolate(Distance displacement);
}
