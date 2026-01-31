package frc.robot.subsystems.launcher;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/**
 * Interpolates a {@link LaunchConfig} (angular velocity and shoot angle) given a displacement to shoot the fuel.
 *
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class LauncherInterpolator {
    /**
     * map of tested shots keyed by distance to travel, storing 2x1 matrices [angle (rad), velocity (rps)]
     */
    private static final InterpolatingMatrixTreeMap<Double, N2, N1> INTERPOLATOR = new InterpolatingMatrixTreeMap<>();

    /**
     * Adds a test point to the {@link InterpolatingMatrixTreeMap} used internally by the data interpolator
     *
     * @param angle        angle that the fuel was shot at
     * @param speed        angular velocity that the fuel was shot at
     * @param displacement total displacement that the fuel traveled
     */
    private static void addData(Angle angle, AngularVelocity speed, Distance displacement) {
        Matrix<N2, N1> matrix = new Matrix<>(N2.instance, N1.instance);
        matrix.set(0, 0, angle.in(Units.Radians));
        matrix.set(1, 0, speed.in(Units.RotationsPerSecond));
        INTERPOLATOR.put(displacement.in(Units.Meters), matrix);
    }

    /**
     * Interpolates the fastest angular velocity and shoot angle for the launcher based on the displacement to fire the
     * fuel.
     *
     * @param displacement total straight-line displacement to shoot fuel at
     *
     * @return {@link LaunchConfig}, containing angular velocity and angle to shoot at
     */
    public LaunchConfig interpolate(Distance displacement) {
        Matrix<N2, N1> interpolated = INTERPOLATOR.get(displacement.in(Units.Meters));

        return new LaunchConfig(
            Units.Radians.of(interpolated.get(0, 0)),
            Units.RotationsPerSecond.of(interpolated.get(1, 0))
        );
    }

//    static {
//
//    }

    public record LaunchConfig(Angle angle, AngularVelocity speed) {}
}
