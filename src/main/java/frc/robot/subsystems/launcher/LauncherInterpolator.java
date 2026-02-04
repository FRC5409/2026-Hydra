package frc.robot.subsystems.launcher;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;

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
     * @param angle  angle that the fuel was shot at
     * @param speed  angular velocity that the fuel was shot at
     * @param trials total displacement traveled, ideally use 3 trials
     */
    private static void addData(Angle angle, AngularVelocity speed, Distance... trials) {
        Matrix<N2, N1> matrix = new Matrix<>(N2.instance, N1.instance);
        matrix.set(0, 0, angle.in(Radians));
        matrix.set(1, 0, speed.in(RotationsPerSecond));
        INTERPOLATOR.put(Arrays.stream(trials).mapToDouble(d -> d.in(Meters)).average().orElseThrow(), matrix);
    }

    /**
     * Interpolates the fastest angular velocity and shoot angle for the launcher based on the displacement to fire the
     * fuel.
     *
     * @param displacement total straight-line displacement to shoot fuel at
     *
     * @return {@link LaunchConfig}, containing angular velocity and angle to shoot at
     */
    public static LaunchConfig interpolate(Distance displacement) {
        Matrix<N2, N1> interpolated = INTERPOLATOR.get(displacement.in(Meters));

        return new LaunchConfig(
                Radians.of(interpolated.get(0, 0)),
                RotationsPerSecond.of(interpolated.get(1, 0))
        );
    }

    static {
        // ==== TESTING DATA FOR PROTOTYPE LAUNCHER ====

        // first test, hit ground
//        addData(Degrees.of(60), RotationsPerSecond.of(50),
//                Meters.of(3.9), Meters.of(4.0), Meters.of(4.05));
//
//        // testing based on x-position of initial landing
//        addData(Degrees.of(60), RotationsPerSecond.of(65),
//                Meters.of(5.33), Meters.of(5.33), Meters.of(5.5));
//
//        addData(Degrees.of(60), RotationsPerSecond.of(70),
//                Meters.of(5.5), Meters.of(5.5), Meters.of(5.35));
//
//        addData(Degrees.of(75), RotationsPerSecond.of(70),
//                Meters.of(4.7), Meters.of(5.3), Meters.of(5.3));
//
//        addData(Degrees.of(75), RotationsPerSecond.of(80),
//                Meters.of(6.2), Meters.of(4.8), Meters.of(5.2));
//
//        addData(Degrees.of(75), RotationsPerSecond.of(85),
//                Meters.of(6.5), Meters.of(6.4), Meters.of(6.2));

        addData(Degrees.of(75), RotationsPerSecond.of(50), Meters.of(2.30));
        addData(Degrees.of(75), RotationsPerSecond.of(60), Meters.of(3.60));
        addData(Degrees.of(75), RotationsPerSecond.of(70), Meters.of(4.75));
        addData(Degrees.of(75), RotationsPerSecond.of(80), Meters.of(6.25));
        addData(Degrees.of(75), RotationsPerSecond.of(90), Meters.of(7.68));
        addData(Degrees.of(75), RotationsPerSecond.of(100), Meters.of(9.40));
    }

    public record LaunchConfig(Angle angle, AngularVelocity speed) {}
}
