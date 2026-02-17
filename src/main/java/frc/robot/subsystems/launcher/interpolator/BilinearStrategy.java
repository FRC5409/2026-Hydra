package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

/**
 * Interpolates a {@link LaunchConfig} (angular velocity and shoot angle) given a displacement to shoot the fuel.
 * <p>
 * The angular velocity and launch angle are separated into 2 different functions, only correlated by the common
 * independent variable (distance).
 *
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class BilinearStrategy implements LaunchStrategy {
    private static final InterpolatingMatrixTreeMap<Double, N1, N1> ANGLE_INTERPOLATOR    =
            new InterpolatingMatrixTreeMap<>();
    private static final InterpolatingMatrixTreeMap<Double, N1, N1> VELOCITY_INTERPOLATOR =
            new InterpolatingMatrixTreeMap<>();

    @Override
    public LaunchConfig interpolate(Distance displacement) {
        return new LaunchConfig(
                Radians.of(ANGLE_INTERPOLATOR.get(displacement.in(Meters)).get(0, 0)),
                RotationsPerSecond.of(VELOCITY_INTERPOLATOR.get(displacement.in(Meters)).get(0, 0))
        );
    }

    /**
     * Adds a test point to the 2 tree maps used internally by the data interpolator
     *
     * @param angle    angle that the fuel was shot at
     * @param speed    angular velocity that the fuel was shot at
     * @param distance total distance the fuel traveled
     */
    private static void addData(Angle angle, AngularVelocity speed, Distance distance) {
        Matrix<N1, N1> theta = new Matrix<>(N1.instance, N1.instance);
        theta.set(0, 0, angle.in(Radians));
        ANGLE_INTERPOLATOR.put(distance.in(Meters), theta);

        Matrix<N1, N1> vel = new Matrix<>(N1.instance, N1.instance);
        vel.set(0, 0, speed.in(RotationsPerSecond));
        VELOCITY_INTERPOLATOR.put(distance.in(Meters), vel);
    }

    static {
        // ==== TESTING DATA FOR PROTOTYPE LAUNCHER ====
        addData(Degrees.of(75), RotationsPerSecond.of(50), Meters.of(2.30));
    }
}
