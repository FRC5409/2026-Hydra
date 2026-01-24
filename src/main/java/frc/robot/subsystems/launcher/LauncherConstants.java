// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class LauncherConstants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class kLauncher {
    public static final int LAUNCHER_CANDID = 5;
    public static final int LAUNCHER_SENSORID = 1;
    public static final int HOOD_CANID = 1;
    public static final int HOOD_SENSORID = 1;

    public static final double kG = 0.0;
    public static final double kS = 0.1;
    public static final double kV = 0.12;
    public static final double kP = 0.11;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
  }
}
