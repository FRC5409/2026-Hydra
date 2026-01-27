package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;


public final class SerializerConstants {
    
    public static final Current ORTONA_SPARK_MAX_CURRENT_LIMIT = Amps.of(30);
    public static final boolean ORTONA_INDEXER_MOTOR_INVERTED = false;
    public static final boolean ORTONA_FEEDER_MOTOR_INVERTED = false;

    public static final int ORTONA_FEEDER_MOTOR_CANID = 5;
    public static final int ORTONA_INDEXER_MOTOR_CANID = 8;


}
