package frc.robot.subsystems.Serializer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;


// Indexer for Ortona
public class SerializerIOSparkMax implements SerializerIO {
    private SparkMax indexerMotor;
    private SparkMax feederMotor;
    private SparkMaxConfig indexerMotorConfig;
    private SparkMaxConfig feederMotorConfig;
    
    public SerializerIOSparkMax(int indexerMotorID, int feederMotorID) {
        indexerMotor =  new SparkMax(indexerMotorID, MotorType.kBrushless);
        feederMotor = new SparkMax(feederMotorID, MotorType.kBrushless);
        indexerMotorConfig = new SparkMaxConfig();
        feederMotorConfig = new SparkMaxConfig();

        indexerMotorConfig.smartCurrentLimit((int) SerializerConstants.ORTONA_SPARK_MAX_CURRENT_LIMIT.magnitude());
        indexerMotorConfig.idleMode(IdleMode.kBrake);
        indexerMotorConfig.inverted(SerializerConstants.ORTONA_INDEXER_MOTOR_INVERTED);

        feederMotorConfig.smartCurrentLimit((int) SerializerConstants.ORTONA_SPARK_MAX_CURRENT_LIMIT.magnitude());
        feederMotorConfig.idleMode(IdleMode.kBrake);
        feederMotorConfig.inverted(SerializerConstants.ORTONA_FEEDER_MOTOR_INVERTED);

        indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorVoltage(double voltage){
        indexerMotor.setVoltage(voltage);
    }

    @Override
    public void setIndexerMotorVoltage(double voltage){
        indexerMotor.setVoltage(voltage);
    }

    @Override
    public void setFeederMotorVoltage(double voltage){
        feederMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(SerializerInputs inputs){
        inputs.isFloorMotorConnected = !(indexerMotor.getFaults().motorType || indexerMotor.getFaults().can);
        inputs.floorAppliedVoltage = indexerMotor.get() * RobotController.getBatteryVoltage();
        inputs.floorAppliedCurrent = indexerMotor.getOutputCurrent();
        inputs.floorMotorTemperature = indexerMotor.getMotorTemperature();

        inputs.isFeederMotorConnected = !(feederMotor.getFaults().motorType || indexerMotor.getFaults().can);
        inputs.feederAppliedVoltage = feederMotor.get() * RobotController.getBatteryVoltage();
        inputs.feederAppliedCurrent = feederMotor.getOutputCurrent();
        inputs.feederMotorTemperature = feederMotor.getMotorTemperature();

    }

    

}
