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
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private final boolean inverted = false;
    
    public SerializerIOSparkMax(int ID) {
        motor =  new SparkMax(ID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();

        motorConfig.smartCurrentLimit(30);
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(inverted);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(SerializerInputs inputs){
        inputs.floorMotorConnection = !(motor.getFaults().motorType || motor.getFaults().can);
        inputs.floorAppliedVoltage = motor.get() * RobotController.getBatteryVoltage();
        inputs.floorAppliedCurrent = motor.getOutputCurrent();
        inputs.floorMotorTemperature = motor.getMotorTemperature();
    }

    

}
