package frc.robot.Interfaces;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.TurretConstants;

public class TurretIOSparkMax implements TurretIO{

    private final SparkMax turretSpark;
    private final RelativeEncoder turretEncoder;

    public TurretIOSparkMax (int kMotorId){

        turretSpark = new SparkMax(kMotorId, MotorType.kBrushless);
        turretEncoder = turretSpark.getEncoder();
        var config = new SparkMaxConfig();

        //configuramos nuestros límites 
        config.softLimit
        .forwardSoftLimit(TurretConstants.kUpperLimit)
        .reverseSoftLimit(TurretConstants.kLowerLimit);

        //conversión de velocidad y posición
        config.encoder.velocityConversionFactor(1/(20*60)); //Es 20:1, RPM a RPS
        config.encoder.positionConversionFactor(1/20); //rotaciones del motor a rotaciones de mi mecanismo  
        
        //aplicar configuración 
        turretSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    @Override
    public void setVoltage(double appliedVolts){
        turretSpark.setVoltage(appliedVolts);
    }

    @Override
    public void setPosition(Rotation2d targetAngle){
        turretEncoder.setPosition(targetAngle.getRotations());
    }

    @Override
    public void setPositionWithFF(Rotation2d targetAngle, double velocityRPS){
        //* 

    }

    @Override
    public void setSpeed(double speed){
        turretSpark.set(speed);
    }

    @Override
    public void stop(){
        turretSpark.stopMotor();
    }

    @Override
    public void resetEnc(){
        turretEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(TurretInputs inputs){

        inputs.currentAngle = Rotation2d.fromRotations(turretEncoder.getPosition()); //nos pide el valor en rotation2d, realizamos conversión, puede pasarse a negativo si es necesario
        inputs.velocityRPS = turretEncoder.getVelocity();
        inputs.appliedVolts = turretSpark.getAppliedOutput()*turretSpark.getBusVoltage();
        inputs.current = turretSpark.getOutputCurrent();

        
    }


    
}
