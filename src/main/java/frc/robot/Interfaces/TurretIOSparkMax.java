package frc.robot.Interfaces;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.TurretConstants;

public class TurretIOSparkMax implements TurretIO{

    private final SparkMax turretSpark;
    private final RelativeEncoder turretEncoder;

    private Rotation2d currentTargetAngle = new Rotation2d();

    public TurretIOSparkMax (){

        turretSpark = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);
        turretEncoder = turretSpark.getEncoder();
        var config = new SparkMaxConfig();

        //configuramos nuestros límites físicos 
        config.softLimit
        .forwardSoftLimit(TurretConstants.kUpperLimit)
        .reverseSoftLimit(TurretConstants.kLowerLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);


        config.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD)
        .outputRange(TurretConstants.kMinOutput, TurretConstants.kMaxOutput);   //limitar pid de -1 a 1


        //configuración de feedforward para control cerrado con FF
        config.closedLoop.feedForward.
        kS(TurretConstants.kS).
        kV(TurretConstants.kV).
        kA(TurretConstants.kA);


        //conversión de velocidad y posición
        config.encoder.velocityConversionFactor(TurretConstants.kVelocityFactor); //Es 20:1, RPM a RPS
        config.encoder.positionConversionFactor(TurretConstants.kPositionFactor); //rotaciones del motor a rotaciones de mi mecanismo  

        
        //aplicar configuración 
        turretSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    @Override
    public void setVoltage(double appliedVolts){
        turretSpark.setVoltage(appliedVolts);
    }   

    @Override
    public void setPosition(Rotation2d currentAngle){
        this.currentTargetAngle = currentAngle;
        turretSpark.getClosedLoopController().setSetpoint(
            currentAngle.getRotations(), //turret trabaja en rotaciones
            ControlType.kPosition); //posición, ir a angulo específico
    }

    @Override
    public void setDutyCycle(double percentage){
        turretSpark.set(percentage);
    }

    @Override
    public void setPositionWithFF(Rotation2d currentAngle, double feedforward){
        this.currentTargetAngle = currentAngle;
        turretSpark.getClosedLoopController().setSetpoint(currentAngle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0,feedforward,ArbFFUnits.kVoltage);
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

        inputs.currentAngle = Rotation2d.fromRotations(-turretEncoder.getPosition()); //nos pide el valor en rotation2d, realizamos conversión, puede pasarse a negativo si es necesario
        inputs.targetAngle = this.currentTargetAngle; //el objetivo actual que le estamos pidiendo a la torreta
        inputs.velocityRPS = turretEncoder.getVelocity();
        inputs.appliedVolts = turretSpark.getAppliedOutput()*turretSpark.getBusVoltage();
        inputs.current = turretSpark.getOutputCurrent();

        
    }


    
}
