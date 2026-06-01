package frc.robot.Interfaces;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.TurretConstants;

public class TurretIOSim implements TurretIO{

    private final SingleJointedArmSim turretSim;
    private final ProfiledPIDController pidController;

    DCMotor gearbox = DCMotor.getNEO(1); //1 solo neo mueve mi torreta
    public double appliedVolts = 0.0;
    public double targetAngle = 0.0;
    public boolean isClosedLoop = false;

    public TurretIOSim (){
        
        turretSim = new SingleJointedArmSim(
            gearbox, // DCMotor gearbox, motor que mueve mi mecanismo
            TurretConstants.kGearRatio, //gear ratio 20:1
            SingleJointedArmSim.estimateMOI(TurretConstants.kRadius.in(Meters), TurretConstants.kMass.in(Kilograms)), //Momento de inercia, como kRadius y kMass son unidades diferentes en constants y nos pide meters y kilograms, lo convertimos
            TurretConstants.kRadius.in(Meters), //longitud del brazo, en este caso el radio de la torreta
            TurretConstants.kLowerLimit, //angulo minimo, límite inferior
            TurretConstants.kUpperLimit, //angulo maximo, límite superior
            false, // false = porque gira en horizontal 
            0, // posición inicial, empieza en 0
            null);

        pidController = 
        new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, 
        new TrapezoidProfile.Constraints(180.0, 360.0)); //velocidad máxima, aceleración máxima

    }



    @Override
    public void setVoltage(double appliedVolts) {
        isClosedLoop = false; // si se está aplicando un voltaje directo, no estamos en control cerrado
        this.appliedVolts = appliedVolts; //guardamos el voltaje aplicado para usarlo en la simulación e inputs
    }

    @Override
    public void setPosition(Rotation2d targetAngle) {
        isClosedLoop = true;
        this.targetAngle = targetAngle.getRotations();
        pidController.setGoal(this.targetAngle);
    }

    @Override
    public void setPositionWithFF(Rotation2d targetAngle, double velocityRPS) {
        isClosedLoop = true;
        this.targetAngle = targetAngle.getRotations();
        pidController.setGoal(this.targetAngle);
    }

    @Override
    public void setSpeed(double speed) {
        isClosedLoop = false;
        this.appliedVolts = speed * TurretConstants.kMaxVolts; //convertimos porcentaje de velocidad a voltaje real
    }

    @Override
    public void stop() {
        isClosedLoop = false;
        this.appliedVolts = 0.0;
    }

    @Override
    public void resetEnc() {
        // *
    }


    @Override
    public void updateInputs(TurretInputs inputs) {

        if (isClosedLoop) {
            appliedVolts = pidController.calculate(Units.radiansToRotations(turretSim.getAngleRads()), targetAngle);
        }

        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0); //clamp para no pasar el voltaje máximo
        turretSim.setInput(appliedVolts); //aplicamos el voltaje a la simulación
        turretSim.update(0.02); //actualizamos la simulación con un timestep de 20ms

        inputs.appliedVolts = appliedVolts;
        inputs.currentAngle = Rotation2d.fromRotations(Units.radiansToRotations(turretSim.getAngleRads()));
        inputs.velocityRPS = Units.radiansToRotations(turretSim.getVelocityRadPerSec());
        inputs.current = turretSim.getCurrentDrawAmps();
        inputs.targetAngle = Rotation2d.fromRotations(targetAngle);
    }

    
}
