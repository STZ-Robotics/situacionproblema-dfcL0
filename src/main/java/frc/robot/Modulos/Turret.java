package frc.robot.Modulos;

import java.util.function.Supplier;
import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.Telemetry;
import com.stzteam.mars.models.singlemodule.ModularSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Interfaces.TurretIO;
import frc.robot.Interfaces.TurretIO.TurretInputs;
import frc.robot.configuration.KeyManager;

import frc.robot.Requests.TurretCommands;

import frc.robot.Requests.TurretRequest;




public class Turret extends ModularSubsystem<TurretInputs, TurretIO> implements TurretCommands {

    public Turret (TurretIO io){ 
        super(SubsystemBuilder.<TurretInputs, TurretIO> setup()
        .key(KeyManager.TURRET_KEY)
        .hardware(io, new TurretInputs())
        .request(new TurretRequest.Idle()) //asi va a empezar
        .telemetry(new TurretTelemetry())
        );

        setDefaultCommand(runRequest(() -> new TurretRequest.Idle()));
    }


    public boolean isAtTarget(double toleranceDegrees) {

        if (Math.abs(inputs.currentAngle.getDegrees() - inputs.targetAngle.getDegrees())< toleranceDegrees) {
            return true;
        } else {
            return false;
        }
    }




    // reporta estado
    @Override
    public TurretInputs getState(){
        return inputs;
    }

    // define control para módulo
    @Override
    public Command setControl(Supplier<TurretRequest> request){
        return runRequest(request);
    }

    @Override
    public void absolutePeriodic(TurretInputs data){
        //es para cálculos pero como ya los tenemos en la IO, no se usa 
    }

    @Override
    public void simulationPeriodic() {
        // tampoco se usa
    }

    public static class TurretTelemetry extends Telemetry<TurretInputs> {

        private static final String VOLTAGE_TURRET_KEY = "Applied Voltage";
        private static final String TARGETANGLE_TURRET_KEY = "Target Angle";
        private static final String CURRENTANGLE_TURRET_KEY = "Current Angle";
        private static final String VELOCITY_TURRET_KEY = "Velocity";
        private static final String LATENCY_TURRET_KEY = "Latency";

        @Override
        public void telemeterize(TurretInputs data) {
            NetworkIO.set(KeyManager.TURRET_KEY, VOLTAGE_TURRET_KEY, data.appliedVolts);
            NetworkIO.set(KeyManager.TURRET_KEY, TARGETANGLE_TURRET_KEY, data.targetAngle);
            NetworkIO.set(KeyManager.TURRET_KEY, CURRENTANGLE_TURRET_KEY, data.currentAngle);
            NetworkIO.set(KeyManager.TURRET_KEY, VELOCITY_TURRET_KEY, data.velocityRPS);
            NetworkIO.set(KeyManager.TURRET_KEY, LATENCY_TURRET_KEY, data.latency);
        }

    }

}
