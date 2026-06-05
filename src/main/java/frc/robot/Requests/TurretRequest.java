package frc.robot.Requests;

import java.util.function.DoubleSupplier;

import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Interfaces.TurretIO;
import frc.robot.Interfaces.TurretIO.TurretInputs;

@RequestFactory
public interface TurretRequest extends Request<TurretInputs, TurretIO> {

    public static final ModuleColorCode IDLE = ModuleColorCode.solid("IDLE", Severity.OK, Color.kGray, "Turret en reposo");
    public static final ModuleColorCode MANUAL_CONTROL = ModuleColorCode.solid("MANUAL CONTROL", Severity.OK, Color.kBlue, "Turret en control manual");
    public static final ModuleColorCode LOCKED = ModuleColorCode.solid("LOCKED", Severity.OK, Color.kGreen, "Turret llegó al objetivo");
    public static final ModuleColorCode TRACKING = ModuleColorCode.solid("TRACKING", Severity.OK, Color.kYellow,"Revisando si Turret llegó al objetivo");
    

    @CreateCommand(name = "stop")
    public static class Idle implements TurretRequest {
        
        @Override
        public ActionStatus apply(TurretInputs data, TurretIO actor){
            actor.stop();
            return ActionStatus.of(IDLE, "Idle");
        }
    }

    @CreateCommand (name = "manualControl") // nombre de comando no puede llevar espacios
    public static class Manual implements TurretRequest {

        private DoubleSupplier velocityJoystick;

        public Manual (DoubleSupplier velocityJoystick) {  // como el override solo usa data y actor, añadimos el joystick al contructor
            this.velocityJoystick = velocityJoystick; 
            
        }

        @Override 
        public ActionStatus apply(TurretInputs data, TurretIO actor) {

            if (data.currentAngle.getDegrees() >= -90 && data.currentAngle.getDegrees() <= 90) {

                actor.setSpeed(velocityJoystick.getAsDouble());

            } else {

                actor.setSpeed(0);

            }

            return ActionStatus.of(MANUAL_CONTROL, "Manual Control");

        }
    }

    @CreateCommand (name = "position")
    public static class Position implements TurretRequest {

        private Rotation2d target;
        private double tolerance;

        public Position (Rotation2d target, double tolerance) {
            this.target = target;
            this.tolerance = tolerance;
        }

        @Override
        public ActionStatus apply(TurretInputs data, TurretIO actor) {

            actor.setPosition(target); //mandar a la posición objetivo

                 // Math.abs es para que retorne valor absoluto
            if  (Math.abs(data.currentAngle.getDegrees() - target.getDegrees()) <= tolerance) {  //si la diferencia es menor que la tolerancia, llego al objetivo, si no, sigue
                return ActionStatus.of(LOCKED, "Locked");
            } else {
                return ActionStatus.of(TRACKING, "Tracking");
            }

        }

    }

}
