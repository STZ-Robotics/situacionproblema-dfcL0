package frc.robot.Interfaces;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;

import edu.wpi.first.math.geometry.Rotation2d;

@Fallback
public interface TurretIO extends IO<TurretIO.TurretInputs> {

    public class TurretInputs extends Data<TurretInputs>{
        
        public Rotation2d currentAngle;
        public Rotation2d targetAngle;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;

        public TurretInputs snapshot() {
            TurretInputs copy = new TurretInputs();

            copy.appliedVolts = appliedVolts;
            copy.current = current;

            return copy;
        }

    }

    public void setVoltage(double appliedVolts);

    public void setPosition(Rotation2d targetAngle);

    public void setDutyCycle(double percentage);

    public void setPositionWithFF(Rotation2d targetAngle, double velocityRPS);

    public void setSpeed(double speed);

    public void stop();

    public void resetEnc();
    
}
