package frc.tests;


import com.stzteam.mars.test.MARSTest;
import com.stzteam.mars.test.TestRoutine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Modulos.Turret;
import frc.robot.Requests.TurretRequestFactory;

@MARSTest (name = "Turret Movement Test")
public class TurretTest extends TestRoutine{

    private final Turret tur;

    public TurretTest(Turret turret) {
       this.tur = turret;
    }

    @Override
    public Command getRoutineCommand(){
        return Commands.sequence(
            run(
                TurretRequestFactory.position(Rotation2d.fromDegrees(45),2.0) , tur),

            waitFor(() -> tur.isAtTarget(2.0), 2),
            
            assertLessThan(
            () -> Math.abs(tur.getState().currentAngle.getRadians() - Math.toRadians(45)), 2.0,
            "High turret error on target 1"
            ),

            run(
                TurretRequestFactory.position(Rotation2d.fromDegrees(-45), 2.0), tur),

            waitFor(() -> tur.isAtTarget(2.0), 2),

            assertLessThan(
            () -> Math.abs(tur.getState().currentAngle.getRadians() - Math.toRadians(-45)), 2.0,
            "High turret error on target 2"
            ),

            run(
                TurretRequestFactory.position(Rotation2d.fromDegrees(0), 2.0), tur),

            waitFor(() -> tur.isAtTarget(2.0), 2),

            assertLessThan(
            () -> Math.abs(tur.getState().currentAngle.getRadians()), 2.0,
            "High turret error on target 3"
            ),

            run(
                TurretRequestFactory.idle(), tur)

            ); 
    }
    
    
}
