package frc.robot.Bindings;

import com.stzteam.mars.models.containers.Binding;
import com.stzteam.mars.operator.ControllerOI;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Modulos.Turret;
import frc.robot.Requests.TurretRequestFactory;

public class OperatorBindings implements Binding {

    private final ControllerOI operator;
    private final Turret turret;

    private OperatorBindings (ControllerOI operator, Turret turret) {
        this.operator = operator;
        this.turret = turret;
    }

    public static OperatorBindings create(ControllerOI operator, Turret turret) {
        return new OperatorBindings(operator, turret);
    }

    @Override
    public void bind() {

        var padButtons = operator.getDPadTriggers();
        var leftStick = operator.getLeftStick();


        new Trigger(() -> Math.abs(leftStick.x().getAsDouble()) > 0.1)
         .and(padButtons.right())
         .whileTrue(
             turret.setControl(() -> TurretRequestFactory.manual(leftStick.x()))
        );

    }
    
}
