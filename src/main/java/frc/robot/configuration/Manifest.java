package frc.robot.configuration;

import com.stzteam.mars.builder.Environment;
import com.stzteam.mars.builder.Injector;
import com.stzteam.mars.builder.Environment.RunMode;

import frc.robot.Interfaces.TurretIO;
import frc.robot.Interfaces.TurretIOFallback;
import frc.robot.Interfaces.TurretIOSim;
import frc.robot.Interfaces.TurretIOSparkMax;
import frc.robot.Modulos.Turret;

public class Manifest {

    public static final RunMode CURRENT_MODE = RunMode.REAL; //definir si usar motores reales o simulación


    //los subsistemas que tenemos
    public static final boolean HAS_TURRET = true;

    static{Environment.setMode(CURRENT_MODE);}


    //Injector decide que implementación se usa: nuestro motor real (spark o kraken), nuetsra simulación o el fallback 
    public static Turret buildTurret() {
        TurretIO io = Injector.createIO(HAS_TURRET, TurretIOFallback::new, TurretIOSparkMax::new, TurretIOSim::new);
        return new Turret(io);
    }

    //Injector.createIO automaticamente checa el entorno en el que estas corriendo el código y decide

}
