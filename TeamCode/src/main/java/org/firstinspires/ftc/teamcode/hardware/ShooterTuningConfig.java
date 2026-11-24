package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Variáveis ao vivo do modo de bancada do shooter, publicadas no Panels.
 * A velocidade é ticks/segundo do encoder. Os ganhos agora produzem potência normalizada
 * [-1, 1] no PIDF da SolversLib (não são os coeficientes internos do Hub REV).
 */
@Configurable
public class ShooterTuningConfig {
    public static double targetVelocityTicksPerSecond = 800;
    public static boolean useCustomPidf = true;
    public static double kP = 0.11;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.001; //Ultimo valor Testado, ainda nao finalizado: 0.011

    private ShooterTuningConfig() { }
}
