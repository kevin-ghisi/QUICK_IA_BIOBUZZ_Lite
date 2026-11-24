package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Parâmetros que o Panels altera durante o teste de posição da torre.
 * Java exige campos public static não-final para o painel escrever os novos valores.
 */
@Configurable
public class TurretTuningConfig {
    public static double targetAngleDegrees = 0.0;
    public static double kP = 0.011;
    public static double kI = 0.0;
    public static double kD = 0.00085;
    /** Potência mínima opcional para superar o atrito; zero mantém a saída contínua do PID. */
    public static double minPower = 0.10;
    public static double maxPower = 0.7;
    public static double integralLimitDegreesSeconds = 100.0;
    public static double toleranceDegrees = 2.0;

    private TurretTuningConfig() { }
}
