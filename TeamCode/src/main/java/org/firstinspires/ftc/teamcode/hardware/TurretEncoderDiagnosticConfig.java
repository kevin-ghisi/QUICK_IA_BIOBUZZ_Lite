package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;

/** Valores ajustáveis pelo Panels durante a conferência do encoder externo da torre. */
@Configurable
public final class TurretEncoderDiagnosticConfig {
    /**
     * Calibrado inicialmente: 90 graus físicos deram aproximadamente 220 graus na leitura
     * (8192 * 220 / 90). Refine após repetir a medição com transferidor.
     */
    public static double countsPerTurretRevolution = GlobalConstants.TURRET_TICKS_PER_OUTPUT_REV;
    /** Sinal compensando a inversão configurada no dispositivo intake usado como canal encoder. */
    public static double encoderSign = GlobalConstants.TURRET_THROUGH_BORE_ENCODER_SIGN;
    /** Ângulo lido manualmente com transferidor, relativo à frente do robô. */
    public static double measuredPhysicalAngleDegrees;

    private TurretEncoderDiagnosticConfig() { }
}
