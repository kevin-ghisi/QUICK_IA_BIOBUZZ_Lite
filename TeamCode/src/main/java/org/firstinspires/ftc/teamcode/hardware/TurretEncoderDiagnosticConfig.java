package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;

/** Valores ajustáveis pelo Panels durante a conferência do encoder externo da torre. */
@Configurable
public class TurretEncoderDiagnosticConfig {
    /** Estimativa inicial pela relação 85:35; refine após medir a torre com transferidor. */
    public static double countsPerTurretRevolution =
            GlobalConstants.TURRET_THROUGH_BORE_TICKS_PER_OUTPUT_REV;
    /** Sinal compensando a inversão configurada no dispositivo intake usado como canal encoder. */
    public static double encoderSign = GlobalConstants.TURRET_THROUGH_BORE_ENCODER_SIGN;
    /** Ângulo lido manualmente com transferidor, relativo à frente do robô. */
    public static double measuredPhysicalAngleDegrees;

    private TurretEncoderDiagnosticConfig() { }
}
