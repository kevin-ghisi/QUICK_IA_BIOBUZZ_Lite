package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;

/** Ajustes em tempo real do rastreamento de alvo da torre no Panels. */
@Configurable
public class TurretTrackingConfig {
    /** Antecipação da pose para reduzir o atraso percebido em giros; zero desativa a previsão. */
    public static double predictionHorizonSeconds = 0.12;
    /** Faixa ao redor da linha central em que o alvo não troca para evitar alternância por ruído. */
    public static double midlineHysteresisInches = 4.0;

    private TurretTrackingConfig() { }
}
