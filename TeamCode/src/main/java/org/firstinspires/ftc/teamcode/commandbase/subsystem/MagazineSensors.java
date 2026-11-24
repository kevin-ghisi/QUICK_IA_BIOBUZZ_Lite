package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.TimeUtils;

/** Amostra um único sensor REV 2m na boca do intake, somente durante coleta para frente. */
public final class MagazineSensors extends SubsystemBase {
    private final DistanceSensor intakeMouthSensor;
    private final PresenceFilter presenceFilter = new PresenceFilter();
    private double inletDistanceMillimeters = Double.NaN;
    private boolean sampleHealthy;
    private boolean periodicSamplingEnabled = true;
    private boolean samplingActive;
    private boolean previousOccupied;
    private int ballCount;
    private long lastSampleMilliseconds = -1L;

    /** Busca o único sensor pela configuração do Control Hub. */
    public MagazineSensors(HardwareMap hardwareMap) {
        intakeMouthSensor = hardwareMap.get(
                DistanceSensor.class, GlobalConstants.MAGAZINE_SENSOR_NAME);
    }

    /** Inicia sem transações I²C; a primeira leitura ocorre quando a coleta é ativada. */
    public void initialize() {
        samplingActive = false;
        sampleHealthy = false;
        previousOccupied = false;
        ballCount = 0;
        lastSampleMilliseconds = -1L;
        presenceFilter.reset(false, TimeUtils.monotonicMilliseconds());
    }

    /**
     * Faz no máximo uma leitura por intervalo, e somente enquanto o intake está coletando.
     * Retorna true quando uma amostra nova foi feita. Na retomada, descarta a presença antiga e
     * força uma amostra imediata para que o estado corresponda à distância atual.
     */
    @Override
    public void periodic() {
        // O GamePieceCoordinator informa explicitamente se a coleta está ativa.
    }

    public boolean periodic(boolean intakeCollecting) {
        if (!periodicSamplingEnabled || !intakeCollecting) {
            if (samplingActive) {
                samplingActive = false;
                sampleHealthy = false;
            }
            return false;
        }

        long nowMillis = TimeUtils.monotonicMilliseconds();
        if (!samplingActive) {
            samplingActive = true;
            sampleHealthy = false;
            presenceFilter.reset(false, nowMillis);
            previousOccupied = false;
            lastSampleMilliseconds = nowMillis
                    - GlobalConstants.MAGAZINE_SENSOR_SAMPLE_INTERVAL_MILLISECONDS;
        }
        if (nowMillis - lastSampleMilliseconds
                < GlobalConstants.MAGAZINE_SENSOR_SAMPLE_INTERVAL_MILLISECONDS) return false;

        lastSampleMilliseconds = nowMillis;
        inletDistanceMillimeters = intakeMouthSensor.getDistance(DistanceUnit.MM);
        sampleHealthy = isUsableDistance(inletDistanceMillimeters);
        if (sampleHealthy) {
            boolean rawOccupied = presenceFilter.isPresent()
                    ? inletDistanceMillimeters <= GlobalConstants.MAGAZINE_CLEAR_DISTANCE_MM
                    : inletDistanceMillimeters <= GlobalConstants.MAGAZINE_OCCUPIED_DISTANCE_MM;
            presenceFilter.update(rawOccupied, nowMillis);
            boolean occupied = presenceFilter.isPresent();
            if (occupied && !previousOccupied) {
                ballCount = Math.min(GlobalConstants.MAGAZINE_CAPACITY, ballCount + 1);
            }
            previousOccupied = occupied;
        }
        return true;
    }

    /** Liga/desliga a amostragem periódica; as leituras permanecem no cache quando pausadas. */
    public void setPeriodicSamplingEnabled(boolean enabled) {
        periodicSamplingEnabled = enabled;
    }

    /** Recusa NaN, infinito e distâncias além do alcance nominal de 2 metros. */
    private boolean isUsableDistance(double distanceMillimeters) {
        return Double.isFinite(distanceMillimeters)
                && distanceMillimeters >= 0.0
                && distanceMillimeters <= 2000.0;
    }

    /** Informa se a última amostra é válida; false também significa que a leitura está pausada. */
    public boolean areSensorsHealthy() {
        return samplingActive && sampleHealthy;
    }

    public boolean isSamplingActive() {
        return samplingActive;
    }

    /** Indica obstrução estável na boca do intake, usada como limite de capacidade. */
    public boolean isInletOccupied() {
        return presenceFilter.isPresent();
    }

    public boolean isMagazineFull() {
        return ballCount >= GlobalConstants.MAGAZINE_CAPACITY;
    }

    /** Mantido para compatibilidade com subsistemas de tiro; agora o critério é um único sensor. */
    public boolean isFullEstimate() {
        return isMagazineFull();
    }

    /** Resume o estado conhecido do único sensor, sem realizar nova leitura. */
    public String getLoadEstimate() {
        if (isMagazineFull()) return ballCount + "/" + GlobalConstants.MAGAZINE_CAPACITY + " (CHEIO)";
        if (!samplingActive) return ballCount + "/" + GlobalConstants.MAGAZINE_CAPACITY + " (pausado)";
        if (!sampleHealthy) return "SENSOR INVÁLIDO";
        return ballCount + "/" + GlobalConstants.MAGAZINE_CAPACITY;
    }

    /** Número de passagens estáveis contadas no sensor único da entrada da rampa. */
    public int getBallCount() {
        return ballCount;
    }

    /** Zera a contagem depois de descarregar ou completar uma sequência de tiro. */
    public void resetBallCount() {
        ballCount = 0;
    }

    /** Distância atual do sensor da entrada em milímetros para calibrar o limiar. */
    public double getInletDistanceMillimeters() {
        return inletDistanceMillimeters;
    }

    /** Informa a idade da última amostragem, útil para telemetria e diagnóstico do cache. */
    public long getSampleAgeMilliseconds() {
        return lastSampleMilliseconds < 0L ? -1L
                : Math.max(0L, TimeUtils.monotonicMilliseconds() - lastSampleMilliseconds);
    }

    /** Filtro temporal com dois limiares para reduzir leituras instáveis em torno da bola. */
    private static final class PresenceFilter {
        private boolean present;
        private boolean candidate;
        private long candidateSinceMillis;

        private void reset(boolean initialPresent, long nowMillis) {
            present = initialPresent;
            candidate = initialPresent;
            candidateSinceMillis = nowMillis;
        }

        /** Aceita uma mudança somente depois que ela permaneceu estável pelo tempo configurado. */
        private void update(boolean rawPresent, long nowMillis) {
            if (rawPresent != candidate) {
                candidate = rawPresent;
                candidateSinceMillis = nowMillis;
            }
            long stableForMillis = nowMillis - candidateSinceMillis;
            if (stableForMillis >= GlobalConstants.MAGAZINE_SENSOR_DEBOUNCE_MILLISECONDS) {
                present = candidate;
            }
        }

        /** Devolve o estado filtrado da presença. */
        private boolean isPresent() {
            return present;
        }
    }
}
