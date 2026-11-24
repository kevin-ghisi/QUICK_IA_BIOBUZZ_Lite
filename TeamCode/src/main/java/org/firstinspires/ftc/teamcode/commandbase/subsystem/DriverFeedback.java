package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;

/** Converte a fase do robô e a estimativa do carregador em padrões do REV Blinkin. */
public final class DriverFeedback extends SubsystemBase {
    private final RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern lastPattern;

    /** Busca o Blinkin pelo nome configurado no Control Hub. */
    public DriverFeedback(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, GlobalConstants.BLINKIN_LED_NAME);
    }

    /** Apaga a fita ao iniciar para que o primeiro estado seja aplicado pelo OpMode. */
    public void initialize() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    /** Escolhe uma cor/padrão por prioridade: pronto, cheio, disparo, coleta e vazio. */
    public void update(GamePieceCoordinator.State state, MagazineSensors magazine,
                       boolean readyToShoot, boolean flowerCollection) {
        RevBlinkinLedDriver.BlinkinPattern nextPattern;
        if (readyToShoot) {
            // A enumeração do SDK inclui GREEN; não existe STROBE_GREEN nesta versão.
            nextPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (magazine.isFullEstimate()) {
            nextPattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
        } else if (state == GamePieceCoordinator.State.FIRING) {
            nextPattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
        } else if (magazine.isSamplingActive() && !magazine.areSensorsHealthy()) {
            nextPattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
        } else if (flowerCollection) {
            nextPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
        } else if (state == GamePieceCoordinator.State.COLLECTING_FLOOR
                || state == GamePieceCoordinator.State.FLOWER_DEPLOY_DELAY
                || state == GamePieceCoordinator.State.COLLECTING_FLOWER
                || state == GamePieceCoordinator.State.MAGAZINE_FULL) {
            nextPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else if (magazine.isInletOccupied()) {
            nextPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        } else {
            nextPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        }
        setPattern(nextPattern);
    }

    /** Só envia uma atualização I2C/PWM quando o padrão visual realmente muda. */
    private void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (pattern != lastPattern) {
            blinkin.setPattern(pattern);
            lastPattern = pattern;
        }
    }
}
