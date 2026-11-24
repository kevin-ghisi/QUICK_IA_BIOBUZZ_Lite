package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.TimeUtils;

/** Orquestra coleta, pré-aceleração e disparo sem duplicar a lógica dos mecanismos. */
public final class GamePieceCoordinator extends SubsystemBase {
    /** Fases do fluxo de bolas e flores durante a partida. */
    public enum State {
        IDLE, COLLECTING_FLOOR, FLOWER_DEPLOY_DELAY, COLLECTING_FLOWER,
        MAGAZINE_FULL, READY_TO_SHOOT, FIRING, COMPLETE
    }

    private final Intake intake;
    private final Shooter shooter;
    private final FlowerCollector collector;
    private final BallGate gate;
    private final Turret turret;
    private final MagazineSensors magazine;
    private final Transfer transfer;
    private State state = State.IDLE;
    private long firingStartedAtMillis;
    private long flowerDeployStartedAtMillis;
    private long shooterStartedAtMillis;
    private boolean magazineFullStopEnabled = true;

    /** Recebe os subsistemas que controla; não cria nem busca hardware adicional. */
    public GamePieceCoordinator(
            Intake intake,
            Shooter shooter,
            FlowerCollector collector,
            BallGate gate,
            Turret turret,
            MagazineSensors magazine,
            Transfer transfer) {
                this.intake = intake;
                this.shooter = shooter;
                this.collector = collector;
                this.gate = gate;
                this.turret = turret;
                this.magazine = magazine;
                this.transfer = transfer;
    }

    /** Inicia o OpMode em condição protegida: trava fechada e motores parados. */
    public void initialize() {
        gate.lock();
        collector.retract();
        intake.setState(Intake.State.STOP);
        transfer.setState(Transfer.State.STOP);
        shooter.setState(Shooter.State.STOP);
        shooterStartedAtMillis = 0L;
        state = State.IDLE;
    }

    /** Coleta normal do chão: mantém a pá guardada e liga intake e pré-aceleração. */
    public void startFloorCollection() {
        gate.lock();
        collector.retract();
        transfer.setState(Transfer.State.REVERSE);
        intake.setState(Intake.State.FORWARD);
        ensureShooterRunning();
        state = State.COLLECTING_FLOOR;
    }

    /** Inicia intake e transfer do TeleOp sem ligar shooter nem pá coletora. */
    public void startManualIntake() {
        if (state == State.FIRING) return;
        intake.setState(Intake.State.FORWARD);
        transfer.setState(Transfer.State.REVERSE);
        state = State.COLLECTING_FLOOR;
    }

    /** Inverte intake e transfer manualmente; a proteção de capacidade só vale para frente. */
    public void reverseManualIntake() {
        if (state == State.FIRING) return;
        magazine.resetBallCount();
        intake.setState(Intake.State.REVERSE);
        transfer.setState(Transfer.State.FORWARD);
        state = State.IDLE;
    }

    /** Para intake e transfer manualmente sem alterar o estado do shooter. */
    public void stopManualIntake() {
        intake.setState(Intake.State.STOP);
        transfer.setState(Transfer.State.STOP);
        if (state != State.FIRING) state = State.IDLE;
    }

    /** Permite desativar a parada por sensor em um OpMode específico, mantendo-a ativa por padrão. */
    public void setMagazineFullStopEnabled(boolean enabled) {
        magazineFullStopEnabled = enabled;
    }

    /** Liga intake primeiro e implanta a pá após um intervalo curto para coletar flower. */
    public void startFlowerCollection() {
        gate.lock();
        collector.retract();
        intake.setState(Intake.State.FORWARD);
        ensureShooterRunning();
        flowerDeployStartedAtMillis = TimeUtils.monotonicMilliseconds();
        state = State.FLOWER_DEPLOY_DELAY;
    }

    /**
     * Prepara o disparo: recolhe a pá e fecha a trava enquanto o shooter continua acelerado.
     * Se a última leitura válida indica obstrução na boca, mantém o intake parado.
     */
    public void prepareShot() {
        if (!isCollecting() && state != State.MAGAZINE_FULL
                && state != State.IDLE && state != State.COMPLETE) return;
        gate.lock();
        collector.retract();
        ensureShooterRunning();
        intake.setState(magazine.isFullEstimate() ? Intake.State.STOP : Intake.State.FORWARD);
        state = State.READY_TO_SHOOT;
    }

    /** Liga o shooter sem reiniciar o relógio se ele já estiver pré-acelerando. */
    private void ensureShooterRunning() {
        if (shooter.getState() != Shooter.State.FORWARD) {
            shooterStartedAtMillis = TimeUtils.monotonicMilliseconds();
            shooter.setState(Shooter.State.FORWARD);
        }
    }

    /** Abre a trava e começa a alimentar as bolas somente depois da fase READY_TO_SHOOT. */
    public void startFiring() {
        if (state != State.READY_TO_SHOOT) return;
        if (!isReadyToShoot()) return;
        // No autônomo, não dispare se a torre não tiver uma referência física calibrada.
        if (GlobalConstants.robotMode == GlobalConstants.RobotMode.AUTONOMOUS
                && !turret.isForwardZeroCalibrated()) return;
        // Se a mira automática está ligada, não solte a bola antes da torre alcançar o alvo.
        if (turret.isAutoAimEnabled() && !turret.isAtTarget()) return;
        shooter.setState(Shooter.State.FORWARD);
        intake.setState(Intake.State.FORWARD);
        gate.unlock();
        firingStartedAtMillis = TimeUtils.monotonicMilliseconds();
        state = State.FIRING;
    }

    /** Fecha a trava primeiro e então encerra a alimentação ao fim da janela de disparo. */
    public void finishFiring() {
        if (state != State.FIRING) return;
        gate.lock();
        intake.setState(Intake.State.STOP);
        transfer.setState(Transfer.State.STOP);
        shooter.setState(Shooter.State.STOP);
        collector.retract();
        magazine.resetBallCount();
        state = State.COMPLETE;
    }

    /** Interrompe qualquer fase em condição segura; usado pelo botão de cancelamento. */
    public void cancelToSafe() {
        gate.lock();
        intake.setState(Intake.State.STOP);
        transfer.setState(Transfer.State.STOP);
        shooter.setState(Shooter.State.STOP);
        collector.retract();
        shooterStartedAtMillis = 0L;
        state = State.IDLE;
    }

    public void startShooterManual() {
        shooter.setState(Shooter.State.FORWARD);
    }

    /** Atualiza o filtro da boca e interrompe intake/transfer ao confirmar capacidade atingida. */
    @Override
    public void periodic() {
        long nowMillis = TimeUtils.monotonicMilliseconds();
        boolean collecting = isCollecting() && intake.getState() == Intake.State.FORWARD;
        magazine.periodic(collecting);
        if (state == State.FLOWER_DEPLOY_DELAY) {
            if (nowMillis - flowerDeployStartedAtMillis
                    >= GlobalConstants.FLOWER_COLLECTOR_DEPLOY_DELAY_MILLISECONDS) {
                if (magazineFullStopEnabled && magazine.isFullEstimate()) {
                    markMagazineFull();
                } else {
                    collector.deploy();
                    state = State.COLLECTING_FLOWER;
                }
            }
        }

        if (magazineFullStopEnabled && collecting && magazine.isMagazineFull()) markMagazineFull();

        if (state == State.FIRING) {
            long elapsedMillis = nowMillis - firingStartedAtMillis;
            if (elapsedMillis >= GlobalConstants.SHOT_FEED_DURATION_MILLISECONDS) finishFiring();
        }
    }

    /** Para intake e transfer e recolhe a pá após a obstrução persistir pelo debounce configurado. */
    private void markMagazineFull() {
        intake.setState(Intake.State.STOP);
        transfer.setState(Transfer.State.STOP);
        collector.retract();
        state = State.MAGAZINE_FULL;
    }

    /** Agrupa fases que ainda estão coletando e devem reagir ao limite do carregador. */
    private boolean isCollecting() {
        return state == State.COLLECTING_FLOOR
                || state == State.FLOWER_DEPLOY_DELAY
                || state == State.COLLECTING_FLOWER;
    }

    /** Informa se shooter, turret e fase permitem abrir a trava com segurança. */
    public boolean isReadyToShoot() {
        if (state != State.READY_TO_SHOOT || shooter.getState() != Shooter.State.FORWARD) return false;
        long spinupElapsedMillis = TimeUtils.monotonicMilliseconds() - shooterStartedAtMillis;
        if (spinupElapsedMillis < GlobalConstants.SHOOTER_SPINUP_ESTIMATE_MILLISECONDS) return false;
        if (GlobalConstants.robotMode == GlobalConstants.RobotMode.AUTONOMOUS
                && !turret.isForwardZeroCalibrated()) return false;
        return !turret.isAutoAimEnabled() || turret.isAtTarget();
    }

    /** Informa se a fase atual foi iniciada para buscar flower com a pá implantada. */
    public boolean isFlowerCollection() {
        return state == State.FLOWER_DEPLOY_DELAY || state == State.COLLECTING_FLOWER;
    }

    /** Devolve a fase atual para orientar operador e depuração. */
    public State getState() {
        return state;
    }
}
