package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/** Servo da trava que retém as bolas até o início efetivo do disparo. */
public final class BallGate extends SubsystemBase {
    private final ServoEx servo;
    private boolean locked = true;

    /** Cria o servo da trava usando o nome configurado no Control Hub. */
    public BallGate(HardwareMap hardwareMap) {
        servo = new ServoEx(hardwareMap, GlobalConstants.BALL_GATE_SERVO_NAME);
    }

    /** Fecha a trava ao iniciar cada OpMode, mantendo as bolas fora do shooter. */
    public void initialize() {
        lock();
    }

    /** Fecha a passagem antes de parar a alimentação das bolas. */
    public void lock() {
        setPosition(GlobalConstants.BALL_GATE_LOCKED_POSITION);
        locked = true;
    }

    /** Abre a passagem; deve ser chamado somente quando o shooter estiver preparado. */
    public void unlock() {
        setPosition(GlobalConstants.BALL_GATE_OPEN_POSITION);
        locked = false;
    }

    /** Limita e envia a posição normalizada do servo. */
    public void setPosition(double position) {
        servo.set(MathUtils.clamp(position, 0.0, 1.0));
    }

    /** Informa se o software comandou a posição de retenção. */
    public boolean isLocked() {
        return locked;
    }

    /** Lê a última posição normalizada reportada pelo servo. */
    public double getPosition() {
        return servo.get();
    }
}
