package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;

public final class Intake extends SubsystemBase {
    /** Estados elétricos possíveis do motor de intake. */
    public enum State {
        FORWARD,
        REVERSE,
        STOP
    }

    private final Motor motor;
    private State state = State.STOP;

    /** Obtém o motor e aplica o modo de potência e frenagem configurados. */
    public Intake(HardwareMap hardwareMap) {
        motor = new Motor(hardwareMap, GlobalConstants.INTAKE_MOTOR_NAME);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setInverted(true);
        motor.setZeroPowerBehavior(GlobalConstants.INTAKE_ZERO_POWER);
    }

    /** Coloca o intake em estado seguro ao iniciar cada OpMode. */
    public void initialize() {
        setState(State.STOP);
    }

    /** Aplica um dos três estados; a inversão usa a mesma potência com sinal oposto. */
    public void setState(State newState) {
        state = newState;
        switch (newState) {
            case FORWARD:
                motor.set(GlobalConstants.INTAKE_POWER);
                break;
            case REVERSE:
                motor.set(-GlobalConstants.INTAKE_POWER);
                break;
            case STOP:
            default:
                motor.set(0);
                break;
        }
    }

    /** Alterna entre ligado para frente e parado, útil para um botão de toggle. */
    public void toggle() {
        setState(state == State.FORWARD ? State.STOP : State.FORWARD);
    }

    /** Informa o último estado solicitado ao motor. */
    public State getState() {
        return state;
    }
}
