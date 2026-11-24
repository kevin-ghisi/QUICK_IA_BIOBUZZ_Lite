package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.hardware.ShooterTuningConfig;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/** Shooter com motor, encoder e controle PIDF executados pela SolversLib no loop do robô. */
public final class Shooter extends SubsystemBase {
    public enum State { FORWARD, REVERSE, STOP }

    private final MotorEx motor;
    private final PIDFController velocityController = new PIDFController(0.11, 0, 0, 0.001);
    private State state = State.STOP;
    private double appliedVelocityTargetTicksPerSecond;
    private double cachedKp = Double.NaN, cachedKi = Double.NaN;
    private double cachedKd = Double.NaN, cachedKf = Double.NaN;
    private boolean cachedCustomPidf;
    private boolean hasCachedConfiguration;

    /** Constrói o shooter com hardware SolversLib e deixa o encoder disponível ao PIDF. */
    public Shooter(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, GlobalConstants.SHOOTER_MOTOR_NAME);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setInverted(GlobalConstants.SHOOTER_MOTOR_REVERSED);
        motor.setZeroPowerBehavior(GlobalConstants.SHOOTER_ZERO_POWER);
    }

    /** Inicia parado e limpa memória integral/derivativa do controlador. */
    public void initialize() {
        velocityController.reset();
        setState(State.STOP);
    }

    /** Seleciona rotação direta, reversa ou parada; a malha de velocidade só atua para frente. */
    public void setState(State newState) {
        state = newState;
        if (newState != State.FORWARD) {
            motor.set(newState == State.REVERSE ? -GlobalConstants.SHOOTER_POWER : 0.0);
            velocityController.reset();
        }
    }

    public State getState() { return state; }

    /** Recalcula a potência em cada ciclo do Scheduler para manter a velocidade alvo. */
    @Override
    public void periodic() {
        if (state == State.FORWARD) {
            applyTuning(ShooterTuningConfig.targetVelocityTicksPerSecond,
                    ShooterTuningConfig.useCustomPidf, ShooterTuningConfig.kP,
                    ShooterTuningConfig.kI, ShooterTuningConfig.kD, ShooterTuningConfig.kF);
        }
    }

    /** Atualiza ao vivo os ganhos do Panels e calcula potência usando o PIDF da SolversLib. */
    public void applyTuning(double targetTicksPerSecond, boolean useCustomPidf,
                            double kP, double kI, double kD, double kF) {
        appliedVelocityTargetTicksPerSecond = Math.max(0.0, targetTicksPerSecond);
        if (!hasCachedConfiguration || cachedCustomPidf != useCustomPidf
                || Double.compare(cachedKp, kP) != 0 || Double.compare(cachedKi, kI) != 0
                || Double.compare(cachedKd, kD) != 0 || Double.compare(cachedKf, kF) != 0) {
            velocityController.setPIDF(useCustomPidf ? kP : 0.0, useCustomPidf ? kI : 0.0,
                    useCustomPidf ? kD : 0.0, useCustomPidf ? kF : 0.0);
            cachedKp = kP; cachedKi = kI; cachedKd = kD; cachedKf = kF;
            cachedCustomPidf = useCustomPidf;
            hasCachedConfiguration = true;
            velocityController.reset();
        }
        if (state != State.FORWARD) return;

        if (appliedVelocityTargetTicksPerSecond <= 0.0) {
            motor.set(GlobalConstants.SHOOTER_POWER);
            velocityController.reset();
            return;
        }
        if (!useCustomPidf) {
            // Sem PID, usa a fração de velocidade nominal do wrapper como referência aberta.
            motor.set(MathUtils.clamp(appliedVelocityTargetTicksPerSecond
                    / motor.ACHIEVABLE_MAX_TICKS_PER_SECOND, 0.0, 1.0));
            velocityController.reset();
            return;
        }
        double power = velocityController.calculate(motor.getVelocity(), appliedVelocityTargetTicksPerSecond);
        motor.set(MathUtils.clamp(power, 0.0, 1.0));
    }

    public double getAppliedVelocityTargetTicksPerSecond() { return appliedVelocityTargetTicksPerSecond; }
    public double getVelocityTicksPerSecond() { return motor.getVelocity(); }

    /** Encerra o modo de bancada com potência zero. */
    public void stop() { setState(State.STOP); }

}
