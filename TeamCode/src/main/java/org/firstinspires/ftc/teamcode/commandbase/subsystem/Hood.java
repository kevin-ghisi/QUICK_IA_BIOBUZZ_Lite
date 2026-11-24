package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/** Servo posicional do hood; a curva de distância precisa ser calibrada com disparos reais. */
public final class Hood extends SubsystemBase {
    private final ServoEx servo;

    /** Cria o controle do servo usando seu nome na configuração do Control Hub. */
    public Hood(HardwareMap hardwareMap) {
        servo = new ServoEx(hardwareMap, GlobalConstants.HOOD_SERVO_NAME);
    }

    /** Posiciona o hood em um ponto calibrado ao iniciar o OpMode. */
    public void initialize() {
        setPosition(GlobalConstants.HOOD_SERVO_POSITIONS[0]);
    }

    /** Usa a tabela empírica distância/posição para escolher a parábola. */
    public void setForDistance(double distanceInches) {
        setPosition(GlobalConstants.hoodPositionForDistance(distanceInches));
    }

    /** Aplica posição normalizada limitada à faixa mecânica configurada. */
    public void setPosition(double position) {
        double safePosition = MathUtils.clamp(
                position, GlobalConstants.HOOD_MIN_POSITION, GlobalConstants.HOOD_MAX_POSITION);
        servo.set(safePosition);
    }

    /** Move o servo por um incremento relativo à posição atual. */
    public void adjust(double delta) {
        setPosition(servo.get() + delta);
    }

    /** Reduz a posição do servo em um passo de calibração. */
    public void adjustTowardLowerAngle() {
        adjust(-GlobalConstants.HOOD_MANUAL_STEP);
    }

    /** Aumenta a posição do servo em um passo de calibração. */
    public void adjustTowardHigherAngle() {
        adjust(GlobalConstants.HOOD_MANUAL_STEP);
    }

    /** Lê a posição normalizada informada pelo servo. */
    public double getPosition() {
        return servo.get();
    }
}
