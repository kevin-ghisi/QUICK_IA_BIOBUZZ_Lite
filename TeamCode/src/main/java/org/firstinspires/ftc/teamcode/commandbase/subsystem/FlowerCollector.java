package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/** Dois servos espelhados que recolhem ou implantam a pá usada para coletar flores. */
public final class FlowerCollector extends SubsystemBase {
    private final ServoEx leftServo;
    private final ServoEx rightServo;
    private boolean deployed;

    /** Cria os servos esquerdo e direito usando os nomes do Control Hub. */
    public FlowerCollector(HardwareMap hardwareMap) {
        leftServo = new ServoEx(hardwareMap, GlobalConstants.FLOWER_COLLECTOR_LEFT_SERVO_NAME);
        rightServo = new ServoEx(hardwareMap, GlobalConstants.FLOWER_COLLECTOR_RIGHT_SERVO_NAME);
    }

    /** Recolhe a pá ao iniciar para evitar colisão durante a seleção do OpMode. */
    public void initialize() {
        retract();
    }

    /** Implanta a pá para a coleta de flores. */
    public void deploy() {
        setPositions(GlobalConstants.FLOWER_COLLECTOR_LEFT_DEPLOYED_POSITION,
                     GlobalConstants.FLOWER_COLLECTOR_RIGHT_DEPLOYED_POSITION);
        deployed = true;
    }

    /** Recolhe a pá para deslocamento e preparação do disparo. */
    public void retract() {
        setPositions(GlobalConstants.FLOWER_COLLECTOR_LEFT_STOWED_POSITION,
                GlobalConstants.FLOWER_COLLECTOR_RIGHT_STOWED_POSITION);
        deployed = false;
    }

    /** Limita e envia posições independentes para os servos esquerdo e direito. */
    public void setPositions(double leftPosition, double rightPosition) {
        leftServo.set(MathUtils.clamp(leftPosition, 0.0, 1.0));
        rightServo.set(MathUtils.clamp(rightPosition, 0.0, 1.0));
    }

    /** Informa se o software comandou a pá para a posição implantada. */
    public boolean isDeployed() {
        return deployed;
    }

    /** Lê a última posição normalizada reportada pelo servo esquerdo. */
    public double getPosition() {
        return leftServo.get();
    }

    public double getLeftPosition() {
        return leftServo.get();
    }

    public double getRightPosition() {
        return rightServo.get();
    }
}
