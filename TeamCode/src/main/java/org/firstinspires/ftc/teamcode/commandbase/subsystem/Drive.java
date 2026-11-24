package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;

/** Chassi Mecanum e localização Pinpoint, pertencentes a uma instância de MyRobot. */
public final class Drive extends SubsystemBase {
    private final Follower follower;

    /** Cria o seguidor Pedro com o hardware e as configurações do projeto. */
    public Drive(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    /** Atualiza a pose antes de iniciar comandos que dependem da localização. */
    public void initialize() {
        follower.update();
    }

    /** Envia potência robot-centric: frente, strafe lateral e rotação do próprio chassi. */
    public void driveManual(DrivePowers powers) {
        follower.manual(powers);
    }

    /** Cria um comando Pedro para seguir o caminho informado. */
    public FollowPathCommand followPath(Path path) {
        return new FollowPathCommand(follower, path);
    }

    /** Cria um comando para seguir o caminho e opcionalmente manter sua pose final. */
    public FollowPathCommand followPath(Path path, boolean holdEnd) {
        return new FollowPathCommand(follower, path, holdEnd);
    }

    /** Cria um comando de caminho com limite de potência. */
    public FollowPathCommand followPath(Path path, double maxPower) {
        return new FollowPathCommand(follower, path, maxPower);
    }

    /** Cria um comando de caminho configurando retenção final e potência máxima. */
    public FollowPathCommand followPath(Path path, boolean holdEnd, double maxPower) {
        return new FollowPathCommand(follower, path, holdEnd, maxPower);
    }

    /** Zera o comando manual e atualiza o seguidor para parar o chassi. */
    public void stop() {
        follower.manual(0, 0, 0);
        follower.update();
    }

    /** Retorna a pose estimada pelo localizador Pinpoint/Pedro, em polegadas e radianos. */
    public Pose getPose() {
        return follower.pose();
    }

    /** Define uma pose inicial conhecida no localizador. */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /** Zera somente o heading estimado, mantendo X e Y atuais. */
    public void resetHeading() {
        Pose currentPose = follower.pose();
        follower.setPose(new Pose(currentPose.x(), currentPose.y(), 0));
    }

    public void resetPose() {
        Pose currentPose = GlobalConstants.AUTO_START_POSE;
        follower.setPose(currentPose);
    }

    /** Atualiza localização e controle de caminho uma vez por ciclo do scheduler. */
    @Override
    public void periodic() {
        follower.update();
    }
}
