package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.hardware.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.hardware.MyRobot;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/** Autônomo enxuto: percorre a rota de pontuação existente e entrega a pose ao TeleOp. */
@Autonomous(name = "BIOBUZZ Lite - Auto", group = "Autonomous")
public class Auto_PoseHandoff extends CommandOpMode {
    private MyRobot robot;
    private boolean blueWasPressed;
    private boolean redWasPressed;
    private boolean started;

    @Override
    public void initialize() {
        super.reset();
        GlobalConstants.robotMode = GlobalConstants.RobotMode.AUTONOMOUS;
        robot = new MyRobot(hardwareMap);
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.dpad_left && !blueWasPressed) GlobalConstants.alliance = GlobalConstants.Alliance.BLUE;
        if (gamepad1.dpad_right && !redWasPressed) GlobalConstants.alliance = GlobalConstants.Alliance.RED;
        blueWasPressed = gamepad1.dpad_left;
        redWasPressed = gamepad1.dpad_right;
        telemetry.addData("Aliança", GlobalConstants.alliance);
        telemetry.addLine("D-pad esquerdo = BLUE | direito = RED");
        telemetry.update();
    }

    @Override
    public void preRun() {
        robot.initializeForOpMode();
        Pose start = forAlliance(GlobalConstants.AUTO_START_POSE);
        Pose score = forAlliance(GlobalConstants.AUTO_SCORE_POSE);
        robot.drive.setPose(start);
        robot.turret.setAutoAimEnabled(robot.turret.isForwardZeroCalibrated());
        Path scorePath = line(start, score).linear(start, score);
        schedule(robot.drive.followPath(scorePath));
        started = true;
    }

    @Override
    public void run() {
        super.run();
        Pose pose = robot.drive.getPose();
        if (robot.turret.isAutoAimEnabled()) {
            robot.turret.aimAt(pose);
        }
        robot.feedback.update(robot.gamePieces.getState(), robot.magazine,
                robot.gamePieces.isReadyToShoot(), robot.gamePieces.isFlowerCollection());
        telemetry.addData("Pose atual", "%.1f / %.1f / %.1f°", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()));
        telemetry.addData("Torre / alvo", "%.1f° / %.1f°", robot.turret.getAngleDegrees(),
                robot.turret.getTargetAngleDegrees());
        telemetry.update();
    }

    @Override
    public void end() {
        if (robot == null) return;
        if (started) AutoPoseHandoff.store(robot.drive.getPose());
        robot.gamePieces.cancelToSafe();
        robot.drive.stop();
        robot.turret.stop();
    }

    private Pose forAlliance(Pose bluePose) {
        if (GlobalConstants.alliance == GlobalConstants.Alliance.BLUE) return bluePose;
        return new Pose(GlobalConstants.FIELD_SIZE_INCHES - bluePose.x(), bluePose.y(),
                MathUtils.wrapRadians(Math.PI - bluePose.heading()));
    }
}
