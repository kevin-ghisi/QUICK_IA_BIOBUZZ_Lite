package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.hardware.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.hardware.MyRobot;
import org.firstinspires.ftc.teamcode.util.FieldTargetSelector;

/** Controles de partida para um operador, todos no Gamepad 1. */
@TeleOp(name = "BIOBUZZ Lite - TeleOp", group = "TeleOp")
public final class TeleOp_All extends OpMode {
    private MyRobot robot;
    private final FieldTargetSelector targetSelector = new FieldTargetSelector();
    private GlobalConstants.FieldTarget target;
    private boolean leftAllianceWasDown, rightAllianceWasDown;
    private boolean collectWasDown, unloadWasDown, safeWasDown, shootWasDown;
    private boolean aimWasDown, resetPoseWasDown, turretZeroWasDown;
    private boolean modifierWasDown, intakeForwardWasDown, intakeReverseWasDown, intakeStopWasDown;
    private boolean transferForwardWasDown, transferReverseWasDown, transferStopWasDown;
    private boolean shooterWasDown, shooterStopWasDown, flowerDeployWasDown, flowerRetractWasDown;
    private boolean gateOpenWasDown, gateLockWasDown;
    private boolean shootRequested, gateOpen;
    private String poseOrigin = "pose inicial configurada";

    @Override public void init() {
        GlobalConstants.robotMode = GlobalConstants.RobotMode.TELEOP;
        robot = new MyRobot(hardwareMap);
        telemetry.addLine("INIT: G1 D-pad esquerdo/direito seleciona BLUE/RED");
        telemetry.addLine("G1: sticks dirigem | A coleta | X descarrega | B para | Y prepara/dispara");
        telemetry.addLine("G1: RB alterna mira | BACK redefine pose | START calibra zero frontal da torre");
        telemetry.addLine("G1: D-pad cima/baixo = flower deploy/recolher; direita/esquerda = gate abrir/fechar");
        telemetry.addLine("Segure LB: A/B/X intake frente/reverso/parar; D-pad direita/esquerda/baixo transfer +/−/parar");
        telemetry.addLine("Segure LB: RT/LT shooter frente/parar; eixo X do stick direito controla a torre");
        telemetry.update();
    }

    @Override public void init_loop() {
        if (gamepad1.dpad_left && !leftAllianceWasDown) GlobalConstants.alliance = GlobalConstants.Alliance.BLUE;
        if (gamepad1.dpad_right && !rightAllianceWasDown) GlobalConstants.alliance = GlobalConstants.Alliance.RED;
        leftAllianceWasDown = gamepad1.dpad_left;
        rightAllianceWasDown = gamepad1.dpad_right;
        telemetry.addData("Aliança", GlobalConstants.alliance);
        telemetry.update();
    }

    @Override public void start() {
        robot.initializeForOpMode();
        Pose handedOffPose = AutoPoseHandoff.consume();
        if (handedOffPose != null) {
            robot.drive.setPose(handedOffPose);
            poseOrigin = "pose final recebida do autônomo";
        } else {
            robot.drive.setPose(teleopStartPose());
        }
        targetSelector.reset(robot.drive.getPose().y());
        target = targetSelector.update(robot.drive.getPose().y());
        robot.turret.setAutoAimEnabled(robot.turret.isForwardZeroCalibrated());
    }

    @Override public void loop() {
        boolean manualMode = gamepad1.left_bumper;
        robot.drive.driveManual(new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x,
                manualMode ? 0.0 : -gamepad1.right_stick_x));
        robot.drive.periodic();
        Pose pose = robot.drive.getPose();
        target = targetSelector.update(pose.y());

        if (manualMode != modifierWasDown) syncControlEdges();
        if (manualMode && !modifierWasDown) {
            shootRequested = false;
            robot.gamePieces.cancelToSafe();
        }
        if (manualMode) handleIndividualControls();
        else handleMatchControls();
        modifierWasDown = manualMode;

        if (shootRequested && robot.gamePieces.isReadyToShoot()) {
            robot.gamePieces.startFiring();
            shootRequested = false;
        }
        if (robot.turret.isAutoAimEnabled()) {
            robot.turret.aimAtFieldTarget(pose, target.x, target.y);
        } else {
            robot.turret.setManualPower(manualMode ? -gamepad1.right_stick_x : 0.0);
        }

        robot.gamePieces.periodic();
        if (robot.shooter.getState() == Shooter.State.FORWARD) robot.shooter.periodic();
        robot.feedback.update(robot.gamePieces.getState(), robot.magazine,
                robot.gamePieces.isReadyToShoot(), robot.gamePieces.isFlowerCollection());
        publishTelemetry(pose);
    }

    /** Evita que um botão mantido durante a troca de modo vire um novo comando. */
    private void syncControlEdges() {
        collectWasDown = gamepad1.a; unloadWasDown = gamepad1.x; safeWasDown = gamepad1.b;
        shootWasDown = gamepad1.y; aimWasDown = gamepad1.right_bumper;
        resetPoseWasDown = gamepad1.back; turretZeroWasDown = gamepad1.start;
        intakeForwardWasDown = gamepad1.a; intakeReverseWasDown = gamepad1.b;
        intakeStopWasDown = gamepad1.x; transferForwardWasDown = gamepad1.dpad_right;
        transferReverseWasDown = gamepad1.dpad_left; transferStopWasDown = gamepad1.dpad_down;
        shooterWasDown = gamepad1.right_trigger > 0.5; shooterStopWasDown = gamepad1.left_trigger > 0.5;
        flowerDeployWasDown = gamepad1.dpad_up; flowerRetractWasDown = gamepad1.dpad_down;
        gateOpenWasDown = gamepad1.dpad_right; gateLockWasDown = gamepad1.dpad_left;
    }

    private void handleMatchControls() {
        if (gamepad1.a && !collectWasDown) { shootRequested = false; robot.gamePieces.startFloorCollection(); }
        if (gamepad1.x && !unloadWasDown) { shootRequested = false; robot.gamePieces.reverseManualIntake(); }
        if (gamepad1.b && !safeWasDown) { shootRequested = false; robot.gamePieces.cancelToSafe(); }
        if (gamepad1.y && !shootWasDown) { robot.gamePieces.prepareShot(); shootRequested = true; }
        if (gamepad1.dpad_up && !flowerDeployWasDown) robot.flowerCollector.deploy();
        if (gamepad1.dpad_down && !flowerRetractWasDown) robot.flowerCollector.retract();
        if (gamepad1.dpad_right && !gateOpenWasDown) { gateOpen = true; robot.ballGate.unlock(); }
        if (gamepad1.dpad_left && !gateLockWasDown) { gateOpen = false; robot.ballGate.lock(); }
        if (gamepad1.right_bumper && !aimWasDown) robot.turret.setAutoAimEnabled(!robot.turret.isAutoAimEnabled());
        if (gamepad1.back && !resetPoseWasDown) {
            robot.drive.setPose(GlobalConstants.TURRET_RESET_POSE);
            targetSelector.reset(GlobalConstants.TURRET_RESET_POSE.y());
            poseOrigin = "pose de reset da torre";
        }
        if (gamepad1.start && !turretZeroWasDown) {
            boolean driveStopped = Math.abs(gamepad1.left_stick_x) < 0.05
                    && Math.abs(gamepad1.left_stick_y) < 0.05 && Math.abs(gamepad1.right_stick_x) < 0.05;
            if (driveStopped && !robot.turret.isAutoAimEnabled()) {
                robot.turret.calibrateForwardZero();
                robot.turret.setAutoAimEnabled(true);
                poseOrigin = "zero frontal da torre recalibrado";
            }
        }
        collectWasDown = gamepad1.a; unloadWasDown = gamepad1.x; safeWasDown = gamepad1.b;
        shootWasDown = gamepad1.y; aimWasDown = gamepad1.right_bumper;
        resetPoseWasDown = gamepad1.back; turretZeroWasDown = gamepad1.start;
        flowerDeployWasDown = gamepad1.dpad_up; flowerRetractWasDown = gamepad1.dpad_down;
        gateOpenWasDown = gamepad1.dpad_right; gateLockWasDown = gamepad1.dpad_left;
    }

    /** O bumper esquerdo troca para controles individuais sem disputa com o fluxo automático. */
    private void handleIndividualControls() {
        if (gamepad1.a && !intakeForwardWasDown) robot.intake.setState(Intake.State.FORWARD);
        if (gamepad1.b && !intakeReverseWasDown) robot.intake.setState(Intake.State.REVERSE);
        if (gamepad1.x && !intakeStopWasDown) robot.intake.setState(Intake.State.STOP);
        if (gamepad1.dpad_right && !transferForwardWasDown) robot.transfer.setState(Transfer.State.FORWARD);
        if (gamepad1.dpad_left && !transferReverseWasDown) robot.transfer.setState(Transfer.State.REVERSE);
        if (gamepad1.dpad_down && !transferStopWasDown) robot.transfer.setState(Transfer.State.STOP);
        if (gamepad1.right_trigger > 0.5 && !shooterWasDown) robot.gamePieces.startShooterManual();
        if (gamepad1.left_trigger > 0.5 && !shooterStopWasDown) robot.shooter.stop();
        if (Math.abs(gamepad1.right_stick_x) > 0.05) robot.turret.setAutoAimEnabled(false);
        intakeForwardWasDown = gamepad1.a; intakeReverseWasDown = gamepad1.b; intakeStopWasDown = gamepad1.x;
        transferForwardWasDown = gamepad1.dpad_right; transferReverseWasDown = gamepad1.dpad_left;
        transferStopWasDown = gamepad1.dpad_down; shooterWasDown = gamepad1.right_trigger > 0.5;
        shooterStopWasDown = gamepad1.left_trigger > 0.5; flowerDeployWasDown = gamepad1.dpad_up;
        flowerRetractWasDown = gamepad1.dpad_down; gateOpenWasDown = gamepad1.dpad_right;
        gateLockWasDown = gamepad1.dpad_left;
    }

    private Pose teleopStartPose() {
        return GlobalConstants.alliance == GlobalConstants.Alliance.BLUE
                ? GlobalConstants.BLUE_FLOWER_START_POSE : GlobalConstants.RED_FLOWER_START_POSE;
    }

    private void publishTelemetry(Pose pose) {
        telemetry.addData("Pose / origem", "%.1f, %.1f, %.1f° | %s", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()), poseOrigin);
        telemetry.addData("Aliança / alvo", "%s | %.1f, %.1f", GlobalConstants.alliance, target.x, target.y);
        telemetry.addData("Torre", "mira %s | %.1f° / %.1f°", robot.turret.isAutoAimEnabled(),
                robot.turret.getAngleDegrees(), robot.turret.getTargetAngleDegrees());
        telemetry.addData("Encoder torre / verificação", "motor %.0f° | Through Bore %.0f°",
                robot.turret.getAngleDegrees(), robot.turret.getVerificationAngleDegrees());
        telemetry.addData("Bolas", "%d / %d | %.0f mm", robot.magazine.getBallCount(),
                GlobalConstants.MAGAZINE_CAPACITY, robot.magazine.getInletDistanceMillimeters());
        telemetry.addData("Intake / transfer / shooter", "%s / %s / %s",
                robot.intake.getState(), robot.transfer.getState(), robot.shooter.getState());
        telemetry.addData("Gate / coletor", "%s / %s", gateOpen ? "aberto" : "fechado",
                robot.flowerCollector.isDeployed());
        telemetry.addData("Fluxo", "%s | sensor %s | mira %s", robot.gamePieces.getState(),
                robot.magazine.getLoadEstimate(), robot.turret.isAtTarget());
        telemetry.update();
    }

    @Override public void stop() {
        if (robot == null) return;
        robot.gamePieces.cancelToSafe(); robot.drive.stop(); robot.turret.stop();
    }
}
