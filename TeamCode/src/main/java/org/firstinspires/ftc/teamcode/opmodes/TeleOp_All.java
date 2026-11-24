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
import org.firstinspires.ftc.teamcode.util.FieldMotionEstimator;
import org.firstinspires.ftc.teamcode.util.FieldTargetSelector;
import org.firstinspires.ftc.teamcode.util.TurretAimPrediction;

/** TeleOp compacto: controles de partida no gamepad 1 e ajustes individuais no gamepad 2. */
@TeleOp(name = "BIOBUZZ Lite - TeleOp", group = "TeleOp")
public final class TeleOp_All extends OpMode {
    private MyRobot robot;
    private final FieldMotionEstimator motionEstimator = new FieldMotionEstimator();
    private final FieldTargetSelector targetSelector = new FieldTargetSelector();
    private GlobalConstants.FieldTarget target;
    private boolean leftAllianceWasDown;
    private boolean rightAllianceWasDown;
    private boolean collectWasDown;
    private boolean unloadWasDown;
    private boolean safeWasDown;
    private boolean shootWasDown;
    private boolean aimWasDown;
    private boolean resetPoseWasDown;
    private boolean turretZeroWasDown;
    private boolean shootRequested;
    private boolean gateOpen;
    private boolean intakeForwardWasDown;
    private boolean intakeReverseWasDown;
    private boolean intakeStopWasDown;
    private boolean transferForwardWasDown;
    private boolean transferReverseWasDown;
    private boolean transferStopWasDown;
    private boolean shooterWasDown;
    private boolean shooterStopWasDown;
    private boolean flowerDeployWasDown;
    private boolean flowerRetractWasDown;
    private boolean gateOpenWasDown;
    private boolean gateLockWasDown;
    private boolean hoodLowerWasDown;
    private boolean hoodRaiseWasDown;
    private String poseOrigin = "pose inicial configurada";

    @Override
    public void init() {
        GlobalConstants.robotMode = GlobalConstants.RobotMode.TELEOP;
        robot = new MyRobot(hardwareMap);
        telemetry.addLine("INIT: gamepad 1 D-pad esquerdo = BLUE | direito = RED");
        telemetry.addLine("G1: sticks dirigem | A coleta | X descarrega | B para | Y prepara/dispara");
        telemetry.addLine("G1: RB alterna mira | BACK redefine pose | START calibra zero frontal da torre");
        telemetry.addLine("G2: A/B/X intake | Y/LB/RB transfer | RT/LT shooter");
        telemetry.addLine("G2: D-pad controla ball gate e flower collector | stick esquerdo controla torre");
        telemetry.addLine("G2: START/BACK ajustam o hood em passos pequenos");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left && !leftAllianceWasDown) {
            GlobalConstants.alliance = GlobalConstants.Alliance.BLUE;
        }
        if (gamepad1.dpad_right && !rightAllianceWasDown) {
            GlobalConstants.alliance = GlobalConstants.Alliance.RED;
        }
        leftAllianceWasDown = gamepad1.dpad_left;
        rightAllianceWasDown = gamepad1.dpad_right;
        telemetry.addData("Aliança", GlobalConstants.alliance);
        telemetry.update();
    }

    @Override
    public void start() {
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
        motionEstimator.reset();
        robot.turret.setAutoAimEnabled(robot.turret.isForwardZeroCalibrated());
    }

    @Override
    public void loop() {
        robot.drive.driveManual(new DrivePowers(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
        robot.drive.periodic();
        Pose pose = robot.drive.getPose();
        FieldMotionEstimator.Motion motion = motionEstimator.update(pose, System.nanoTime());
        target = targetSelector.update(pose.y());

        handleMatchControls();
        handleIndividualControls();

        if (shootRequested && robot.gamePieces.isReadyToShoot()) {
            robot.gamePieces.startFiring();
            shootRequested = false;
        }

        if (robot.turret.isAutoAimEnabled()) {
            Pose predictedPose = TurretAimPrediction.predictRobotPose(pose, motion);
            robot.turret.aimAtFieldTarget(predictedPose, target.x, target.y);
            robot.shooter.updateHoodForDistance(robot.turret.getTargetDistanceInches());
        } else {
            robot.turret.setManualPower(-gamepad2.left_stick_x);
        }

        robot.gamePieces.periodic();
        if (robot.shooter.getState() == Shooter.State.FORWARD) robot.shooter.periodic();
        robot.feedback.update(robot.gamePieces.getState(), robot.magazine,
                robot.gamePieces.isReadyToShoot(), robot.gamePieces.isFlowerCollection());
        publishTelemetry(pose, motion);
    }

    private void handleMatchControls() {
        if (gamepad1.a && !collectWasDown) {
            shootRequested = false;
            robot.gamePieces.startFloorCollection();
        }
        if (gamepad1.x && !unloadWasDown) {
            shootRequested = false;
            robot.gamePieces.reverseManualIntake();
        }
        if (gamepad1.b && !safeWasDown) {
            shootRequested = false;
            robot.gamePieces.cancelToSafe();
        }
        if (gamepad1.y && !shootWasDown) {
            robot.gamePieces.prepareShot();
            shootRequested = true;
        }
        if (gamepad1.right_bumper && !aimWasDown) {
            robot.turret.setAutoAimEnabled(!robot.turret.isAutoAimEnabled());
        }
        if (gamepad1.back && !resetPoseWasDown) {
            robot.drive.setPose(GlobalConstants.TURRET_RESET_POSE);
            targetSelector.reset(GlobalConstants.TURRET_RESET_POSE.y());
            poseOrigin = "pose de reset da torre";
            motionEstimator.reset();
        }
        if (gamepad1.start && !turretZeroWasDown) {
            boolean driveStopped = Math.abs(gamepad1.left_stick_x) < 0.05
                    && Math.abs(gamepad1.left_stick_y) < 0.05
                    && Math.abs(gamepad1.right_stick_x) < 0.05;
            boolean turretStopped = Math.abs(gamepad2.left_stick_x) < 0.05;
            if (driveStopped && turretStopped && !robot.turret.isAutoAimEnabled()) {
                robot.turret.calibrateForwardZero();
                robot.turret.setAutoAimEnabled(true);
                poseOrigin = "zero frontal da torre recalibrado";
            }
        }
        collectWasDown = gamepad1.a;
        unloadWasDown = gamepad1.x;
        safeWasDown = gamepad1.b;
        shootWasDown = gamepad1.y;
        aimWasDown = gamepad1.right_bumper;
        resetPoseWasDown = gamepad1.back;
        turretZeroWasDown = gamepad1.start;
    }

    private void handleIndividualControls() {
        if (gamepad2.a && !intakeForwardWasDown) robot.intake.setState(Intake.State.FORWARD);
        if (gamepad2.b && !intakeReverseWasDown) robot.intake.setState(Intake.State.REVERSE);
        if (gamepad2.x && !intakeStopWasDown) robot.intake.setState(Intake.State.STOP);
        if (gamepad2.y && !transferForwardWasDown) robot.transfer.setState(Transfer.State.FORWARD);
        if (gamepad2.left_bumper && !transferReverseWasDown) robot.transfer.setState(Transfer.State.REVERSE);
        if (gamepad2.right_bumper && !transferStopWasDown) robot.transfer.setState(Transfer.State.STOP);
        if (gamepad2.right_trigger > 0.5 && !shooterWasDown) robot.gamePieces.startShooterManual();
        if (gamepad2.left_trigger > 0.5 && !shooterStopWasDown) robot.shooter.stop();
        if (gamepad2.dpad_up && !flowerDeployWasDown) robot.flowerCollector.deploy();
        if (gamepad2.dpad_down && !flowerRetractWasDown) robot.flowerCollector.retract();
        if (gamepad2.dpad_right && !gateOpenWasDown) {
            gateOpen = true;
            robot.ballGate.unlock();
        }
        if (gamepad2.dpad_left && !gateLockWasDown) {
            gateOpen = false;
            robot.ballGate.lock();
        }
        if (gamepad2.left_stick_button || Math.abs(gamepad2.left_stick_x) > 0.05) {
            robot.turret.setAutoAimEnabled(false);
        }
        if (gamepad2.start && !hoodLowerWasDown) robot.hood.adjustTowardLowerAngle();
        if (gamepad2.back && !hoodRaiseWasDown) robot.hood.adjustTowardHigherAngle();

        intakeForwardWasDown = gamepad2.a;
        intakeReverseWasDown = gamepad2.b;
        intakeStopWasDown = gamepad2.x;
        transferForwardWasDown = gamepad2.y;
        transferReverseWasDown = gamepad2.left_bumper;
        transferStopWasDown = gamepad2.right_bumper;
        shooterWasDown = gamepad2.right_trigger > 0.5;
        shooterStopWasDown = gamepad2.left_trigger > 0.5;
        flowerDeployWasDown = gamepad2.dpad_up;
        flowerRetractWasDown = gamepad2.dpad_down;
        gateOpenWasDown = gamepad2.dpad_right;
        gateLockWasDown = gamepad2.dpad_left;
        hoodLowerWasDown = gamepad2.start;
        hoodRaiseWasDown = gamepad2.back;
    }

    private Pose teleopStartPose() {
        if (GlobalConstants.alliance == GlobalConstants.Alliance.BLUE) {
            return GlobalConstants.BLUE_FLOWER_START_POSE;
        }
        return GlobalConstants.RED_FLOWER_START_POSE;
    }

    private void publishTelemetry(Pose pose, FieldMotionEstimator.Motion motion) {
        telemetry.addData("Pose / origem", "%.1f, %.1f, %.1f° | %s", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()), poseOrigin);
        telemetry.addData("Aliança / alvo", "%s | %.1f, %.1f", GlobalConstants.alliance,
                target.x, target.y);
        telemetry.addData("Torre", "mira %s | %.1f° / %.1f°",
                robot.turret.isAutoAimEnabled(), robot.turret.getAngleDegrees(),
                robot.turret.getTargetAngleDegrees());
        telemetry.addData("Encoder torre / verificação", "motor %.0f° | Through Bore %.0f°",
                robot.turret.getAngleDegrees(), robot.turret.getVerificationAngleDegrees());
        telemetry.addData("Bolas", "%d / %d | %.0f mm", robot.magazine.getBallCount(),
                GlobalConstants.MAGAZINE_CAPACITY, robot.magazine.getInletDistanceMillimeters());
        telemetry.addData("Intake / transfer / shooter", "%s / %s / %s",
                robot.intake.getState(), robot.transfer.getState(), robot.shooter.getState());
        telemetry.addData("Gate / coletor / hood", "%s / %s / %.2f",
                gateOpen ? "aberto" : "fechado", robot.flowerCollector.isDeployed(), robot.hood.getPosition());
        telemetry.addData("Fluxo", "%s | sensor %s | mira %s", robot.gamePieces.getState(),
                robot.magazine.getLoadEstimate(), robot.turret.isAtTarget());
        telemetry.addData("Velocidade do robô (in/s)", "%.1f / %.1f", motion.velocityX, motion.velocityY);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot == null) return;
        robot.gamePieces.cancelToSafe();
        robot.drive.stop();
        robot.turret.stop();
    }
}
