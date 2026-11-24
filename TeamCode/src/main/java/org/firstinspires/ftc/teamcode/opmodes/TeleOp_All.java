package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.hardware.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.hardware.MyRobot;
import org.firstinspires.ftc.teamcode.util.FieldTargetSelector;

/** Controles de partida de um operador, com comandos do SolverLib vinculados ao Gamepad 1. */
@TeleOp(name = "BIOBUZZ Lite - TeleOp", group = "TeleOp")
public final class TeleOp_All extends OpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final FieldTargetSelector targetSelector = new FieldTargetSelector();
    private MyRobot robot;
    private GamepadEx driver;
    private GlobalConstants.FieldTarget target;
    private boolean started;
    private boolean gateOpen;
    private String poseOrigin = "pose inicial configurada";

    @Override public void init() {
        scheduler.reset();
        GlobalConstants.robotMode = GlobalConstants.RobotMode.TELEOP;
        robot = new MyRobot(hardwareMap);
        driver = new GamepadEx(gamepad1);
        robot.initializeForOpMode();
        configureDefaultCommands();
        configureGamepadCommands();
        telemetry.addLine("INIT: G1 D-pad esquerdo/direito seleciona BLUE/RED");
        telemetry.addLine("G1: sticks dirigem | A coleta | X descarrega | B para | Y prepara/dispara");
        telemetry.addLine("G1: RB alterna mira | BACK redefine pose | START calibra zero frontal da torre");
        telemetry.addLine("G1: D-pad cima/baixo = flower deploy/recolher; direita/esquerda = gate abrir/fechar");
        telemetry.addLine("Segure LB: A/B/X intake frente/reverso/parar; D-pad direita/esquerda/baixo transfer +/−/parar");
        telemetry.addLine("Segure LB: RT/LT shooter frente/parar; eixo X do stick direito controla a torre");
        telemetry.update();
    }

    private void configureDefaultCommands() {
        robot.drive.setDefaultCommand(new RunCommand(() -> {
            if (!started) {
                robot.drive.driveManual(new DrivePowers(0, 0, 0));
                return;
            }
            double turn = driver.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 0.0 : -driver.getRightX();
            robot.drive.driveManual(new DrivePowers(-driver.getLeftY(), -driver.getLeftX(), turn));
        }, robot.drive));

        robot.turret.setDefaultCommand(new RunCommand(() -> {
            if (!started) return;
            Pose pose = robot.drive.getPose();
            target = targetSelector.update(pose.y());
            if (driver.getButton(GamepadKeys.Button.LEFT_BUMPER) && Math.abs(driver.getRightX()) > 0.05) {
                robot.turret.setManualPower(-driver.getRightX());
            } else if (robot.turret.isAutoAimEnabled()) {
                robot.turret.aimAtFieldTarget(pose, target.x, target.y);
            } else {
                robot.turret.stop();
            }
        }, robot.turret));

        robot.feedback.setDefaultCommand(new RunCommand(() -> robot.feedback.update(
                robot.gamePieces.getState(), robot.magazine, robot.gamePieces.isReadyToShoot(),
                robot.gamePieces.isFlowerCollection()), robot.feedback));
    }

    private void configureGamepadCommands() {
        Trigger initMode = new Trigger(() -> !started);
        Trigger matchMode = new Trigger(() -> started
                && !driver.getButton(GamepadKeys.Button.LEFT_BUMPER));
        Trigger manualMode = new Trigger(() -> started
                && driver.getButton(GamepadKeys.Button.LEFT_BUMPER));

        bind(initMode.and(button(GamepadKeys.Button.DPAD_LEFT)),
                () -> GlobalConstants.alliance = GlobalConstants.Alliance.BLUE);
        bind(initMode.and(button(GamepadKeys.Button.DPAD_RIGHT)),
                () -> GlobalConstants.alliance = GlobalConstants.Alliance.RED);

        // Comandos de jogo em modo normal.
        bind(matchMode.and(button(GamepadKeys.Button.A)), robot.gamePieces::startFloorCollection,
                collectionRequirements());
        bind(matchMode.and(button(GamepadKeys.Button.X)), robot.gamePieces::reverseManualIntake,
                collectionRequirements());
        bind(matchMode.and(button(GamepadKeys.Button.B)), robot.gamePieces::cancelToSafe,
                safeRequirements());
        matchMode.and(button(GamepadKeys.Button.Y)).whenActive(new SequentialCommandGroup(
                new InstantCommand(robot.gamePieces::prepareShot, collectionRequirements()),
                new WaitUntilCommand(robot.gamePieces::isReadyToShoot),
                new InstantCommand(robot.gamePieces::startFiring, collectionRequirements())));

        bind(matchMode.and(button(GamepadKeys.Button.DPAD_UP)), robot.flowerCollector::deploy,
                robot.flowerCollector);
        bind(matchMode.and(button(GamepadKeys.Button.DPAD_DOWN)), robot.flowerCollector::retract,
                robot.flowerCollector);
        bind(matchMode.and(button(GamepadKeys.Button.DPAD_RIGHT)), () -> {
            gateOpen = true;
            robot.ballGate.unlock();
        }, robot.ballGate);
        bind(matchMode.and(button(GamepadKeys.Button.DPAD_LEFT)), () -> {
            gateOpen = false;
            robot.ballGate.lock();
        }, robot.ballGate);
        bind(matchMode.and(button(GamepadKeys.Button.RIGHT_BUMPER)),
                () -> robot.turret.setAutoAimEnabled(!robot.turret.isAutoAimEnabled()), robot.turret);
        bind(matchMode.and(button(GamepadKeys.Button.BACK)), this::resetPose, robot.drive);
        bind(matchMode.and(button(GamepadKeys.Button.START)), this::calibrateTurretZero, robot.turret);

        // Entrar no modo individual interrompe o fluxo composto; depois cada botão age diretamente.
        bind(new Trigger(() -> started && driver.getButton(GamepadKeys.Button.LEFT_BUMPER)),
                robot.gamePieces::cancelToSafe,
                safeRequirements());
        bind(manualMode.and(button(GamepadKeys.Button.A)),
                () -> robot.intake.setState(Intake.State.FORWARD),
                robot.intake);
        bind(manualMode.and(button(GamepadKeys.Button.B)),
                () -> robot.intake.setState(Intake.State.REVERSE),
                robot.intake);
        bind(manualMode.and(button(GamepadKeys.Button.X)),
                () -> robot.intake.setState(Intake.State.STOP),
                robot.intake);
        bind(manualMode.and(button(GamepadKeys.Button.DPAD_RIGHT)),
                () -> robot.transfer.setState(Transfer.State.FORWARD),
                robot.transfer);
        bind(manualMode.and(button(GamepadKeys.Button.DPAD_LEFT)),
                () -> robot.transfer.setState(Transfer.State.REVERSE),
                robot.transfer);
        bind(manualMode.and(button(GamepadKeys.Button.DPAD_DOWN)),
                () -> robot.transfer.setState(Transfer.State.STOP),
                robot.transfer);
        bind(new Trigger(() -> started && driver.getButton(GamepadKeys.Button.LEFT_BUMPER)
                        && driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5),
                robot.gamePieces::startShooterManual, robot.gamePieces, robot.shooter);
        bind(new Trigger(() -> started && driver.getButton(GamepadKeys.Button.LEFT_BUMPER)
                        && driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5),
                robot.shooter::stop, robot.shooter);
    }

    private Trigger button(GamepadKeys.Button button) {
        return driver.getGamepadButton(button);
    }

    private void bind(Trigger trigger, Runnable action, Subsystem... requirements) {
        trigger.whenActive(new InstantCommand(action, requirements));
    }

    private Subsystem[] collectionRequirements() {
        return new Subsystem[] {robot.gamePieces, robot.intake, robot.transfer, robot.shooter,
                robot.ballGate, robot.flowerCollector};
    }

    private Subsystem[] safeRequirements() {
        return new Subsystem[] {robot.gamePieces, robot.intake, robot.transfer, robot.shooter,
                robot.ballGate, robot.flowerCollector};
    }

    @Override public void init_loop() {
        scheduler.run();
        telemetry.addData("Aliança", GlobalConstants.alliance);
        telemetry.update();
    }

    @Override public void start() {
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
        started = true;
    }

    @Override public void loop() {
        scheduler.run();
        publishTelemetry(robot.drive.getPose());
    }

    private void resetPose() {
        robot.drive.setPose(GlobalConstants.TURRET_RESET_POSE);
        targetSelector.reset(GlobalConstants.TURRET_RESET_POSE.y());
        poseOrigin = "pose de reset da torre";
    }

    private void calibrateTurretZero() {
        boolean driveStopped = Math.abs(driver.getLeftX()) < 0.05
                && Math.abs(driver.getLeftY()) < 0.05 && Math.abs(driver.getRightX()) < 0.05;
        if (driveStopped && !robot.turret.isAutoAimEnabled()) {
            robot.turret.calibrateForwardZero();
            robot.turret.setAutoAimEnabled(true);
            poseOrigin = "zero frontal da torre recalibrado";
        }
    }

    private Pose teleopStartPose() {
        return GlobalConstants.alliance == GlobalConstants.Alliance.BLUE
                ? GlobalConstants.BLUE_FLOWER_START_POSE : GlobalConstants.RED_FLOWER_START_POSE;
    }

    private void publishTelemetry(Pose pose) {
        telemetry.addData("Pose / origem", "%.1f, %.1f, %.1f° | %s", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()), poseOrigin);
        telemetry.addData("Aliança / alvo", "%s | %.1f, %.1f", GlobalConstants.alliance,
                target == null ? robot.turret.getTargetX() : target.x,
                target == null ? robot.turret.getTargetY() : target.y);
        telemetry.addData("Torre", "mira %s | %.1f° / %.1f°", robot.turret.isAutoAimEnabled(),
                robot.turret.getAngleDegrees(), robot.turret.getTargetAngleDegrees());
        telemetry.addData("Encoder torre / verificação", "motor %.0f° | Through Bore %.0f°",
                robot.turret.getAngleDegrees(), robot.turret.getVerificationAngleDegrees());
        telemetry.addData("Bolas", "%d / %d | %.0f mm", robot.magazine.getBallCount(),
                GlobalConstants.MAGAZINE_CAPACITY, robot.magazine.getInletDistanceMillimeters());
        telemetry.addData("Intake / transfer / shooter", "%s / %s / %s", robot.intake.getState(),
                robot.transfer.getState(), robot.shooter.getState());
        telemetry.addData("Gate / coletor", "%s / %s", gateOpen ? "aberto" : "fechado",
                robot.flowerCollector.isDeployed());
        telemetry.addData("Fluxo", "%s | sensor %s | mira %s", robot.gamePieces.getState(),
                robot.magazine.getLoadEstimate(), robot.turret.isAtTarget());
        telemetry.update();
    }

    @Override public void stop() {
        scheduler.cancelAll();
        if (robot == null) return;
        robot.gamePieces.cancelToSafe();
        robot.drive.stop();
        robot.turret.stop();
    }
}
