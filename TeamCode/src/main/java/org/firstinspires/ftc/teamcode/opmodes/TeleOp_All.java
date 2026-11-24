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

/** TeleOp de um operador usando diretamente as bindings do GamepadEx/SolverLib. */
@TeleOp(name = "BIOBUZZ Lite - TeleOp", group = "TeleOp")
public final class TeleOp_All extends OpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final FieldTargetSelector targetSelector = new FieldTargetSelector();
    private MyRobot robot;
    private GamepadEx driver;
    private GlobalConstants.FieldTarget target;
    private boolean gateOpen;
    private String poseOrigin = "pose inicial configurada";

    @Override public void init() {
        scheduler.reset();
        GlobalConstants.robotMode = GlobalConstants.RobotMode.TELEOP;
        robot = new MyRobot(hardwareMap);
        driver = new GamepadEx(gamepad1);
        robot.initializeForOpMode();

        telemetry.addLine("INIT: G1 D-pad esquerdo/direito seleciona BLUE/RED");
        telemetry.addLine("G1: sticks dirigem | A coleta | X descarrega | B para | Y prepara/dispara");
        telemetry.addLine("G1: RB alterna mira | BACK redefine pose | START calibra zero frontal da torre");
        telemetry.addLine("G1: D-pad cima/baixo = flower deploy/recolher; direita/esquerda = gate abrir/fechar");
        telemetry.addLine("Segure LB: A/B/X intake frente/reverso/parar; D-pad direita/esquerda/baixo transfer +/−/parar");
        telemetry.addLine("Segure LB: RT/LT shooter frente/parar; eixo X do stick direito controla a torre");
        telemetry.update();
    }

    @Override public void init_loop() {
        if (driver.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            GlobalConstants.alliance = GlobalConstants.Alliance.BLUE;
        }
        if (driver.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            GlobalConstants.alliance = GlobalConstants.Alliance.RED;
        }
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

        robot.drive.setDefaultCommand(new RunCommand(() -> robot.drive.driveManual(new DrivePowers(
                -driver.getLeftY(), -driver.getLeftX(),
                driver.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 0 : -driver.getRightX())), robot.drive));
        robot.turret.setDefaultCommand(new RunCommand(() -> {
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

        // A/B/X/Y em jogo principal. LB desvia os mesmos botões para controle individual.
        driver.getGamepadButton(GamepadKeys.Button.A)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(robot.gamePieces::startFloorCollection,
                        robot.gamePieces, robot.intake, robot.transfer, robot.shooter,
                        robot.ballGate, robot.flowerCollector));
        driver.getGamepadButton(GamepadKeys.Button.X)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(robot.gamePieces::reverseManualIntake,
                        robot.gamePieces, robot.intake, robot.transfer, robot.shooter,
                        robot.ballGate, robot.flowerCollector));
        driver.getGamepadButton(GamepadKeys.Button.B)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(robot.gamePieces::cancelToSafe,
                        robot.gamePieces, robot.intake, robot.transfer, robot.shooter,
                        robot.ballGate, robot.flowerCollector));
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(robot.gamePieces::prepareShot,
                                robot.gamePieces, robot.intake, robot.transfer, robot.shooter,
                                robot.ballGate, robot.flowerCollector),
                        new WaitUntilCommand(robot.gamePieces::isReadyToShoot),
                        new InstantCommand(robot.gamePieces::startFiring,
                                robot.gamePieces, robot.intake, robot.transfer, robot.shooter,
                                robot.ballGate, robot.flowerCollector)));

        // Coletor e gate no D-pad; o bumper esquerdo usa o D-pad para controlar transfer.
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(robot.flowerCollector::deploy, robot.flowerCollector));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(robot.flowerCollector::retract, robot.flowerCollector));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(() -> { gateOpen = true; robot.ballGate.unlock(); }, robot.ballGate));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(() -> { gateOpen = false; robot.ballGate.lock(); }, robot.ballGate));

        // Cancela a operação composta primeiro; um botão mantido pode então ajustar um mecanismo.
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(robot.gamePieces::cancelToSafe,
                        robot.gamePieces, robot.intake, robot.transfer, robot.shooter,
                        robot.ballGate, robot.flowerCollector));

        // D-pad e botões de ajuste individual enquanto LB estiver pressionado.
        driver.getGamepadButton(GamepadKeys.Button.A)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new InstantCommand(() -> robot.intake.setState(Intake.State.FORWARD), robot.intake));
        driver.getGamepadButton(GamepadKeys.Button.B)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new InstantCommand(() -> robot.intake.setState(Intake.State.REVERSE), robot.intake));
        driver.getGamepadButton(GamepadKeys.Button.X)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new InstantCommand(() -> robot.intake.setState(Intake.State.STOP), robot.intake));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new InstantCommand(() -> robot.transfer.setState(Transfer.State.FORWARD), robot.transfer));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new InstantCommand(() -> robot.transfer.setState(Transfer.State.REVERSE), robot.transfer));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new InstantCommand(() -> robot.transfer.setState(Transfer.State.STOP), robot.transfer));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(
                        () -> robot.turret.setAutoAimEnabled(!robot.turret.isAutoAimEnabled()), robot.turret));
        driver.getGamepadButton(GamepadKeys.Button.BACK)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(() -> {
                    robot.drive.setPose(GlobalConstants.TURRET_RESET_POSE);
                    targetSelector.reset(GlobalConstants.TURRET_RESET_POSE.y());
                    poseOrigin = "pose de reset da torre";
                }, robot.drive));
        driver.getGamepadButton(GamepadKeys.Button.START)
                .and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).negate())
                .whenActive(new InstantCommand(() -> {
                    if (Math.abs(driver.getLeftX()) < 0.05 && Math.abs(driver.getLeftY()) < 0.05
                            && Math.abs(driver.getRightX()) < 0.05 && !robot.turret.isAutoAimEnabled()) {
                        robot.turret.calibrateForwardZero();
                        robot.turret.setAutoAimEnabled(true);
                        poseOrigin = "zero frontal da torre recalibrado";
                    }
                }, robot.turret));
        new Trigger(() -> driver.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(new InstantCommand(robot.gamePieces::startShooterManual,
                        robot.gamePieces, robot.shooter));
        new Trigger(() -> driver.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(new InstantCommand(robot.shooter::stop, robot.shooter));
    }

    private Pose teleopStartPose() {
        return GlobalConstants.alliance == GlobalConstants.Alliance.BLUE
                ? GlobalConstants.BLUE_FLOWER_START_POSE : GlobalConstants.RED_FLOWER_START_POSE;
    }

    @Override public void loop() {
        scheduler.run();
        publishTelemetry(robot.drive.getPose());
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
