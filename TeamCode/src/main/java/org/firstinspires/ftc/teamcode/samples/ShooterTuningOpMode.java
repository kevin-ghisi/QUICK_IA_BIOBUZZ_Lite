package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.hardware.MyRobot;
import org.firstinspires.ftc.teamcode.hardware.ShooterTuningConfig;
import org.firstinspires.ftc.teamcode.util.TimeUtils;

/** Bancada fechada para ajustar velocity PIDF no Panels mantendo a trava física bloqueada. */
@TeleOp(name = "SETUP - Shooter PIDF Panels", group = "Setup")
public final class ShooterTuningOpMode extends OpMode {
    private static final double TELEMETRY_INTERVAL_MILLISECONDS = 100.0;
    private MyRobot robot;
    private VoltageSensor battery;
    private double lastTelemetryMilliseconds;
    private boolean shooterEnabled;
    private boolean intakeEnabled;
    private boolean trasnferEnabled;
    private boolean previousA;
    private boolean previousB;
    private boolean previousX;

    private boolean ballGateEnabled;

    private boolean previousY;
    private boolean previousDpadL;



    /** Constrói e inicializa o robô; não gira nenhum mecanismo durante o init. */
    @Override
    public void init() {
        GlobalConstants.robotMode = GlobalConstants.RobotMode.TELEOP;
        robot = new MyRobot(hardwareMap);
        battery = hardwareMap.voltageSensor.iterator().next();
        robot.initializeForOpMode();
        telemetry.setMsTransmissionInterval(100);
        lastTelemetryMilliseconds = TimeUtils.monotonicMillisecondsPrecise();
        telemetry.addLine("Trava fechada. Ajuste alvo e ganhos no Panels antes de habilitar o shooter.");
        telemetry.addLine("PIDF executado pela SolversLib; ganhos são potência normalizada por erro em ticks/s.");
        telemetry.update();
    }

    /** Lê novamente os parâmetros do Panels a cada loop e reporta velocidade e tensão em tempo real. */
    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean dpad_l = gamepad1.dpad_left;
        if (a && !previousA) {
            shooterEnabled = !shooterEnabled;
            robot.shooter.setState(shooterEnabled
                    ? org.firstinspires.ftc.teamcode.commandbase.subsystem.Shooter.State.FORWARD
                    : org.firstinspires.ftc.teamcode.commandbase.subsystem.Shooter.State.STOP);
        }
        if (b && !previousB) {
            intakeEnabled = true;
            trasnferEnabled = true;
            robot.intake.setState(Intake.State.FORWARD);
            robot.transfer.setState(Transfer.State.REVERSE);
        }
        if (x && !previousX) {
            intakeEnabled = false;
            trasnferEnabled = false;
            robot.intake.setState(org.firstinspires.ftc.teamcode.commandbase.subsystem.Intake.State.STOP);
            robot.transfer.setState(Transfer.State.STOP);
        }
        if (y && !previousY) {
            ballGateEnabled = true;
            robot.ballGate.unlock();
        }

        if (dpad_l && !previousDpadL) {
            ballGateEnabled = false;
            robot.ballGate.lock();
        }


        previousA = a;
        previousB = b;
        previousX = x;
        previousY = y;
        previousDpadL = dpad_l;

        robot.shooter.applyTuning(ShooterTuningConfig.targetVelocityTicksPerSecond,
                ShooterTuningConfig.useCustomPidf, ShooterTuningConfig.kP,
                ShooterTuningConfig.kI, ShooterTuningConfig.kD, ShooterTuningConfig.kF);

        double nowMilliseconds = TimeUtils.monotonicMillisecondsPrecise();
        if (nowMilliseconds - lastTelemetryMilliseconds < TELEMETRY_INTERVAL_MILLISECONDS) return;
        lastTelemetryMilliseconds = nowMilliseconds;
        double measuredTicksPerSecond = robot.shooter.getVelocityTicksPerSecond();
        telemetry.addData("Alvo lido do Panels (ticks/s)", ShooterTuningConfig.targetVelocityTicksPerSecond);
        telemetry.addData("Setpoint aplicado (ticks/s)",
                robot.shooter.getAppliedVelocityTargetTicksPerSecond());
        telemetry.addData("PIDF SolversLib (painel)", "P %.6f I %.6f D %.6f F %.6f",
                ShooterTuningConfig.kP, ShooterTuningConfig.kI,
                ShooterTuningConfig.kD, ShooterTuningConfig.kF);
        telemetry.addData("Medido (ticks/s)", measuredTicksPerSecond);
        telemetry.addData("Erro (ticks/s)", ShooterTuningConfig.targetVelocityTicksPerSecond
                - measuredTicksPerSecond);
        telemetry.addData("Bateria (V)", "%.2f", battery.getVoltage());
        telemetry.addData("Shooter / intake", "%s / %s", shooterEnabled, intakeEnabled);
        telemetry.addData("Trava", robot.ballGate.isLocked() ? "FECHADA" : "ABERTA");
        telemetry.addLine("A shooter | B intake | X para ambos. Teste preso e sem bolas.");
        telemetry.update();
    }

    /** Para intake, shooter e fecha a trava ao sair da bancada. */
    @Override
    public void stop() {
        if (robot != null) {
            robot.gamePieces.cancelToSafe();
            robot.drive.stop();
        }
    }
}
