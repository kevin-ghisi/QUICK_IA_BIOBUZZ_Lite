package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.hardware.MyRobot;
import org.firstinspires.ftc.teamcode.hardware.TurretTuningConfig;
import org.firstinspires.ftc.teamcode.util.TimeUtils;

/** OpMode de bancada para ajustar o PID horizontal da torre usando os campos ao vivo do Panels. */
@TeleOp(name = "SETUP - Turret PID Panels", group = "Setup")
public class TurretPidTuningOpMode extends OpMode {
    private static final double TELEMETRY_INTERVAL_MILLISECONDS = 100.0;
    private MyRobot robot;
    private boolean pidEnabled;
    private boolean previousEnableButton;
    private boolean previousResetButton;
    private double lastTelemetryMilliseconds;

    /** Monta o MyRobot e inicia os mecanismos em posição parada/segura. */
    @Override
    public void init() {
        GlobalConstants.robotMode = GlobalConstants.RobotMode.TELEOP;
        robot = new MyRobot(hardwareMap);
        robot.initializeForOpMode();
        telemetry.setMsTransmissionInterval(100);
        lastTelemetryMilliseconds = TimeUtils.monotonicMillisecondsPrecise();
        telemetry.addLine("Torre parada. Use A para habilitar PID e B para parar.");
        telemetry.addLine("O zero frontal precisa estar calibrado no Robot Setup antes do teste.");
        telemetry.update();
    }

    /** Lê os ganhos configuráveis em cada ciclo e mostra os valores realmente recebidos pelo código. */
    @Override
    public void loop() {
        boolean enablePressed = gamepad1.a;
        boolean resetPressed = gamepad1.x;
        boolean stopPressed = gamepad1.b;

        if (enablePressed && !previousEnableButton && robot.turret.isForwardZeroCalibrated()) {
            pidEnabled = !pidEnabled;
            robot.turret.resetPositionPid();
        }
        if (resetPressed && !previousResetButton) robot.turret.resetPositionPid();
        if (stopPressed) pidEnabled = false;
        previousEnableButton = enablePressed;
        previousResetButton = resetPressed;

        if (pidEnabled) {
            if (!robot.turret.isForwardZeroCalibrated()) pidEnabled = false;
        }
        if (pidEnabled) {
            robot.turret.runPositionPid(TurretTuningConfig.targetAngleDegrees);
        } else {
            robot.turret.stop();
        }

        double nowMilliseconds = TimeUtils.monotonicMillisecondsPrecise();
        if (nowMilliseconds - lastTelemetryMilliseconds < TELEMETRY_INTERVAL_MILLISECONDS) return;
        lastTelemetryMilliseconds = nowMilliseconds;
        telemetry.addData("Zero calibrado", robot.turret.isForwardZeroCalibrated());
        telemetry.addData("PID ativo", pidEnabled);
        telemetry.addData("Alvo Panels (graus)", TurretTuningConfig.targetAngleDegrees);
        telemetry.addData("Ângulo encoder (graus)", robot.turret.getAngleDegrees());
        telemetry.addData("Erro aprox. (graus)", TurretTuningConfig.targetAngleDegrees
                - robot.turret.getAngleDegrees());
        telemetry.addData("PID lido", "P %.5f I %.5f D %.5f",
                TurretTuningConfig.kP, TurretTuningConfig.kI, TurretTuningConfig.kD);
        telemetry.addData("Limite de potência", TurretTuningConfig.maxPower);
        telemetry.addLine("A liga/desliga | B para imediatamente | X limpa histórico PID");
        telemetry.update();
    }

    /** Remove potência da torre e do drive ao sair do modo de teste. */
    @Override
    public void stop() {
        if (robot == null) return;
        pidEnabled = false;
        robot.turret.stop();
        robot.drive.stop();
    }
}
