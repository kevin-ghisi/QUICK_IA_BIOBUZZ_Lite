package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.hardware.TurretEncoderDiagnosticConfig;

/**
 * Leitura passiva para validar o Through Bore na torre. Não envia potência nem altera o modo
 * dos controladores; use o transferidor e ajuste o ângulo físico no Panels.
 */
@TeleOp(name = "TESTE - Encoder Through Bore torre", group = "Testes")
public final class TurretThroughBoreDiagnosticOpMode extends OpMode {
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    /** Encoder integrado do HD Hex, usado pelo controlador da torre. */
    private DcMotorEx turretMotorEncoder;
    /** Through Bore ligado à entrada de encoder da porta intake, que não usa seu encoder de motor. */
    private DcMotorEx throughBoreEncoder;
    private int motorZeroTicks;
    private int throughBoreZeroTicks;
    private boolean zeroCaptured;
    private boolean previousZeroButton;

    @Override
    public void init() {
        turretMotorEncoder = hardwareMap.get(DcMotorEx.class, GlobalConstants.TURRET_MOTOR_NAME);
        throughBoreEncoder = hardwareMap.get(
                DcMotorEx.class, GlobalConstants.TURRET_THROUGH_BORE_ENCODER_NAME);
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("Somente leitura: nenhum motor será acionado.");
        telemetry.addLine("HD Hex integrado = controle | Through Bore na porta de encoder 'intake' = verificação.");
        telemetry.addLine("Alinhe a torre para frente e pressione A para gravar os dois zeros lógicos.");
        telemetry.addLine("Gire manualmente e compare os ângulos; nenhum motor recebe potência.");
        panelsTelemetry.debug("Somente leitura: nenhum motor será acionado.");
        panelsTelemetry.debug("HD Hex integrado e Through Bore são mostrados separadamente.");
        panelsTelemetry.debug("A: gravar zero lógico dos dois encoders com a torre à frente.");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        boolean zeroButton = gamepad1.a;
        int motorTicks = turretMotorEncoder.getCurrentPosition();
        int throughBoreTicks = throughBoreEncoder.getCurrentPosition();
        if (zeroButton && !previousZeroButton) {
            motorZeroTicks = motorTicks;
            throughBoreZeroTicks = throughBoreTicks;
            zeroCaptured = true;
        }
        previousZeroButton = zeroButton;

        double externalCountsPerRevolution = Math.abs(TurretEncoderDiagnosticConfig.countsPerTurretRevolution);
        double motorAngleDegrees = Double.NaN;
        double throughBoreAngleDegrees = Double.NaN;
        double correctionDegrees = Double.NaN;
        if (externalCountsPerRevolution > 0.0 && zeroCaptured) {
            motorAngleDegrees = (motorTicks - motorZeroTicks)
                    * 360.0 / GlobalConstants.TURRET_MOTOR_TICKS_PER_OUTPUT_REV
                    * (GlobalConstants.TURRET_MOTOR_REVERSED ? -1.0 : 1.0);
            throughBoreAngleDegrees = (throughBoreTicks - throughBoreZeroTicks)
                    * 360.0 / externalCountsPerRevolution
                    * -Math.signum(TurretEncoderDiagnosticConfig.encoderSign);
            correctionDegrees = throughBoreAngleDegrees - motorAngleDegrees;
        }

        telemetry.addData("Encoder integrado HD Hex (ticks / graus)", "%d / %.2f", motorTicks, motorAngleDegrees);
        telemetry.addData("REV Through Bore (ticks / graus)", "%d / %.2f", throughBoreTicks, throughBoreAngleDegrees);
        telemetry.addData("Zeros capturados", "%d / %d | %s", motorZeroTicks, throughBoreZeroTicks, zeroCaptured);
        telemetry.addData("Escalas (ticks por volta de saída)", "motor %.2f | Through Bore %.2f",
                GlobalConstants.TURRET_MOTOR_TICKS_PER_OUTPUT_REV, externalCountsPerRevolution);
        telemetry.addData("Correção externa - motor (graus)", "%.2f", correctionDegrees);
        telemetry.addData("Ângulo físico Panels (graus)", "%.2f",
                TurretEncoderDiagnosticConfig.measuredPhysicalAngleDegrees);
        telemetry.addData("Erro Through Bore - físico (graus)", "%.2f",
                throughBoreAngleDegrees - TurretEncoderDiagnosticConfig.measuredPhysicalAngleDegrees);
        telemetry.addLine("A: capturar os dois zeros com a torre à frente. Stop para encerrar.");
        panelsTelemetry.debug("HD Hex motor ticks / graus: " + motorTicks + " / " + motorAngleDegrees);
        panelsTelemetry.debug("REV Through Bore ticks / graus: " + throughBoreTicks + " / " + throughBoreAngleDegrees);
        panelsTelemetry.debug("Diferença de correção (graus): " + correctionDegrees);
        panelsTelemetry.debug("Ângulo físico Panels (graus): "
                + TurretEncoderDiagnosticConfig.measuredPhysicalAngleDegrees);
        panelsTelemetry.debug("Erro Through Bore - físico (graus): "
                + (throughBoreAngleDegrees - TurretEncoderDiagnosticConfig.measuredPhysicalAngleDegrees));
        panelsTelemetry.update(telemetry);
    }
}
