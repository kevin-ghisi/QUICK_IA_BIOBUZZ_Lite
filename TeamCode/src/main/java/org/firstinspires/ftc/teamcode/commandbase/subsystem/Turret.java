package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.hardware.TurretEncoderDiagnosticConfig;
import org.firstinspires.ftc.teamcode.hardware.TurretTuningConfig;
import org.firstinspires.ftc.teamcode.util.FieldTargetSelector;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 * Subsistema que gira a torre apenas no plano horizontal.
 * O encoder precisa estar zerado com a torre alinhada para a frente do robô.
 *
 *                       ALVO
 *                         *
 *                         |
 * ESQUERDA <---------------+---------------> DIREITA
 *                         |
 *                         ^
 *                         |
 *                    FRENTE DO ROBÔ
 *
 * O ângulo de campo aponta do robô até o alvo. Subtrair o heading do robô converte esse
 * ângulo global em uma rotação relativa à carcaça, que é o que o motor da torre deve executar.
 */
public final class Turret extends SubsystemBase {
    // A validade dura entre OpModes do mesmo processo; o setup também zera o encoder físico no centro.
    private static volatile boolean forwardZeroCalibrated;

    private final Motor motor; // Motor e encoder encapsulados pelo wrapper de hardware do SolversLib.
    private final DcMotorEx verificationEncoder; // REV Through Bore no canal de encoder da porta intake.
    private int verificationZeroTicks;
    private final FieldTargetSelector targetSelector = new FieldTargetSelector();
    private boolean autoAimEnabled; // A mira automática começa desligada para impedir movimento inesperado.
    private double targetX; // Coordenada X do alvo escolhido, exibida na telemetria.
    private double targetY; // Coordenada Y do alvo após escolher aliança e refletir o lado do campo.
    private double targetTurretAngleDegrees; // Ângulo relativo pedido para a torre, limitado mecanicamente.
    private double targetGlobalBearingDegrees; // Direção do alvo no referencial do campo.
    private double pidPreviousTargetDegrees = Double.NaN;
    private double lastAppliedPidPower;
    private final PIDController positionPid = new PIDController(
            TurretTuningConfig.kP, TurretTuningConfig.kI, TurretTuningConfig.kD);
    private final PIDController aimPid = new PIDController(
            TurretTuningConfig.kP, TurretTuningConfig.kI, TurretTuningConfig.kD);

    /** Cria o motor, seleciona controle aberto e configura sentido e escala do encoder. */
    public Turret(HardwareMap hardwareMap) {
        motor = new Motor(hardwareMap, GlobalConstants.TURRET_MOTOR_NAME); // Busca o motor pelo nome configurado.
        verificationEncoder = hardwareMap.get(
                DcMotorEx.class, GlobalConstants.TURRET_THROUGH_BORE_ENCODER_NAME);
        motor.setRunMode(Motor.RunMode.RawPower); // O loop proporcional deste subsistema calcula a potência.
        motor.setZeroPowerBehavior(GlobalConstants.TURRET_ZERO_POWER); // Freia a torre quando a correção chega a zero.
        motor.setInverted(GlobalConstants.TURRET_MOTOR_REVERSED); // Ajusta o sinal do motor/encoder no wrapper SolversLib.
        // A escala/sinal do Through Bore foi conferida lendo o dispositivo sem inversão: esquerda é positiva.
        // Como a inversão do motor também inverte a posição vista pelo SDK, desfazemos só essa inversão
        // no encoder SolversLib para manter ângulo positivo = torre girando para a esquerda.
        motor.encoder.setDirection(Motor.Direction.REVERSE);
        motor.encoder.setDistancePerPulse(
                360.0 / GlobalConstants.TURRET_MOTOR_TICKS_PER_OUTPUT_REV); // Escala o encoder integrado do HD Hex.
    }

    /** Para a torre ao preparar o OpMode; não inicia homing nem mira sozinha. */
    public void initialize() {
        stop(); // Garante potência zero ao selecionar TeleOp ou Autônomo.
    }

    /** Controla manualmente a torre enquanto uma das duas direções do D-pad estiver pressionada. */
    public void setManual(boolean up, boolean down) {
        autoAimEnabled = false; // O comando manual sempre assume prioridade sobre a mira automática.
        if (up == down) { // Se os dois botões têm o mesmo estado, não existe direção única pedida.
            stop(); // Remove a potência que poderia ter sido enviada no ciclo anterior.
            return; // Encerra o método sem aplicar movimento.
        }
        double power = up ? GlobalConstants.TURRET_MANUAL_POWER : -GlobalConstants.TURRET_MANUAL_POWER; // Converte o botão em sinal.
        motor.set(limitPowerAtSoftStops(power)); // Aplica o comando sem empurrar a torre contra um limite calibrado.
    }

    /**
     * Controla a torre com potência analógica, por exemplo usando o eixo horizontal do joystick.
     * O valor é limitado a [-1, 1], escala pela potência manual configurada e respeita os batentes
     * de software quando o zero frontal já foi calibrado.
     */
    public void setManualPower(double power) {
        autoAimEnabled = false; // Movimento manual sempre desativa a mira automática.
        double safePower = MathUtils.clamp(power, -1.0, 1.0); // Evita valores fora da faixa válida do motor.
        if (Math.abs(safePower) < 0.05) { // Zona morta evita deriva com ruído do joystick.
            stop(); // Para a torre ao soltar o controle.
            return; // Não envia potência residual.
        }
        motor.set(limitPowerAtSoftStops(safePower * GlobalConstants.TURRET_MANUAL_POWER));
    }

    /** Aplica potência calculada por um controlador externo, mantendo calibração e limites seguros. */
    public void setExternalControllerPower(double power) {
        if (!autoAimEnabled || !forwardZeroCalibrated || !Double.isFinite(power)) {
            stop();
            return;
        }
        motor.set(limitPowerAtSoftStops(MathUtils.clamp(power, -1.0, 1.0)));
    }

    /**
     * Executa um controlador PID de posição para o OpMode de tuning.
     * O alvo fica limitado aos batentes configurados; o integral é limitado para evitar windup.
     * O método exige zero frontal calibrado antes de permitir movimento automático.
     */
    public void runPositionPid(double targetDegrees) {
        if (!forwardZeroCalibrated) { // Sem homing, posição e limites não têm referência física segura.
            stop();
            resetPositionPid();
            return;
        }

        double safeTarget = MathUtils.clamp(targetDegrees,
                -GlobalConstants.TURRET_MAX_ANGLE_DEGREES,
                GlobalConstants.TURRET_MAX_ANGLE_DEGREES);
        // Mudou o alvo? Zera memória antiga para não carregar erro integral de outro movimento.
        if (Double.compare(safeTarget, pidPreviousTargetDegrees) != 0) {
            positionPid.reset();
        }

        double errorDegrees = safeTarget - getControlAngleDegrees();
        if (Math.abs(errorDegrees) <= Math.abs(TurretTuningConfig.toleranceDegrees)) {
            stop();
            positionPid.reset(); // Evita carregar integral entre movimentos separados.
            pidPreviousTargetDegrees = safeTarget;
            return;
        }

        // PIDController da SolversLib mede encoder em graus e calcula a saída do motor.
        positionPid.setPID(TurretTuningConfig.kP, TurretTuningConfig.kI, TurretTuningConfig.kD);
        positionPid.integrationControl.setIntegrationBounds(
                -Math.abs(TurretTuningConfig.integralLimitDegreesSeconds),
                Math.abs(TurretTuningConfig.integralLimitDegreesSeconds));
        positionPid.setTolerance(Math.abs(TurretTuningConfig.toleranceDegrees));
        double output = positionPid.calculate(getControlAngleDegrees(), safeTarget);
        applyPidPower(output);

        pidPreviousTargetDegrees = safeTarget;
    }

    /** Limpa integral, derivada e histórico de tempo antes de uma nova sessão de tuning. */
    public void resetPositionPid() {
        positionPid.reset();
        aimPid.reset();
        pidPreviousTargetDegrees = Double.NaN;
    }

    /** Zera o encoder no alinhamento frontal feito manualmente durante o teste de bancada. */
    public void calibrateForwardZero() {
        stop(); // Evita que a torre se mova enquanto o zero é gravado.
        motor.stopAndResetEncoder(); // Grava zero no controlador do motor na posição frontal atual.
        motor.encoder.setDistancePerPulse(
                360.0 / GlobalConstants.TURRET_MOTOR_TICKS_PER_OUTPUT_REV);
        verificationZeroTicks = verificationEncoder.getCurrentPosition();
        markForwardZeroCalibrated(); // Libera limites e auto-aim até o processo do Control Hub ser reiniciado.
    }

    /** Marca como válido o zero que o OpMode de setup calculou no meio entre os dois limites. */
    public static void markForwardZeroCalibrated() {
        forwardZeroCalibrated = true; // O OpMode de setup chama isso somente depois de homing bem-sucedido.
    }

    /** Invalida o zero quando um homing é interrompido ou falha. */
    public static void clearForwardZeroCalibration() {
        forwardZeroCalibrated = false; // Um encoder sem centro conhecido não pode limitar nem mirar com segurança.
    }

    /** Informa ao OpMode de setup e à telemetria se o homing já foi concluído. */
    public static boolean isCalibrationValid() {
        return forwardZeroCalibrated; // O zero é compartilhado por OpModes enquanto o Control Hub app fica aberto.
    }

    /** Informa se existe uma referência frontal conhecida para o encoder. */
    public boolean isForwardZeroCalibrated() {
        return forwardZeroCalibrated; // Telemetria pode orientar a pessoa durante a configuração.
    }

    /** Liga a mira somente quando o zero do encoder é válido; ao desligar, para o motor. */
    public void setAutoAimEnabled(boolean enabled) {
        autoAimEnabled = enabled && forwardZeroCalibrated; // Ignora o pedido de ligar enquanto não houver calibração.
        if (!autoAimEnabled) stop(); // Evita que a torre continue andando depois de desligar o modo.
    }

    /** Informa se o operador deixou a mira automática ativa. */
    public boolean isAutoAimEnabled() {
        return autoAimEnabled; // Este estado é usado pelo TeleOp para escolher controle manual ou automático.
    }

    /** Calcula e aplica a correção necessária para apontar ao alvo da aliança. */
    public void aimAt(Pose robotPose) {
        GlobalConstants.FieldTarget target = targetSelector.update(robotPose.y());
        aimAtFieldTarget(robotPose, target.x, target.y);
    }

    /** Mira para uma coordenada fixa da arena, sem trocar o alvo conforme o lado do robô. */
    public void aimAtFieldTarget(Pose robotPose, double fieldTargetX, double fieldTargetY) {
        if (!autoAimEnabled || !forwardZeroCalibrated) { // Sem modo automático ou referência, não existe ângulo confiável.
            stop(); // A condição inválida sempre resulta em motor parado.
            return; // Evita qualquer cálculo baseado em encoder sem zero.
        }

        targetX = fieldTargetX; // Guarda o ponto fixo para conferir a configuração na telemetria.
        targetY = fieldTargetY;

        double deltaX = targetX - robotPose.x();
        double deltaY = targetY - robotPose.y();
        double fieldBearingRadians = Math.atan2(deltaY, deltaX); // Usa somente a pose atual do Pinpoint.
        targetGlobalBearingDegrees = Math.toDegrees(MathUtils.wrapRadians(fieldBearingRadians));
        double relativeBearingRadians = MathUtils.wrapRadians(fieldBearingRadians - robotPose.heading()); // Remove a rotação atual do robô.
        double relativeBearingDegrees = Math.toDegrees(relativeBearingRadians); // Converte radianos para a unidade do encoder.
        targetTurretAngleDegrees = MathUtils.clamp(relativeBearingDegrees,
                -GlobalConstants.TURRET_MAX_ANGLE_DEGREES,
                GlobalConstants.TURRET_MAX_ANGLE_DEGREES); // Limita o setpoint para não pedir volta além dos batentes.

        double angleError = targetTurretAngleDegrees - getControlAngleDegrees(); // Erro corrigido pela verificação externa.
        if (Math.abs(angleError) <= Math.abs(TurretTuningConfig.toleranceDegrees)) { // Compara erro com a faixa de aceitação.
            stop(); // Para a torre quando já está apontada para evitar vibração.
            return; // Não envia correção desnecessária.
        }

        aimPid.setPID(TurretTuningConfig.kP, TurretTuningConfig.kI, TurretTuningConfig.kD);
        aimPid.integrationControl.setIntegrationBounds(
                -Math.abs(TurretTuningConfig.integralLimitDegreesSeconds),
                Math.abs(TurretTuningConfig.integralLimitDegreesSeconds));
        applyPidPower(aimPid.calculate(getControlAngleDegrees(), targetTurretAngleDegrees));
    }

    /** Lê a posição atual do eixo em graus desde o zero frontal. */
    public double getAngleDegrees() {
        return motor.getDistance(); // A escala em graus foi configurada no construtor.
    }

    /** Ângulo relativo do REV Through Bore, usado para conferir/corrigir o encoder integrado. */
    public double getVerificationAngleDegrees() {
        double countsPerOutputRev = Math.abs(TurretEncoderDiagnosticConfig.countsPerTurretRevolution);
        if (!Double.isFinite(countsPerOutputRev) || countsPerOutputRev <= 0.0) return Double.NaN;
        return (verificationEncoder.getCurrentPosition() - verificationZeroTicks)
                * 360.0 / countsPerOutputRev
                * Math.signum(TurretEncoderDiagnosticConfig.encoderSign);
    }

    /** Usa o encoder integrado como base e aplica a diferença medida pelo Through Bore. */
    public double getControlAngleDegrees() {
        double motorAngle = getAngleDegrees();
        double verificationAngle = getVerificationAngleDegrees();
        if (!Double.isFinite(motorAngle) || !Double.isFinite(verificationAngle)) return motorAngle;
        return motorAngle + (verificationAngle - motorAngle);
    }

    /** Lê o ângulo relativo que o cálculo de mira está solicitando. */
    public double getTargetAngleDegrees() {
        return targetTurretAngleDegrees; // O TeleOp exibe este número ao lado do ângulo medido.
    }

    /** Direção global do alvo no campo, em graus, normalizada para [-180, 180]. */
    public double getTargetGlobalBearingDegrees() {
        return targetGlobalBearingDegrees;
    }

    /** Estima a direção global atual da turret a partir do heading do chassi e do encoder relativo. */
    public double getCurrentGlobalBearingDegrees(double robotHeadingRadians) {
        return Math.toDegrees(MathUtils.wrapRadians(
                robotHeadingRadians + Math.toRadians(getControlAngleDegrees())));
    }

    /** Setpoint angular convertido para ticks do encoder a partir do zero frontal. */
    public double getTargetEncoderTicks() {
        return targetTurretAngleDegrees * GlobalConstants.TURRET_MOTOR_TICKS_PER_OUTPUT_REV / 360.0;
    }

    /** Potência PID aplicada no último ciclo, útil para depurar todos os modos com a mesma lógica. */
    public double getLastAppliedPidPower() {
        return lastAppliedPidPower;
    }

    /** Confirma se a torre calibrada está dentro da tolerância angular do alvo calculado. */
    public boolean isAtTarget() {
        return forwardZeroCalibrated
                && Math.abs(targetTurretAngleDegrees - getControlAngleDegrees())
                <= Math.abs(TurretTuningConfig.toleranceDegrees);
    }

    /** Devolve a coordenada X atual do alvo da aliança. */
    public double getTargetX() {
        return targetX; // Útil para conferir a seleção de campo/aliança.
    }

    /** Devolve a coordenada Y atual do alvo, já refletida se o robô cruzou a linha central. */
    public double getTargetY() {
        return targetY; // Útil para conferir em telemetria a troca de metade do campo.
    }

    /** Para o motor da torre com potência zero. */
    public void stop() {
        lastAppliedPidPower = 0.0;
        motor.set(0); // Não deixa a última potência manual ou automática permanecer ativa.
    }

    /** Limita a saída PID usando a configuração única compartilhada por todos os OpModes. */
    private void applyPidPower(double requestedPower) {
        if (!Double.isFinite(requestedPower)) {
            stop();
            return;
        }
        double maxPower = MathUtils.clamp(Math.abs(TurretTuningConfig.maxPower), 0.0, 1.0);
        double minPower = Math.min(
                MathUtils.clamp(Math.abs(TurretTuningConfig.minPower), 0.0, 1.0), maxPower);
        double output = MathUtils.clamp(requestedPower, -maxPower, maxPower);
        if (Math.abs(output) > 1e-6 && Math.abs(output) < minPower) {
            output = Math.copySign(minPower, output);
        }
        lastAppliedPidPower = limitPowerAtSoftStops(output);
        motor.set(lastAppliedPidPower);
    }

    /** Impede comandos que aumentariam o ângulo para fora dos limites de software. */
    private double limitPowerAtSoftStops(double power) {
        if (!forwardZeroCalibrated) return power; // Sem referência, os limites numéricos não têm significado físico.
        double currentAngle = getControlAngleDegrees(); // Usa o ângulo verificado para proteger os limites.
        boolean pushesPastRightStop = currentAngle >= GlobalConstants.TURRET_MAX_ANGLE_DEGREES && power > 0;
        boolean pushesPastLeftStop = currentAngle <= -GlobalConstants.TURRET_MAX_ANGLE_DEGREES && power < 0;
        if (pushesPastRightStop || pushesPastLeftStop) return 0; // Bloqueia só o movimento que empurra contra o batente.
        return power; // Ainda permite voltar para dentro da área segura.
    }
}
