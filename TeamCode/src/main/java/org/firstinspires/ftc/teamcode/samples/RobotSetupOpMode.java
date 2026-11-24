package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystem.Turret;
import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.MathUtils;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Ferramenta interativa de bancada para testar motores, calibrar a turret e ajustar servos. */
@TeleOp(name = "SETUP - Motores, Turret e Servos", group = "Setup")
public class RobotSetupOpMode extends OpMode {
    private enum TurretHomeState { IDLE, SEEK_LEFT, SEEK_RIGHT, RETURN_CENTER, COMPLETE, FAILED }

    private MotorEx[] motors;
    private ServoEx[] servos;
    private DistanceSensor[] magazineSensors;
    private int selectedMotor;
    private int selectedServo;
    private final double[] servoPositions = new double[GlobalConstants.SETUP_SERVO_NAMES.length];
    private final double[] servoMinimums = new double[GlobalConstants.SETUP_SERVO_NAMES.length];
    private final double[] servoMaximums = new double[GlobalConstants.SETUP_SERVO_NAMES.length];
    private TurretHomeState homeState = TurretHomeState.IDLE;
    private String status = "Pronto: teste os motores ou ajuste o servo.";
    private int leftLimitTicks;
    private int centerTicks;
    private int lastObservedTicks;
    private boolean movedSinceSeekStarted;
    private long seekStartedAt;
    private long lastEncoderMovementAt;
    private long returnStartedAt;
    private final ButtonEdge nextMotorButton = new ButtonEdge();
    private final ButtonEdge previousMotorButton = new ButtonEdge();
    private final ButtonEdge startHomingButton = new ButtonEdge();
    private final ButtonEdge abortHomingButton = new ButtonEdge();
    private final ButtonEdge clearFailureButton = new ButtonEdge();
    private final ButtonEdge nextServoButton = new ButtonEdge();
    private final ButtonEdge previousServoButton = new ButtonEdge();
    private final ButtonEdge servoLeftButton = new ButtonEdge();
    private final ButtonEdge servoRightButton = new ButtonEdge();
    private final ButtonEdge captureMinimumButton = new ButtonEdge();
    private final ButtonEdge captureMaximumButton = new ButtonEdge();

    /** Busca os motores e servos pelos nomes definidos na configuração do Control Hub. */
    @Override
    public void init() {
        motors = new MotorEx[GlobalConstants.SETUP_MOTOR_NAMES.length];
        for (int index = 0; index < motors.length; index++) {
            String motorName = GlobalConstants.SETUP_MOTOR_NAMES[index];
            motors[index] = new MotorEx(hardwareMap, motorName);
            motors[index].setInverted(GlobalConstants.setupMotorDirection(motorName)
                    == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
            motors[index].setRunMode(Motor.RunMode.RawPower);
            motors[index].setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motors[index].set(0);
        }

        servos = new ServoEx[GlobalConstants.SETUP_SERVO_NAMES.length];
        for (int index = 0; index < servos.length; index++) {
            servos[index] = new ServoEx(hardwareMap, GlobalConstants.SETUP_SERVO_NAMES[index]);
            // ServoEx.get() can be NaN until this OpMode has commanded a position.
            // Keep a known starting value without moving the servo during init().
            servoPositions[index] = GlobalConstants.SETUP_SERVO_INITIAL_POSITIONS[index];
            servoMinimums[index] = GlobalConstants.SETUP_SERVO_MIN_POSITIONS[index];
            servoMaximums[index] = GlobalConstants.SETUP_SERVO_MAX_POSITIONS[index];
        }

        magazineSensors = new DistanceSensor[GlobalConstants.SETUP_DISTANCE_SENSOR_NAMES.length];
        for (int index = 0; index < magazineSensors.length; index++) {
            magazineSensors[index] = hardwareMap.get(DistanceSensor.class,
                    GlobalConstants.SETUP_DISTANCE_SENSOR_NAMES[index]);
        }
    }

    /** Exibe os controles e as precauções antes do início do teste. */
    @Override
    public void init_loop() {
        telemetry.addLine("SETUP: D-pad ↑/↓ escolhe motor; joystick esquerdo testa avanço/strafe.");
        telemetry.addLine("Gamepad 1 A inicia homing da turret; B interrompe a sequência.");
        telemetry.addLine("Gamepad 2 D-pad escolhe servo; ←/→ ajusta; A/B registra mínimo/máximo.");
        telemetry.addLine("Sensor da boca do intake: confira distância em mm e limiar no V3 Quick Start.");
        telemetry.addLine("Prenda o robô e mantenha rodas, torre e hood sem obstruções.");
        telemetry.update();
    }

    /** Atualiza seleções, homing, saídas manuais e telemetria a cada ciclo. */
    @Override
    public void loop() {
        updateMotorSelection();
        updateServoSelection();
        updateServoAdjustment();
        updateHomingButtons();

        if (isHoming()) {
            updateTurretHoming();
        } else if (homeState == TurretHomeState.FAILED) {
            stopAllMotors();
        } else {
            applySelectedMotorTest();
        }

        showTelemetry();
    }

    /** Desliga todos os motores quando a equipe encerra o OpMode de bancada. */
    @Override
    public void stop() {
        if (motors == null) return;
        for (MotorEx motor : motors) motor.set(0);
    }

    /** Altera a roda ou mecanismo que receberá o comando do joystick. */
    private void updateMotorSelection() {
        if (isHoming()) return;
        if (nextMotorButton.rising(gamepad1.dpad_down)) {
            selectedMotor = (selectedMotor + 1) % motors.length;
        }
        if (previousMotorButton.rising(gamepad1.dpad_up)) {
            selectedMotor = (selectedMotor - 1 + motors.length) % motors.length;
        }
    }

    /** Seleciona um servo sem alterar os demais. */
    private void updateServoSelection() {
        if (nextServoButton.rising(gamepad2.dpad_down)) {
            selectedServo = (selectedServo + 1) % servos.length;
        }
        if (previousServoButton.rising(gamepad2.dpad_up)) {
            selectedServo = (selectedServo - 1 + servos.length) % servos.length;
        }
    }

    /** Ajusta o servo em passos curtos e grava limites candidatos para copiar à configuração global. */
    private void updateServoAdjustment() {
        ServoEx servo = servos[selectedServo];
        if (servoLeftButton.rising(gamepad2.dpad_left)) {
            servoPositions[selectedServo] = MathUtils.clamp(
                    servoPositions[selectedServo] - GlobalConstants.SETUP_SERVO_STEP, 0.0, 1.0);
            servo.set(servoPositions[selectedServo]);
        }
        if (servoRightButton.rising(gamepad2.dpad_right)) {
            servoPositions[selectedServo] = MathUtils.clamp(
                    servoPositions[selectedServo] + GlobalConstants.SETUP_SERVO_STEP, 0.0, 1.0);
            servo.set(servoPositions[selectedServo]);
        }
        if (captureMinimumButton.rising(gamepad2.a)) {
            servoMinimums[selectedServo] = servoPositions[selectedServo];
        }
        if (captureMaximumButton.rising(gamepad2.b)) {
            servoMaximums[selectedServo] = servoPositions[selectedServo];
        }
    }

    /** Inicia ou interrompe o homing; o processo só valida o zero após achar os dois batentes. */
    private void updateHomingButtons() {
        if (startHomingButton.rising(gamepad1.a) && !isHoming()) startTurretHoming();
        if (abortHomingButton.rising(gamepad1.b) && isHoming()) failHoming("Interrompido pelo operador.");
        if (clearFailureButton.rising(gamepad1.y) && homeState == TurretHomeState.FAILED) {
            homeState = TurretHomeState.IDLE;
            status = "Falha reconhecida; testes manuais liberados.";
        }
    }

    /** Começa a busca lenta pelo batente esquerdo e invalida qualquer zero anterior. */
    private void startTurretHoming() {
        stopAllMotors();
        Turret.clearForwardZeroCalibration();
        turretMotor().motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        status = "Buscando limite esquerdo da turret...";
        beginLimitSearch(TurretHomeState.SEEK_LEFT, -GlobalConstants.TURRET_HOME_POWER);
    }

    /** Executa a máquina de estados: limite esquerdo, limite direito e retorno ao centro. */
    private void updateTurretHoming() {
        if (homeState == TurretHomeState.SEEK_LEFT || homeState == TurretHomeState.SEEK_RIGHT) {
            updateLimitSearch();
        } else if (homeState == TurretHomeState.RETURN_CENTER) {
            updateCenterReturn();
        }
    }

    /** Liga o motor com baixa potência e inicia os temporizadores de movimento/limite. */
    private void beginLimitSearch(TurretHomeState direction, double power) {
        MotorEx motor = turretMotor();
        homeState = direction;
        movedSinceSeekStarted = false;
        seekStartedAt = System.currentTimeMillis();
        lastEncoderMovementAt = seekStartedAt;
        lastObservedTicks = motor.getCurrentPosition();
        motor.set(power);
    }

    /** Detecta o batente pela ausência de pulsos depois que o encoder já registrou deslocamento. */
    private void updateLimitSearch() {
        MotorEx motor = turretMotor();
        long now = System.currentTimeMillis();
        int currentTicks = motor.getCurrentPosition();

        if (Math.abs(currentTicks - lastObservedTicks) >= GlobalConstants.TURRET_HOME_ENCODER_MOVEMENT_TICKS) {
            lastObservedTicks = currentTicks;
            movedSinceSeekStarted = true;
            lastEncoderMovementAt = now;
        }
        if (now - seekStartedAt > GlobalConstants.TURRET_HOME_LIMIT_TIMEOUT_MILLISECONDS) {
            failHoming("Tempo máximo excedido procurando um batente.");
            return;
        }
        if (!movedSinceSeekStarted
                && now - seekStartedAt > GlobalConstants.TURRET_HOME_NO_MOTION_TIMEOUT_MILLISECONDS) {
            failHoming("Encoder sem movimento; confira motor, encoder e sentido.");
            return;
        }
        if (movedSinceSeekStarted
                && now - lastEncoderMovementAt >= GlobalConstants.TURRET_HOME_STALL_MILLISECONDS) {
            onLimitReached(currentTicks);
        }
    }

    /** Guarda o primeiro limite; ao encontrar o segundo, valida curso e calcula o ponto médio. */
    private void onLimitReached(int currentTicks) {
        MotorEx motor = turretMotor();
        motor.set(0);

        if (homeState == TurretHomeState.SEEK_LEFT) {
            leftLimitTicks = currentTicks;
            status = "Limite esquerdo salvo; buscando limite direito...";
            beginLimitSearch(TurretHomeState.SEEK_RIGHT, GlobalConstants.TURRET_HOME_POWER);
            return;
        }

        int rightLimitTicks = currentTicks;
        // Convertemos antes de subtrair para evitar overflow na diferença entre posições int.
        long totalTravelTicks = Math.abs((long) rightLimitTicks - leftLimitTicks);
        double travelDegrees = totalTravelTicks * 360.0 / GlobalConstants.TURRET_MOTOR_TICKS_PER_OUTPUT_REV;
        if (travelDegrees < GlobalConstants.TURRET_HOME_MIN_TRAVEL_DEGREES
                || travelDegrees > GlobalConstants.TURRET_HOME_MAX_TRAVEL_DEGREES) {
            failHoming(String.format("Curso medido %.1f° fora da faixa configurada.", travelDegrees));
            return;
        }

        // Math.round(double) devolve long; Math.toIntExact faz a conversão explícita para o alvo do SDK.
        // Somamos como double para não estourar int antes de calcular a média dos dois batentes.
        centerTicks = Math.toIntExact(Math.round(((double) leftLimitTicks + rightLimitTicks) / 2.0));
        motor.motor.setTargetPosition(centerTicks);
        motor.motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        motor.set(GlobalConstants.TURRET_HOME_CENTER_POWER);
        returnStartedAt = System.currentTimeMillis();
        homeState = TurretHomeState.RETURN_CENTER;
        status = String.format("Curso %.1f°; retornando ao centro (%d ticks)...", travelDegrees, centerTicks);
    }

    /** Aguarda a torre alcançar o meio dos dois batentes e zera o encoder no Control Hub. */
    private void updateCenterReturn() {
        MotorEx motor = turretMotor();
        int ticksFromCenter = Math.abs(motor.getCurrentPosition() - centerTicks);
        boolean reachedCenter = ticksFromCenter <= GlobalConstants.TURRET_HOME_CENTER_TOLERANCE_TICKS
                || !motor.motor.isBusy();
        if (reachedCenter) {
            motor.set(0);
            motor.stopAndResetEncoder();
            motor.motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Turret.markForwardZeroCalibrated();
            homeState = TurretHomeState.COMPLETE;
            status = "Homing concluído: encoder zero no centro dos batentes.";
        } else if (System.currentTimeMillis() - returnStartedAt
                > GlobalConstants.TURRET_HOME_LIMIT_TIMEOUT_MILLISECONDS) {
            failHoming("A turret não retornou ao centro dentro do tempo limite.");
        }
    }

    /** Interrompe o motor e invalida o zero se a sequência não for concluída. */
    private void failHoming(String reason) {
        stopAllMotors();
        MotorEx motor = turretMotor();
        motor.motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.clearForwardZeroCalibration();
        homeState = TurretHomeState.FAILED;
        status = "Homing falhou: " + reason;
    }

    /** Testa a roda ou mecanismo selecionado, limitando a potência da bancada. */
    private void applySelectedMotorTest() {
        for (int index = 0; index < motors.length; index++) {
            if (index != selectedMotor) motors[index].set(0);
        }

        String name = GlobalConstants.SETUP_MOTOR_NAMES[selectedMotor];
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double requestedPower = isDriveMotor(name)
                ? forward + strafe * GlobalConstants.driveStrafeSign(name)
                        + turn * GlobalConstants.driveTurnSign(name)
                : forward;
        motors[selectedMotor].set(MathUtils.clamp(requestedPower,
                -GlobalConstants.DRIVE_SETUP_MAX_POWER, GlobalConstants.DRIVE_SETUP_MAX_POWER));
    }

    /** Para todas as saídas antes de abortar uma rotina ou proteger a bancada. */
    private void stopAllMotors() {
        for (MotorEx motor : motors) motor.set(0);
    }

    /** Identifica as quatro rodas do chassi pelos sinais de strafe configurados. */
    private boolean isDriveMotor(String name) {
        return GlobalConstants.driveStrafeSign(name) != 0;
    }

    /** Devolve o motor da turret que já foi obtido em init(), sem buscar hardware a cada ciclo. */
    private MotorEx turretMotor() {
        for (int index = 0; index < motors.length; index++) {
            if (GlobalConstants.TURRET_MOTOR_NAME.equals(GlobalConstants.SETUP_MOTOR_NAMES[index])) {
                return motors[index];
            }
        }
        throw new IllegalStateException("O motor da turret não está na lista de motores do setup.");
    }

    /** Indica se a máquina de homing ainda está comandando o motor da torre. */
    private boolean isHoming() {
        return homeState == TurretHomeState.SEEK_LEFT
                || homeState == TurretHomeState.SEEK_RIGHT
                || homeState == TurretHomeState.RETURN_CENTER;
    }

    /** Mostra no Driver Station os estados, medições e valores que devem ser copiados à configuração. */
    private void showTelemetry() {
        String motorName = GlobalConstants.SETUP_MOTOR_NAMES[selectedMotor];
        String servoName = GlobalConstants.SETUP_SERVO_NAMES[selectedServo];
        telemetry.addData("Estado", status);
        telemetry.addData("Turret zero válido", Turret.isCalibrationValid());
        telemetry.addData("Motor selecionado", "%s (%d/%d)", motorName, selectedMotor + 1, motors.length);
        telemetry.addData("Potência aplicada", "%.2f", motors[selectedMotor].get());
        telemetry.addData("Encoder turret", turretMotor().getCurrentPosition());
        telemetry.addData("Curso medido", "%.1f°", Math.abs(turretMotor().getCurrentPosition() - leftLimitTicks)
                * 360.0 / GlobalConstants.TURRET_MOTOR_TICKS_PER_OUTPUT_REV);
        telemetry.addData("Servo selecionado", "%s (%d/%d)", servoName, selectedServo + 1, servos.length);
        telemetry.addData("Posição servo", "%.3f", servoPositions[selectedServo]);
        telemetry.addData("Mínimo / máximo salvos", "%.3f / %.3f",
                servoMinimums[selectedServo], servoMaximums[selectedServo]);
        for (int index = 0; index < magazineSensors.length; index++) {
            telemetry.addData("Distância " + GlobalConstants.SETUP_DISTANCE_SENSOR_NAMES[index] + " (mm)",
                    "%.1f", magazineSensors[index].getDistance(DistanceUnit.MM));
        }
        telemetry.addLine("Motor: D-pad ↑/↓ seleciona; joystick esquerdo testa avanço/strafe; direito gira.");
        telemetry.addLine("Turret: gamepad 1 A inicia; B cancela. Supervisione em área livre.");
        telemetry.addLine("Após falha, Y reconhece o estado; A tenta homing novamente.");
        telemetry.addData("Controles servo", "D-pad ↑/↓ escolhe; ←/→ muda %.2f; A/B salva min/máx",
                GlobalConstants.SETUP_SERVO_STEP);
        telemetry.update();
    }

    /** Detecta uma nova pressão para que cada ajuste execute uma única vez, sem repetição por ciclo. */
    private static final class ButtonEdge {
        private boolean wasPressed;

        private boolean rising(boolean pressed) {
            boolean justPressed = pressed && !wasPressed;
            wasPressed = pressed;
            return justPressed;
        }
    }
}
