package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


/**
 * Configuração editável do robô, separada em blocos por mecanismo.
 * As configurações Pedro (Mecanum, Pinpoint e Foresight) ficam em pedroPathing/Constants.java.
 */
public final class GlobalConstants {
    private GlobalConstants() { }

    // ============================================================================
    // MODO E ALIANÇA
    // ============================================================================

    /** Cor da aliança escolhida na tela de inicialização dos OpModes. */
    public enum Alliance { BLUE, RED }

    /** Uma das duas posições iniciais selecionáveis para o sample de autônomo. */
    public enum StartPosition { POSITION_1, POSITION_2 }

    /** Indica qual fase do código está ativa para os mecanismos que precisarem saber. */
    public enum RobotMode { AUTONOMOUS, TELEOP }

    /** Estado compartilhado entre OpModes enquanto o Robot Controller continua aberto. */
    public static volatile Alliance alliance = Alliance.BLUE;
    /** Posição escolhida no init do autônomo; não altera o TeleOp. */
    public static volatile StartPosition startPosition = StartPosition.POSITION_1;
    /** O OpMode ativo atualiza esse valor em initialize(). */
    public static volatile RobotMode robotMode = RobotMode.TELEOP;

    // ============================================================================
    // CAMPO E ALVOS
    // ============================================================================

    /** Dimensão do campo usada pela simetria de coordenadas, em polegadas. */
    public static final double FIELD_SIZE_INCHES = 141.5; //PEDRO INDICA 141.5
    /** Linha que divide a arena nos lados Y positivo e Y negativo. */
    public static final double FIELD_MID_Y = FIELD_SIZE_INCHES / 2.0;

    /** Linha usada para escolher o alvo esquerdo/direito da turret. */
    /** Alvo de exemplo da aliança azul; substitua pela coordenada oficial da temporada. */
    public static final double BLUE_TARGET_X = 84.0;
    public static final double BLUE_TARGET_Y = 85.25;
    /** Alvo de exemplo da aliança vermelha; substitua pela coordenada oficial da temporada. */
    public static final double RED_TARGET_X = 58.0;
    public static final double RED_TARGET_Y = 85.25;


    // ============================================================================
    // DRIVE MECANUM — CONTROLES DO OPMode DE SETUP
    // ============================================================================

    public static final double DRIVE_SETUP_MAX_POWER = 0.50;

    /**
     * Sinal de strafe para testar uma roda de cada vez com o padrão Mecanum convencional.
     * Frente: todas recebem positivo. Strafe para a esquerda: LF/RR positivo e LR/RF negativo.
     */
    public static int driveStrafeSign(String motorName) {
        if (Constants.FRONT_LEFT_MOTOR_NAME.equals(motorName)
                || Constants.BACK_RIGHT_MOTOR_NAME.equals(motorName)) return 1;
        if (Constants.BACK_LEFT_MOTOR_NAME.equals(motorName)
                || Constants.FRONT_RIGHT_MOTOR_NAME.equals(motorName)) return -1;
        return 0;
    }

    /** Sinal de rotação de cada roda para o teste individual de giro do chassi. */
    public static int driveTurnSign(String motorName) {
        if (Constants.FRONT_LEFT_MOTOR_NAME.equals(motorName)
                || Constants.BACK_LEFT_MOTOR_NAME.equals(motorName)) return 1;
        if (Constants.FRONT_RIGHT_MOTOR_NAME.equals(motorName)
                || Constants.BACK_RIGHT_MOTOR_NAME.equals(motorName)) return -1;
        return 0;
    }

    /** Sentido configurado do motor selecionado no OpMode de teste. */
    public static DcMotorSimple.Direction driveDirection(String motorName) {
        if (Constants.FRONT_LEFT_MOTOR_NAME.equals(motorName)) return Constants.FRONT_LEFT_DIRECTION;
        if (Constants.BACK_LEFT_MOTOR_NAME.equals(motorName)) return Constants.BACK_LEFT_DIRECTION;
        if (Constants.FRONT_RIGHT_MOTOR_NAME.equals(motorName)) return Constants.FRONT_RIGHT_DIRECTION;
        if (Constants.BACK_RIGHT_MOTOR_NAME.equals(motorName)) return Constants.BACK_RIGHT_DIRECTION;
        return DcMotorSimple.Direction.FORWARD;
    }

    // ============================================================================
    // INTAKE
    // ============================================================================

    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final double INTAKE_POWER = 1.0;
    public static final boolean INTAKE_MOTOR_REVERSED = true;
    public static final Motor.ZeroPowerBehavior INTAKE_ZERO_POWER = Motor.ZeroPowerBehavior.BRAKE;

    // ============================================================================
    // Transfer
    // ============================================================================
    public static final String TRANSFER_MOTOR_NAME = "transfer";
    public static final double TRANSFER_POWER = 1.0;
    public static final boolean TRANSFER_MOTOR_REVERSED = true;
    public static final Motor.ZeroPowerBehavior TRANSFER_ZERO_POWER = Motor.ZeroPowerBehavior.BRAKE;
    // ============================================================================
    // PÁ COLETORA DE FLORES
    // ============================================================================

    /** Nomes dos servos esquerdo e direito da pá na configuração do Control Hub. */
    public static final String FLOWER_COLLECTOR_LEFT_SERVO_NAME = "flowerCollectorLeft";
    public static final String FLOWER_COLLECTOR_RIGHT_SERVO_NAME = "flowerCollectorRight";
    /** Posições da pá esquerda; confirme o curso físico antes do uso. */
    public static final double FLOWER_COLLECTOR_LEFT_STOWED_POSITION = 0.20;
    public static final double FLOWER_COLLECTOR_LEFT_DEPLOYED_POSITION = 0.80;
    /** Posições da pá direita; invertidas porque o servo está montado em espelho. */
    public static final double FLOWER_COLLECTOR_RIGHT_STOWED_POSITION = 0.80;
    public static final double FLOWER_COLLECTOR_RIGHT_DEPLOYED_POSITION = 0.20;
    /** Tempo entre iniciar o intake e implantar a pá na coleta de flores. */
    public static final long FLOWER_COLLECTOR_DEPLOY_DELAY_MILLISECONDS = 200;

    // ============================================================================
    // TURRET HORIZONTAL
    // ============================================================================

    public static final String TURRET_MOTOR_NAME = "turret";
    /** Invertido para o ângulo positivo do encoder corresponder à rotação física para a esquerda. */
    public static final boolean TURRET_MOTOR_REVERSED = true;
    /** Escala calibrada do encoder integrado: ticks do HD Hex para uma volta completa da torre. */
    public static final double TURRET_MOTOR_TICKS_PER_OUTPUT_REV = 497.896;
    /** A leitura externa fica no encoder da porta intake, que não é usado pelo mecanismo intake. */
    public static final String TURRET_THROUGH_BORE_ENCODER_NAME = INTAKE_MOTOR_NAME;
    /** REV HD Hex fornece 28 contagens por volta do eixo do motor, antes das reduções. */
    public static final double TURRET_MOTOR_TICKS_PER_MOTOR_REV = 28.0;
    public static final double TURRET_MOTOR_GEAR_REDUCTION = 3.0;
    public static final double TURRET_THROUGH_BORE_COUNTS_PER_SENSOR_REV = 8192.0;
    public static final int TURRET_MOTOR_PINION_TEETH = 25;
    public static final int TURRET_RING_GEAR_TEETH = 85;
    public static final int TURRET_THROUGH_BORE_GEAR_TEETH = 35;
    /** Escala nominal externa, calculada pela relação 85:35; ainda precisa de calibração física. */
    public static final double TURRET_THROUGH_BORE_TICKS_PER_OUTPUT_REV =
            TURRET_THROUGH_BORE_COUNTS_PER_SENSOR_REV
                    * TURRET_RING_GEAR_TEETH / (double) TURRET_THROUGH_BORE_GEAR_TEETH;
    /** Corrige o sinal invertido pelo sentido configurado no dispositivo intake. */
    public static final double TURRET_THROUGH_BORE_ENCODER_SIGN = -1.0;
    /** Limite de software com margem em relação ao curso mecânico estimado de 320 graus. */
    public static final double TURRET_MAX_ANGLE_DEGREES = 160;
    public static final double TURRET_MANUAL_POWER = 0.40;
    public static final Motor.ZeroPowerBehavior TURRET_ZERO_POWER = Motor.ZeroPowerBehavior.BRAKE;

    // Homing sem sensor: use baixa potência e confirme os limites no robô, sem obstruções.
    public static final double TURRET_HOME_POWER = 0.3;
    public static final double TURRET_HOME_CENTER_POWER = 0.25;
    public static final long TURRET_HOME_STALL_MILLISECONDS = 1000;
    public static final long TURRET_HOME_NO_MOTION_TIMEOUT_MILLISECONDS = 1500;
    public static final long TURRET_HOME_LIMIT_TIMEOUT_MILLISECONDS = 10000;
    public static final double TURRET_HOME_MIN_TRAVEL_DEGREES = 50;
    public static final double TURRET_HOME_MAX_TRAVEL_DEGREES = 350.0;
    public static final int TURRET_HOME_ENCODER_MOVEMENT_TICKS = 2;
    public static final int TURRET_HOME_CENTER_TOLERANCE_TICKS = 5;

    // ============================================================================
    // SHOOTER
    // ============================================================================

    public static final String SHOOTER_MOTOR_NAME = "shooter";
    public static final double SHOOTER_POWER = 1.0;
    public static final boolean SHOOTER_MOTOR_REVERSED = true;
    public static final Motor.ZeroPowerBehavior SHOOTER_ZERO_POWER = Motor.ZeroPowerBehavior.FLOAT;

    // ============================================================================
    // TRAVA DE BOLAS NA BOCA DO SHOOTER
    // ============================================================================

    /** Nome do servo da trava na configuração do Control Hub. */
    public static final String BALL_GATE_SERVO_NAME = "ballGate";
    /** Posição que bloqueia fisicamente a passagem das bolas para o shooter. */
    public static final double BALL_GATE_LOCKED_POSITION = 0.55;
    /** Posição que libera as bolas para serem lançadas. */
    public static final double BALL_GATE_OPEN_POSITION = 0.8;
    /** Intervalo para a pá recolher e o fluxo mecânico estabilizar antes de abrir a trava. */
    public static final long SHOT_PREPARE_DELAY_MILLISECONDS = 300;
    /** Limite de espera da mira antes de cancelar a abertura da trava por segurança. */
    public static final long TURRET_AIM_WAIT_TIMEOUT_MILLISECONDS = 2000;
    /** Tempo de alimentação por sequência; ajuste até todas as bolas passarem sem prolongar o ciclo. */
    public static final long SHOT_FEED_DURATION_MILLISECONDS = 3000;
    /** Tempo mínimo em potência aberta antes de sinalizar shooter preparado. */
    public static final long SHOOTER_SPINUP_ESTIMATE_MILLISECONDS = 1000;

    // ============================================================================
    // SENSOR ÚNICO DO INTAKE E FEEDBACK VISUAL
    // ============================================================================

    /** Único sensor REV 2m: montado na boca do intake para detectar carregador cheio. */
    public static final String MAGAZINE_SENSOR_NAME = "magazineInlet";
    /** O carregador considera cheio ao contar quatro bolas passando pelo sensor da rampa. */
    public static final int MAGAZINE_CAPACITY = 4;
    /** Distância abaixo da qual a bola obstrui a boca do intake; calibre no robô montado. */
    public static final double MAGAZINE_OCCUPIED_DISTANCE_MM = 35.0;
    /** Distância acima da qual a boca é considerada livre; histerese contra ruído. */
    public static final double MAGAZINE_CLEAR_DISTANCE_MM = 85.0;
    /** Exige presença contínua por este tempo antes de parar o intake. */
    public static final long MAGAZINE_SENSOR_DEBOUNCE_MILLISECONDS = 100;
    /** Frequência de leitura durante coleta: no máximo 20 leituras por segundo. */
    public static final long MAGAZINE_SENSOR_SAMPLE_INTERVAL_MILLISECONDS = 50;
    /** Nome do REV Blinkin LED Driver na configuração do Control Hub. */
    public static final String BLINKIN_LED_NAME = "blinkin";

    // ============================================================================
    // OPMode DE SETUP — DISPOSITIVOS TESTÁVEIS
    // ============================================================================

    public static final String[] SETUP_MOTOR_NAMES = {
            Constants.FRONT_LEFT_MOTOR_NAME, Constants.BACK_LEFT_MOTOR_NAME,
            Constants.FRONT_RIGHT_MOTOR_NAME, Constants.BACK_RIGHT_MOTOR_NAME,
            INTAKE_MOTOR_NAME, TRANSFER_MOTOR_NAME, TURRET_MOTOR_NAME, SHOOTER_MOTOR_NAME
    };
    public static final String[] SETUP_SERVO_NAMES = {
            FLOWER_COLLECTOR_LEFT_SERVO_NAME, FLOWER_COLLECTOR_RIGHT_SERVO_NAME,
            BALL_GATE_SERVO_NAME
    };
    public static final double[] SETUP_SERVO_MIN_POSITIONS = {
            FLOWER_COLLECTOR_LEFT_STOWED_POSITION, FLOWER_COLLECTOR_RIGHT_DEPLOYED_POSITION,
            BALL_GATE_LOCKED_POSITION
    };
    public static final double[] SETUP_SERVO_MAX_POSITIONS = {
            FLOWER_COLLECTOR_LEFT_DEPLOYED_POSITION, FLOWER_COLLECTOR_RIGHT_STOWED_POSITION,
            BALL_GATE_OPEN_POSITION
    };
    public static final double[] SETUP_SERVO_INITIAL_POSITIONS = {
            FLOWER_COLLECTOR_LEFT_STOWED_POSITION, FLOWER_COLLECTOR_RIGHT_STOWED_POSITION,
            BALL_GATE_LOCKED_POSITION
    };
    public static final double SETUP_SERVO_STEP = 0.05;
    public static final String[] SETUP_DISTANCE_SENSOR_NAMES = {
            MAGAZINE_SENSOR_NAME
    };

    /** Resolve o sentido elétrico do motor selecionado no teste individual de bancada. */
    public static DcMotorSimple.Direction setupMotorDirection(String motorName) {
        if (driveMotorNamesContain(motorName)) return driveDirection(motorName);
        if (TURRET_MOTOR_NAME.equals(motorName) && TURRET_MOTOR_REVERSED) {
            return DcMotorSimple.Direction.REVERSE;
        }
        if (INTAKE_MOTOR_NAME.equals(motorName) && INTAKE_MOTOR_REVERSED) {
            return DcMotorSimple.Direction.REVERSE;
        }
        if (TRANSFER_MOTOR_NAME.equals(motorName) && TRANSFER_MOTOR_REVERSED) {
            return DcMotorSimple.Direction.REVERSE;
        }
        if (SHOOTER_MOTOR_NAME.equals(motorName) && SHOOTER_MOTOR_REVERSED) {
            return DcMotorSimple.Direction.REVERSE;
        }
        return DcMotorSimple.Direction.FORWARD;
    }

    /** Confere se o nome informado corresponde a uma das quatro rodas Mecanum. */
    private static boolean driveMotorNamesContain(String motorName) {
        return Constants.FRONT_LEFT_MOTOR_NAME.equals(motorName)
                || Constants.BACK_LEFT_MOTOR_NAME.equals(motorName)
                || Constants.FRONT_RIGHT_MOTOR_NAME.equals(motorName)
                || Constants.BACK_RIGHT_MOTOR_NAME.equals(motorName);
    }

    // ============================================================================
    // LOG DE POSES SALVAS PARA AUTO E TELEOP
    // ============================================================================

    private static final PoseFactory AUTO_POSE_DEGREES = PoseFactory.degrees();
    // POSES TELEOP
    public static final Pose RAW_TELEOP_START_POSE = AUTO_POSE_DEGREES.of(82, 8, 90);
    public static final Pose BLUE_HIVE_START_POSE = AUTO_POSE_DEGREES.of(82, 130, -90);
    public static final Pose RED_HIVE_START_POSE = AUTO_POSE_DEGREES.of(58, 8, -90);
    public static final Pose BLUE_FLOWER_START_POSE = AUTO_POSE_DEGREES.of(82, 8, 90);
    public static final Pose RED_FLOWER_START_POSE = AUTO_POSE_DEGREES.of(58, 130, 90);

    // POSES AUTO
    public static final Pose AUTO_START_POSE = AUTO_POSE_DEGREES.of(9, 111, -90);
    /** Pose inicial configurada para o Raw TeleOp; ajuste para o local real onde o robô começa. */
    public static final Pose AUTO_SCORE_POSE = AUTO_POSE_DEGREES.of(16, 128, -45);
    public static final Pose AUTO_PICKUP_1_POSE = AUTO_POSE_DEGREES.of(30, 121, 0);
    public static final Pose AUTO_PICKUP_2_POSE = AUTO_POSE_DEGREES.of(30, 131, 0);
    public static final Pose AUTO_PICKUP_3_POSE = AUTO_POSE_DEGREES.of(45, 128, 90);
    public static final Pose AUTO_PARK_CONTROL_POSE = AUTO_POSE_DEGREES.of(68, 110, 0);
    public static final Pose AUTO_PARK_POSE = AUTO_POSE_DEGREES.of(68, 96, -90);
    /** Posição conhecida para corrigir a localização antes de retomar a mira da torre. */
    public static final Pose TURRET_RESET_POSE = AUTO_POSE_DEGREES.of(135, 81, 90);
    /**
     * Deslocamento do modelo inteiro da rota para a segunda posição inicial (polegadas).
     * É um valor de exemplo: confirme os dois tiles e as colisões no campo antes de competir.
     */
    public static final double AUTO_POSITION_2_OFFSET_X_INCHES = 24.0;
    public static final double AUTO_POSITION_2_OFFSET_Y_INCHES = 0.0;

    public static final long AUTO_WAIT_MILLISECONDS = 1000;
    public static final double AUTO_PICKUP_MAX_PATH_SPEED = 0.5;
    public static final double AUTO_SCORE_MAX_PATH_SPEED = 1.0;
    /** Pequena pausa no ponto de coleta para a pá/intake alcançarem as peças. */
    public static final long AUTO_PICKUP_DWELL_MILLISECONDS = 750;
    /** Se true, a primeira rota implanta a pá; false coleta somente com intake e pá recolhida. */
    public static final boolean AUTO_PICKUP_1_USES_FLOWER_COLLECTOR = false;
    /** Seleção independente do tipo de coleta na segunda rota de pickup. */
    public static final boolean AUTO_PICKUP_2_USES_FLOWER_COLLECTOR = false;
    /** Seleção independente do tipo de coleta na terceira rota de pickup. */
    public static final boolean AUTO_PICKUP_3_USES_FLOWER_COLLECTOR = false;

    /** Escolhe um dos quatro alvos da temporada pela aliança e pelo lado Y do robô. */
    public static FieldTarget targetForRobotY(double robotY) {
        Alliance selectedAlliance = alliance;
        boolean robotCrossedMidline = selectedAlliance == Alliance.BLUE
                ? robotY < FIELD_MID_Y
                : robotY > FIELD_MID_Y;
        return targetForCrossedMidline(robotCrossedMidline);
    }

    /** Escolhe o alvo usando o estado de cruzamento mantido com histerese pelo seletor do OpMode. */
    public static FieldTarget targetForCrossedMidline(boolean robotCrossedMidline) {
        double targetX = alliance == Alliance.BLUE ? BLUE_TARGET_X : RED_TARGET_X;
        double targetY = alliance == Alliance.BLUE ? BLUE_TARGET_Y : RED_TARGET_Y;
        if (robotCrossedMidline) targetY = FIELD_SIZE_INCHES - targetY;
        return new FieldTarget(targetX, targetY);
    }

    /** Resultado imutável da seleção do alvo no campo. */
    public static final class FieldTarget {
        public final double x;
        public final double y;

        /** Guarda as coordenadas selecionadas para a mira sem expor setters. */
        private FieldTarget(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
