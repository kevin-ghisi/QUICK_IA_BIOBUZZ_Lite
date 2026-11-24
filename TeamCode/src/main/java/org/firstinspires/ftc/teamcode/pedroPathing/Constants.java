package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Configurações exigidas pelo Pedro Pathing: drivetrain, localizer e algoritmo Foresight.
 * A documentação oficial do Pedro reúne esses três blocos neste arquivo porque o Follower os recebe juntos.
 */
public class Constants {
    // Nomes e sentidos das rodas pertencem à configuração de drivetrain do Pedro.
    public static final String FRONT_LEFT_MOTOR_NAME = "lf";
    public static final String BACK_LEFT_MOTOR_NAME = "lb";
    public static final String FRONT_RIGHT_MOTOR_NAME = "rf";
    public static final String BACK_RIGHT_MOTOR_NAME = "rb";
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // --------------------------------------------------------------------------
    // MECANUM: nomes e sentidos dos motores exigidos pelo Follower do Pedro.
    // --------------------------------------------------------------------------
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lb");
        c.backRightName.set("rb");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // --------------------------------------------------------------------------
    // PINPOINT: nome, unidades, offsets e sentidos dos encoders de odometria.
    // Calibre offsets e sentidos no robô antes de confiar em coordenadas de campo.
    // --------------------------------------------------------------------------
    public static final PinpointConfig localizerConfig = new PinpointConfig(config -> {
        config.name.set("pinpoint");
        config.xPodOffset.set(5.98425);
        config.yPodOffset.set(1.1811);
        config.offsetUnits.set(DistanceUnit.INCH);
        config.globalDistanceUnit.set(DistanceUnit.INCH);
        config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // confirmar com kevin - le
        config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    });

    // --------------------------------------------------------------------------
    // FORESIGHT: parâmetros do seguidor de caminhos do Pedro, separados da odometria.
    // Substitua estes valores pelos resultados do AutoTune para este drivetrain.
    // --------------------------------------------------------------------------
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4292770068864633);
                Controller secondaryTranslationalForward = Controller.proportional(0.15860638355166912);
                Controller primaryTranslationalLateral = Controller.proportional(0.5548454819138047);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2050005798228245);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011495216658257817));
                c.brake.set(Controller.proportionalFeedforward(0.009770934159519145));

                c.headingFeedback.set(Controller.proportional(6.945536144498462));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.12639402449444578, 0.002227905493001494));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09364576699242606, 0.048314621355114856));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0022495499185542717, 0.0015706552731321176));

                c.maxAchievableForwardVelocity.set(79.10657000566728);
                c.maxAchievableStrafeVelocity.set(60.12958399111294);
                c.naturalForwardDeceleration.set(36.92241950771221);
                c.naturalStrafeDeceleration.set(71.78920890361171);
            }
    );

    /**
     * Monta um único Follower combinando odometria Pinpoint, chassi Mecanum e Foresight.
     * As três configurações são passadas ao construtor porque o Pedro precisa delas para localizar,
     * aplicar potência às rodas e calcular os caminhos.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
