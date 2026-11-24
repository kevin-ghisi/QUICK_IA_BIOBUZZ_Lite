package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.hardware.TurretTrackingConfig;

/** Prediz a pose curta do chassi para antecipar o atraso mecânico da torre. */
public final class TurretAimPrediction {
    private static final double MAX_PREDICTION_SECONDS = 0.30;

    private TurretAimPrediction() { }

    public static Pose predictRobotPose(Pose pose, FieldMotionEstimator.Motion motion) {
        double requestedHorizon = TurretTrackingConfig.predictionHorizonSeconds;
        double horizon = Double.isFinite(requestedHorizon)
                ? MathUtils.clamp(requestedHorizon, 0.0, MAX_PREDICTION_SECONDS) : 0.0;
        return new Pose(
                pose.x() + motion.velocityX * horizon,
                pose.y() + motion.velocityY * horizon,
                MathUtils.wrapRadians(pose.heading() + motion.angularVelocity * horizon));
    }
}
