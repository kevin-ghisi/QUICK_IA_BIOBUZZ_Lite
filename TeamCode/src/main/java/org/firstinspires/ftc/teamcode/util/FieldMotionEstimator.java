package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;

/** Estima velocidade translacional e angular em coordenadas do campo a partir das poses Pinpoint. */
public final class FieldMotionEstimator {
    private Pose previousPose;
    private long previousTimeNanos;
    private double velocityX;
    private double velocityY;
    private double angularVelocity;

    /** Uma amostra diferencial; velocidades estão em polegadas/s e radianos/s. */
    public static final class Motion {
        public final double velocityX;
        public final double velocityY;
        public final double angularVelocity;

        public Motion(double velocityX, double velocityY, double angularVelocity) {
            this.velocityX = velocityX;
            this.velocityY = velocityY;
            this.angularVelocity = angularVelocity;
        }
    }

    /** Calcula diferenças entre poses e filtra ruído sem introduzir biblioteca externa. */
    public Motion update(Pose currentPose, long nowNanos) {
        if (previousPose != null) {
            double elapsedSeconds = (nowNanos - previousTimeNanos) / 1_000_000_000.0;
            if (elapsedSeconds > 0.001 && elapsedSeconds < 0.25) {
                double rawX = (currentPose.x() - previousPose.x()) / elapsedSeconds;
                double rawY = (currentPose.y() - previousPose.y()) / elapsedSeconds;
                double rawOmega = MathUtils.wrapRadians(currentPose.heading() - previousPose.heading())
                        / elapsedSeconds;
                double alpha = GlobalConstants.FIELD_VELOCITY_FILTER_ALPHA;
                velocityX += alpha * (rawX - velocityX);
                velocityY += alpha * (rawY - velocityY);
                angularVelocity += alpha * (rawOmega - angularVelocity);
            }
        }
        previousPose = new Pose(currentPose.x(), currentPose.y(), currentPose.heading());
        previousTimeNanos = nowNanos;
        return new Motion(velocityX, velocityY, angularVelocity);
    }

    /** Zera o histórico para impedir um salto de velocidade entre períodos/poses distintas. */
    public void reset() {
        previousPose = null;
        previousTimeNanos = 0L;
        velocityX = 0.0;
        velocityY = 0.0;
        angularVelocity = 0.0;
    }
}
