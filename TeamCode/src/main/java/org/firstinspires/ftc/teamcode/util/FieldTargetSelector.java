package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.hardware.GlobalConstants;
import org.firstinspires.ftc.teamcode.hardware.TurretTrackingConfig;

/** Mantém a escolha do hive estável ao cruzar a linha central do campo. */
public final class FieldTargetSelector {
    private GlobalConstants.Alliance alliance;
    private boolean crossedMidline;
    private boolean initialized;

    /** Define o lado inicial usando a pose já alinhada do Pinpoint. */
    public void reset(double robotY) {
        alliance = GlobalConstants.alliance;
        crossedMidline = isAcrossMidline(robotY, alliance);
        initialized = true;
    }

    /** Atualiza o estado com histerese e devolve o hive correspondente à aliança e ao lado. */
    public GlobalConstants.FieldTarget update(double robotY) {
        if (!initialized || alliance != GlobalConstants.alliance) reset(robotY);

        double hysteresis = Double.isFinite(TurretTrackingConfig.midlineHysteresisInches)
                ? Math.max(0.0, TurretTrackingConfig.midlineHysteresisInches) : 0.0;
        double switchToOtherSide = GlobalConstants.FIELD_MID_Y
                + (alliance == GlobalConstants.Alliance.BLUE ? -hysteresis : hysteresis);
        double switchBack = GlobalConstants.FIELD_MID_Y
                + (alliance == GlobalConstants.Alliance.BLUE ? hysteresis : -hysteresis);

        if (!crossedMidline && isPastThreshold(robotY, switchToOtherSide, alliance)) {
            crossedMidline = true;
        } else if (crossedMidline && isBackBeforeThreshold(robotY, switchBack, alliance)) {
            crossedMidline = false;
        }
        return GlobalConstants.targetForCrossedMidline(crossedMidline);
    }

    private static boolean isAcrossMidline(double robotY, GlobalConstants.Alliance alliance) {
        return alliance == GlobalConstants.Alliance.BLUE
                ? robotY < GlobalConstants.FIELD_MID_Y
                : robotY > GlobalConstants.FIELD_MID_Y;
    }

    private static boolean isPastThreshold(double robotY, double threshold,
                                          GlobalConstants.Alliance alliance) {
        return alliance == GlobalConstants.Alliance.BLUE
                ? robotY < threshold : robotY > threshold;
    }

    private static boolean isBackBeforeThreshold(double robotY, double threshold,
                                                 GlobalConstants.Alliance alliance) {
        return alliance == GlobalConstants.Alliance.BLUE
                ? robotY > threshold : robotY < threshold;
    }
}
