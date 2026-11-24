package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.math.Pose;

/** Transferência de uso único da pose entre autônomo e teleoperado no mesmo processo do Control Hub. */
public final class AutoPoseHandoff {
    public static volatile Pose pose;

    private AutoPoseHandoff() { }

    /** Guarda uma cópia da pose final para que alterações posteriores não a modifiquem. */
    public static void store(Pose finalPose) {
        pose = finalPose == null ? null : new Pose(finalPose.x(), finalPose.y(), finalPose.heading());
    }

    /** Devolve a pose guardada e a limpa para evitar reutilização acidental. */
    public static Pose consume() {
        Pose result = pose;
        pose = null;
        return result;
    }
}
