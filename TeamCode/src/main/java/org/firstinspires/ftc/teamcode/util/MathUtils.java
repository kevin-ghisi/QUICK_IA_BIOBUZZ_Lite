package org.firstinspires.ftc.teamcode.util;

/** Operações matemáticas reutilizáveis para evitar duplicar lógica entre subsistemas. */
public final class MathUtils {
    private MathUtils() { }

    /**
     * Converte um número de uma faixa para outra por proporção linear.
     * Se a faixa de entrada não tiver largura, devolve o início da faixa de saída.
     */
    public static double mapRange(double value, double inputMin, double inputMax,
                                  double outputMin, double outputMax) {
        if (inputMax == inputMin) return outputMin;
        double progress = (value - inputMin) / (inputMax - inputMin);
        return outputMin + progress * (outputMax - outputMin);
    }

    /** Garante que um valor fique entre os limites mínimo e máximo informados. */
    public static double clamp(double value, double min, double max) {
        double lowerLimit = Math.min(min, max);
        double upperLimit = Math.max(min, max);
        return Math.max(lowerLimit, Math.min(upperLimit, value));
    }

    /**
     * Reduz um ângulo em radianos ao intervalo [-π, π].
     * O resultado é a menor rotação assinada equivalente ao ângulo original.
     */
    public static double wrapRadians(double angle) {
        double wrapped = angle;
        while (wrapped > Math.PI) wrapped -= 2.0 * Math.PI;
        while (wrapped < -Math.PI) wrapped += 2.0 * Math.PI;
        return wrapped;
    }
}
