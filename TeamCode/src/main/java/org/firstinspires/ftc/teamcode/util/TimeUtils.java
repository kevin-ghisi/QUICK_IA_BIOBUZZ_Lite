package org.firstinspires.ftc.teamcode.util;

/**
 * Relógio monotônico simples, apresentado em milissegundos para facilitar leitura e tuning.
 * Ele deriva de nanoTime, portanto mede duração sem ser afetado por ajuste de data/hora.
 */
public final class TimeUtils {
    private TimeUtils() { }

    /** Retorna o instante monotônico atual em milissegundos, não a data/hora do calendário. */
    public static long monotonicMilliseconds() {
        return System.nanoTime() / 1_000_000L;
    }

    /** Retorna milissegundos fracionários para medições de ciclo sem arredondar cada amostra. */
    public static double monotonicMillisecondsPrecise() {
        return System.nanoTime() / 1_000_000.0;
    }
}
