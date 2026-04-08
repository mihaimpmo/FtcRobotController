package org.firstinspires.ftc.teamcode;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class EncoderCsvLogger {

    private final String path;
    private BufferedWriter writer;

    private final StringBuilder buffer = new StringBuilder(65536);
    private static final int FLUSH_THRESHOLD_CHARS = 32768;

    private boolean started = false;

    public EncoderCsvLogger(String path) {
        this.path = path;
    }

    public void start() throws IOException {
        if (started) return;

        writer = new BufferedWriter(new FileWriter(path, false), 65536);
        writer.write(
                "type,timestamp_ns,timestamp_ms,prev_state,curr_state,delta," +
                        "counts,angle_deg,velocity_counts_per_sec,velocity_deg_per_sec," +
                        "a,b,error_count,last_direction\n"
        );
        started = true;
    }

    public void logEvent(
            long timestampNs,
            int prevState,
            int currState,
            int delta,
            int counts,
            double angleDeg,
            double velocityCountsPerSec,
            double velocityDegPerSec,
            boolean a,
            boolean b,
            long errorCount,
            int lastDirection
    ) throws IOException {
        appendRow(
                "EVENT",
                timestampNs,
                prevState,
                currState,
                delta,
                counts,
                angleDeg,
                velocityCountsPerSec,
                velocityDegPerSec,
                a,
                b,
                errorCount,
                lastDirection
        );
    }

    public void logSample(
            long timestampNs,
            int currState,
            int counts,
            double angleDeg,
            double velocityCountsPerSec,
            double velocityDegPerSec,
            boolean a,
            boolean b,
            long errorCount,
            int lastDirection
    ) throws IOException {
        appendRow(
                "SAMPLE",
                timestampNs,
                currState,
                currState,
                0,
                counts,
                angleDeg,
                velocityCountsPerSec,
                velocityDegPerSec,
                a,
                b,
                errorCount,
                lastDirection
        );
    }

    private void appendRow(
            String type,
            long timestampNs,
            int prevState,
            int currState,
            int delta,
            int counts,
            double angleDeg,
            double velocityCountsPerSec,
            double velocityDegPerSec,
            boolean a,
            boolean b,
            long errorCount,
            int lastDirection
    ) throws IOException {
        if (!started) return;

        long timestampMs = timestampNs / 1_000_000L;

        buffer.append(type).append(',')
                .append(timestampNs).append(',')
                .append(timestampMs).append(',')
                .append(prevState).append(',')
                .append(currState).append(',')
                .append(delta).append(',')
                .append(counts).append(',')
                .append(formatDouble(angleDeg)).append(',')
                .append(formatDouble(velocityCountsPerSec)).append(',')
                .append(formatDouble(velocityDegPerSec)).append(',')
                .append(a ? 1 : 0).append(',')
                .append(b ? 1 : 0).append(',')
                .append(errorCount).append(',')
                .append(lastDirection)
                .append('\n');

        if (buffer.length() >= FLUSH_THRESHOLD_CHARS) {
            flush();
        }
    }

    private String formatDouble(double value) {
        return String.format(Locale.US, "%.6f", value);
    }

    public void flush() throws IOException {
        if (!started || buffer.length() == 0) return;
        writer.write(buffer.toString());
        buffer.setLength(0);
        writer.flush();
    }

    public void close() {
        if (!started) return;

        try {
            flush();
            writer.close();
        } catch (IOException ignored) {
        } finally {
            started = false;
        }
    }
}