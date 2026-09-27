package org.firstinspires.ftc.teamcode.timing;

import java.io.File;
import java.io.IOException;
import org.firstinspires.ftc.teamcode.tuning.TuneCsvWriter;

/** Logs existing electrical API reads individually; never performs extra sensor reads. */
final class PowerTrace implements AutoCloseable {
    private final long originNs = System.nanoTime();
    private long sequence;
    private final TuneCsvWriter writer;
    final String path;
    PowerTrace(File directory) throws IOException {
        if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("No log directory");
        File file = new File(directory, "robot4_power_" + System.currentTimeMillis() + "_" + Long.toHexString(originNs) + ".csv");
        path = file.getAbsolutePath();
        writer = new TuneCsvWriter(file, "t_ms,seq,signal,value,ok,read_ms,ext_angle_deg,ext_target_deg,ext_out,ext_stuck,ext_reverse,ext_fault,period_ms,build,queue_rows,last_sync_ms");
    }
    void record(String signal, double value, long startNs, boolean ok, ExtendController ext, double period) {
        long now = System.nanoTime();
        writer.offer((now - originNs) / 1e6 + "," + (++sequence) + "," + signal + "," + value + "," + (ok ? 1 : 0)
            + "," + (now - startNs) / 1e6 + "," + ext.current + "," + ext.targetAngle + "," + ext.output
            + "," + (ext.isStuck ? 1 : 0) + "," + (ext.reverse ? 1 : 0) + "," + (ext.poteFault || ext.stallFault ? 1 : 0)
            + "," + period + "," + BuildInfo.STAMP + "," + writer.pending() + "," + writer.lastSyncMs);
    }
    boolean healthy() { return writer.healthy(); }
    public void close() { writer.close(); }
}
