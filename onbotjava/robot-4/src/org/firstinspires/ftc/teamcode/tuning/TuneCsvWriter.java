package org.firstinspires.ftc.teamcode.tuning;

import java.io.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

/** Bounded queue: storage failures/backpressure disable motion instead of blocking its loop. */
public final class TuneCsvWriter implements AutoCloseable {
    private final ArrayBlockingQueue<String> queue = new ArrayBlockingQueue<>(256);
    private volatile boolean closing, failed;
    public volatile double lastSyncMs;
    public volatile String error = "";
    private final Thread worker;
    public TuneCsvWriter(final File file, final String header) throws IOException {
        final FileOutputStream stream = new FileOutputStream(file);
        final BufferedWriter out = new BufferedWriter(new OutputStreamWriter(stream, "UTF-8"));
        try { out.write(header); out.newLine(); out.flush(); stream.getFD().sync(); }
        catch (IOException e) { try { out.close(); } catch (IOException ignored) { } throw e; }
        worker = new Thread(() -> {
            long lastSync = System.nanoTime();
            try {
                while (!closing || !queue.isEmpty()) {
                    String row = queue.poll(100, TimeUnit.MILLISECONDS);
                    if (row != null) { out.write(row); out.newLine(); }
                    // Flush userspace buffers every row; fsync at most once per 200 ms.
                    out.flush();
                    long now = System.nanoTime();
                    if (now - lastSync >= 200_000_000L || closing) {
                        long start = System.nanoTime(); stream.getFD().sync();
                        lastSyncMs = (System.nanoTime() - start) / 1e6; lastSync = now;
                    }
                }
            } catch (Exception e) { fail(e.toString()); }
            finally {
                try { out.flush(); stream.getFD().sync(); out.close(); }
                catch (IOException e) { fail(e.toString()); }
            }
        }, "R4-extend-log");
        worker.setDaemon(true); worker.start();
    }
    private void fail(String message) { error = message; failed = true; }
    public boolean offer(String row) {
        if (closing || failed) return false;
        if (!queue.offer(row)) { fail("Log queue full"); return false; }
        return true;
    }
    public boolean healthy() { return !failed && !closing && worker.isAlive(); }
    public int pending() { return queue.size(); }
    public void close() {
        closing = true;
        try { worker.join(2000); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        if (worker.isAlive()) fail("Writer did not drain within 2 seconds");
    }
}
