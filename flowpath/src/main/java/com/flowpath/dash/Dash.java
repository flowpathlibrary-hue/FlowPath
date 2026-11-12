package com.flowpath.dash;

import android.content.res.AssetManager;

import com.flowpath.Flow;
import com.flowpath.Pose;
import com.flowpath.Segment;
import com.flowpath.Tune;
import com.flowpath.Vec;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URLDecoder;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * FlowPath's own dashboard: a tiny web server inside the robot app (no extra dependencies).
 * Live field view, error graphs, telemetry, live sliders for every Tune value, and a route planner
 * that exports Java code. Open http://192.168.43.1:8099 from any device on the Control Hub's Wi-Fi.
 * Flow.mecanum(...).build() starts it for you.
 */
public final class Dash {
    public static final int PORT = 8009;

    private static ServerSocket server;
    private static ExecutorService pool;
    private static volatile boolean running;
    private static volatile String packet = "{}";
    private static volatile long seq;
    private static long lastPublishNs;
    private static final long T0 = System.nanoTime();
    private static final Map<String, String> custom = new ConcurrentHashMap<>();
    private static volatile Object routeOwner;
    private static volatile String routeJson = "[]";
    private static final Map<String, Runnable> actions = Collections.synchronizedMap(new LinkedHashMap<String, Runnable>());
    private static final ConcurrentLinkedQueue<Runnable> queue = new ConcurrentLinkedQueue<>();
    private static final ConcurrentLinkedQueue<Pose> poseQueue = new ConcurrentLinkedQueue<>();

    private Dash() {}

    public static boolean isRunning() { return running; }

    /** Start the server (idempotent). Tune is always registered; pass your own classes to tune them too. */
    public static synchronized void start(Class<?>... extraTunables) {
        Params.register(Tune.class);
        Params.register(extraTunables);
        if (running) return;
        try {
            server = new ServerSocket(PORT);
        } catch (Exception e) {
            android.util.Log.e("FlowPath", "Dashboard could not bind port " + PORT, e);
            return;
        }
        pool = Executors.newCachedThreadPool(r -> { Thread t = new Thread(r, "flow-dash"); t.setDaemon(true); return t; });
        running = true;
        Thread acceptor = new Thread(() -> {
            while (running) {
                try {
                    final Socket s = server.accept();
                    pool.execute(() -> handle(s));
                } catch (Exception e) { if (!running) return; }
            }
        }, "flow-dash-accept");
        acceptor.setDaemon(true);
        acceptor.start();
        android.util.Log.i("FlowPath", "Dashboard on port " + PORT);
    }

    public static void routeChanged() { routeOwner = null; }

    /** Tombol di dashboard. Dijalankan di loop robot (lewat flow.update), bukan di thread web. */
    public static void action(String name, Runnable r) { actions.put(name, r); }

    /** Cari & daftarkan semua class ber-@Tunable di package ini, mis. "org.firstinspires.ftc.teamcode". */
    public static void scan(String pkg) { Params.scanPackage(pkg); }

    /** Jalankan aksi dan set-pose dari dashboard. Otomatis dipanggil di publish(). */
    public static void poll(Flow f) {
        Runnable r;
        while ((r = queue.poll()) != null) {
            try { r.run(); } catch (Exception e) { android.util.Log.e("FlowPath", "action failed", e); }
        }
        Pose p;
        while ((p = poseQueue.poll()) != null) f.setPose(p);
    }

    /** Show an extra value in the telemetry table. */
    public static void put(String key, Object value) { custom.put(key, String.valueOf(value)); }

    // ------------------------------------------------------------------ packet building

    public static void publish(Flow f) {
        poll(f);
        if (!running) return;
        long now = System.nanoTime();
        if (now - lastPublishNs < 30_000_000L) return; // ~30 Hz
        lastPublishNs = now;

        Pose p = f.pose(), v = f.velocity();
        Vec c = f.closestPoint();
        StringBuilder sb = new StringBuilder(768);
        sb.append("{\"t\":").append(n((now - T0) / 1e9))
          .append(",\"mode\":\"").append(f.mode()).append('"')
          .append(",\"busy\":").append(f.busy())
          .append(",\"pose\":[").append(n(p.x)).append(',').append(n(p.y)).append(',').append(n(p.heading)).append(']')
          .append(",\"vel\":[").append(n(v.x)).append(',').append(n(v.y)).append(',').append(n(v.heading)).append(']')
          .append(",\"closest\":[").append(n(c.x)).append(',').append(n(c.y)).append(']')
          .append(",\"th\":").append(n(f.targetHeading()))
          .append(",\"tval\":").append(n(f.progress()))
          .append(",\"idx\":").append(f.segmentIndex())
          .append(",\"err\":{\"cross\":").append(n(f.crossError()))
          .append(",\"head\":").append(n(f.headingError()))
          .append(",\"rem\":").append(n(f.remaining())).append('}');
        double[] pw = f.motorPowers();
        sb.append(",\"pw\":[");
        for (int i = 0; i < pw.length; i++) { if (i > 0) sb.append(','); sb.append(n(pw[i])); }
        sb.append(']');

        if (f.route() != null && routeOwner != f.route()) {
            StringBuilder ps = new StringBuilder("[");
            for (int i = 0; i < f.route().size(); i++) {
                Segment s = f.route().get(i);
                if (i > 0) ps.append(',');
                ps.append('[');
                int k = 32;
                for (int j = 0; j <= k; j++) {
                    Vec q = s.curve.point(j / (double) k);
                    if (j > 0) ps.append(',');
                    ps.append('[').append(n(q.x)).append(',').append(n(q.y)).append(']');
                }
                ps.append(']');
            }
            routeJson = ps.append(']').toString();
            routeOwner = f.route();
        }
        sb.append(",\"path\":").append(f.route() == null ? "[]" : routeJson);

        sb.append(",\"tel\":{");
        boolean first = true;
        for (Map.Entry<String, String> e : custom.entrySet()) {
            if (!first) sb.append(',');
            first = false;
            sb.append('"').append(esc(e.getKey())).append("\":\"").append(esc(e.getValue())).append('"');
        }
        sb.append("}}");
        packet = sb.toString();
        seq++;
    }

    private static String n(double d) {
        if (Double.isNaN(d) || Double.isInfinite(d)) return "0";
        return Double.toString(Math.round(d * 1000.0) / 1000.0);
    }

    private static String esc(String s) {
        return s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", " ").replace("\r", " ");
    }

    // ------------------------------------------------------------------ http

    private static void handle(Socket sock) {
        try {
            sock.setTcpNoDelay(true);
            BufferedReader in = new BufferedReader(new InputStreamReader(sock.getInputStream(), "UTF-8"));
            String line = in.readLine();
            if (line == null) { sock.close(); return; }
            String[] parts = line.split(" ");
            String target = parts.length > 1 ? parts[1] : "/";
            String l;
            while ((l = in.readLine()) != null && !l.isEmpty()) { /* skip headers */ }

            String path = target, query = "";
            int q = target.indexOf('?');
            if (q >= 0) { path = target.substring(0, q); query = target.substring(q + 1); }
            OutputStream out = sock.getOutputStream();

            if (path.equals("/stream")) { try { stream(out); } finally { sock.close(); } return; }
            try {
                switch (path) {
                    case "/api/params": send(out, "application/json", Params.toJson().getBytes("UTF-8")); break;
                    case "/api/set": {
                        Map<String, String> qs = parse(query);
                        send(out, "application/json", ("{\"ok\":" + Params.set(qs.get("name"), qs.get("value")) + "}").getBytes());
                        break;
                    }
                    case "/api/actions": {
                        StringBuilder b = new StringBuilder("{\"actions\":[");
                        synchronized (actions) {
                            boolean first = true;
                            for (String k : actions.keySet()) {
                                if (!first) b.append(',');
                                first = false;
                                b.append('"').append(esc(k)).append('"');
                            }
                        }
                        send(out, "application/json", b.append("]}").toString().getBytes("UTF-8"));
                        break;
                    }
                    case "/api/run": {
                        Runnable r = actions.get(parse(query).get("name"));
                        if (r != null) queue.add(r);
                        send(out, "application/json", ("{\"ok\":" + (r != null) + "}").getBytes());
                        break;
                    }
                    case "/api/pose": { // x, y (inci), h (derajat)
                        Map<String, String> qs = parse(query);
                        poseQueue.add(Pose.of(Double.parseDouble(qs.get("x")), Double.parseDouble(qs.get("y")), Double.parseDouble(qs.get("h"))));
                        send(out, "application/json", "{\"ok\":true}".getBytes());
                        break;
                    }
                    case "/api/save": send(out, "application/json", ("{\"ok\":" + Params.save() + "}").getBytes()); break;
                    case "/api/clear": send(out, "application/json", ("{\"ok\":" + Params.clearSaved() + "}").getBytes()); break;
                    default: asset(out, path);
                }
            } finally { sock.close(); }
        } catch (Exception ignored) {
            try { sock.close(); } catch (Exception e) { /* ignore */ }
        }
    }

    private static void stream(OutputStream out) throws Exception {
        out.write(("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache\r\n"
                + "Access-Control-Allow-Origin: *\r\nConnection: keep-alive\r\n\r\n").getBytes());
        out.flush();
        long last = -1;
        int idle = 0;
        while (running) {
            long s = seq;
            if (s != last) {
                out.write(("data: " + packet + "\n\n").getBytes("UTF-8"));
                out.flush();
                last = s; idle = 0;
            } else if (++idle > 25) {
                out.write(": hb\n\n".getBytes());
                out.flush();
                idle = 0;
            }
            Thread.sleep(40);
        }
    }

    private static void asset(OutputStream out, String path) throws Exception {
        String name = path.equals("/") ? "index.html" : path.substring(1);
        if (!name.matches("[A-Za-z0-9._-]+")) { status(out, "404 Not Found"); return; }
        try {
            AssetManager am = AppUtil.getInstance().getApplication().getAssets();
            InputStream is = am.open("flowpath/" + name);
            ByteArrayOutputStream bo = new ByteArrayOutputStream();
            byte[] buf = new byte[4096];
            int r;
            while ((r = is.read(buf)) > 0) bo.write(buf, 0, r);
            is.close();
            String type = name.endsWith(".html") ? "text/html; charset=utf-8" : name.endsWith(".png") ? "image/png"
                    : name.endsWith(".jpg") ? "image/jpeg" : "application/octet-stream";
            send(out, type, bo.toByteArray());
        } catch (Exception e) { status(out, "404 Not Found"); }
    }

    private static void send(OutputStream out, String type, byte[] body) throws Exception {
        out.write(("HTTP/1.1 200 OK\r\nContent-Type: " + type + "\r\nContent-Length: " + body.length
                + "\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n").getBytes());
        out.write(body);
        out.flush();
    }

    private static void status(OutputStream out, String s) throws Exception {
        out.write(("HTTP/1.1 " + s + "\r\nContent-Length: 0\r\nConnection: close\r\n\r\n").getBytes());
        out.flush();
    }

    private static Map<String, String> parse(String query) throws Exception {
        Map<String, String> m = new HashMap<>();
        for (String kv : query.split("&")) {
            int i = kv.indexOf('=');
            if (i > 0) m.put(URLDecoder.decode(kv.substring(0, i), "UTF-8"), URLDecoder.decode(kv.substring(i + 1), "UTF-8"));
        }
        return m;
    }
}
