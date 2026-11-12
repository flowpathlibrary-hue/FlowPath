package com.flowpath.dash;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import dalvik.system.DexFile;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Enumeration;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Properties;

/**
 * Registry of live-tunable values: public static non-final double/int/boolean fields of the given
 * classes AND of their nested classes. Persisted to /sdcard/FIRST/flowpath_params.properties.
 * Key format: "Group.field" where Group is the simple name of the declaring class (e.g. "Translation.kP").
 */
public final class Params {
    private static final Map<String, Field> fields = new LinkedHashMap<>();
    private static final Map<String, String> defaults = new LinkedHashMap<>();

    private Params() {}

    private static File file() { return new File(AppUtil.FIRST_FOLDER, "flowpath_params.properties"); }

    public static synchronized void register(Class<?>... classes) {
        Properties saved = readSaved();
        for (Class<?> c : classes) scan(c, saved);
    }

    private static void scan(Class<?> c, Properties saved) {
        for (Field f : c.getDeclaredFields()) {
            int m = f.getModifiers();
            Class<?> ty = f.getType();
            if (!Modifier.isStatic(m) || Modifier.isFinal(m) || f.isSynthetic()) continue; // public / private, semua ikut
            if (ty != double.class && ty != int.class && ty != boolean.class) continue;
            String key = c.getSimpleName() + "." + f.getName();
            if (fields.containsKey(key)) { // nama class sama di file lain -> pakai nama lengkap
                if (fields.get(key).equals(f)) continue;
                key = c.getName().replace('$', '.') + "." + f.getName();
                if (fields.containsKey(key)) continue;
            }
            f.setAccessible(true);
            fields.put(key, f);
            defaults.put(key, get(f));
            String sv = saved.getProperty(key);
            if (sv != null) set(key, sv); // saved tuning overrides code defaults
        }
        for (Class<?> n : c.getDeclaredClasses()) scan(n, saved);
    }

    /** Label grup di dashboard: "NamaFile > NamaClassDalam". */
    private static String label(Field f) {
        String n = f.getDeclaringClass().getName();
        return n.substring(n.lastIndexOf('.') + 1).replace("$", " \u203a ");
    }

    /** Cari semua class ber-@Tunable di package ini (semua file di TeamCode) dan daftarkan. Best effort. */
    public static synchronized void scanPackage(String prefix) {
        Properties saved = readSaved();
        try {
            DexFile df = new DexFile(AppUtil.getInstance().getApplication().getPackageCodePath());
            for (Enumeration<String> en = df.entries(); en.hasMoreElements(); ) {
                String n = en.nextElement();
                if (!n.startsWith(prefix)) continue;
                try {
                    Class<?> c = Class.forName(n, false, Params.class.getClassLoader()); // false = tanpa static init
                    if (c.isAnnotationPresent(Tunable.class)) scan(c, saved);
                } catch (Throwable ignored) {}
            }
        } catch (Throwable ignored) {}
    }

    private static String get(Field f) {
        try { return String.valueOf(f.get(null)); } catch (Exception e) { return "0"; }
    }

    public static synchronized boolean set(String key, String value) {
        Field f = fields.get(key);
        if (f == null || value == null) return false;
        try {
            Class<?> ty = f.getType();
            if (ty == double.class) f.setDouble(null, Double.parseDouble(value));
            else if (ty == int.class) f.setInt(null, (int) Math.round(Double.parseDouble(value)));
            else f.setBoolean(null, Boolean.parseBoolean(value));
            return true;
        } catch (Exception e) { return false; }
    }

    public static synchronized String toJson() {
        StringBuilder sb = new StringBuilder("{\"saved\":").append(file().exists()).append(",\"params\":[");
        boolean first = true;
        for (Map.Entry<String, Field> e : fields.entrySet()) {
            if (!first) sb.append(',');
            first = false;
            Class<?> t = e.getValue().getType();
            String ty = t == boolean.class ? "bool" : t == int.class ? "int" : "double";
            sb.append("{\"c\":\"").append(label(e.getValue())).append("\",\"k\":\"").append(e.getKey()).append("\",\"v\":").append(get(e.getValue()))
              .append(",\"d\":").append(defaults.get(e.getKey())).append(",\"t\":\"").append(ty).append("\"}");
        }
        return sb.append("]}").toString();
    }

    public static synchronized boolean save() {
        Properties p = new Properties();
        for (Map.Entry<String, Field> e : fields.entrySet()) p.setProperty(e.getKey(), get(e.getValue()));
        try (FileOutputStream out = new FileOutputStream(file())) {
            p.store(out, "FlowPath tuned values");
            return true;
        } catch (Exception e) { return false; }
    }

    /** Delete saved overrides and restore the defaults written in code. */
    public static synchronized boolean clearSaved() {
        for (Map.Entry<String, String> e : defaults.entrySet()) set(e.getKey(), e.getValue());
        return !file().exists() || file().delete();
    }

    private static Properties readSaved() {
        Properties p = new Properties();
        File f = file();
        if (f.exists()) try (FileInputStream in = new FileInputStream(f)) { p.load(in); } catch (Exception ignored) {}
        return p;
    }
}
