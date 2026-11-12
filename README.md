# FlowPath

Library pathing + dashboard FTC (mecanum) yang ditulis dari nol dengan API sendiri, siap dipakai bareng **Ivy** untuk command-based.

| Bagian | Isi |
|---|---|
| `Flow` | controller robot: `run(route)`, `hold(pose)`, `turnTo(deg)`, `manual(...)`, `update()` |
| `Route` / `RouteBuilder` | `Route.from(pose).lineTo(..).curveTo(.., controls).faceTangent().maxPower(..).at(t, runnable).build()` |
| Kontrol | translational PID (cross-track + hold x/y), heading PID, speed = feedforward dari kecepatan terukur + velocity PID + profil deselerasi, centripetal, kS |
| `Tune.*` | semua gain, jadi slider live di dashboard |
| `AutoTuner` | mengukur `maxSpeed`, `strafeGain`, `decel` otomatis |
| Dashboard (`Dash`) | web server di dalam robot: Live (field, trail, grafik error, telemetry, slider) + **Planner** (klik field -> kode Java `Route`) |
| `flowpath-ivy` | `FlowCommands.follow / driveTo / hold / turnTo / stop` |

Konvensi: inch, derajat di API (`Pose.of(x, y, deg)`), origin pojok field 0..144, +x kanan, +y atas, heading 0 = menghadap +x, CCW positif.

## Contoh
```java
Flow flow = Flow.mecanum(hardwareMap)
        .motors("fl", "fr", "bl", "br")
        .threeWheel("lEnc", "rEnc", "sEnc", 1982, 13.0, -5.0)   // ticks/inch, track width, strafe offset
        .start(Pose.of(24, 72, 0))
        .build();                                                // sekalian menyalakan dashboard

Route r = Route.from(flow.pose())
        .lineTo(Pose.of(60, 100, 135))
        .curveTo(Pose.of(48, 36, 270), Vec.of(70, 60)).maxPower(0.7).at(0.6, () -> intake.on())
        .build();
flow.run(r);
while (opModeIsActive()) flow.update();
```
Dengan Ivy: `sequential(follow(flow, r), instant(...), ...)` dan loop `Scheduler.execute(); flow.update();` (lihat `ExampleAuto.java`).
Pinpoint / odometri lain: `.localizer(Localizer.custom(update, poseSupplier, velocitySupplier, poseSetter))`.

## Odometri: pilih salah satu
| Sumber | Builder | Catatan |
|---|---|---|
| 3 dead wheel | `.threeWheel("l","r","s", ticksPerInch, trackWidth, strafeOffset)` | paling akurat dengan hardware murah |
| goBILDA Pinpoint | `.pinpoint("pinpoint", xOffsetIn, yOffsetIn)` | x offset = jarak samping pod depan (kiri +); y offset = jarak depan pod strafe (depan +). Robot harus diam saat init. Opsi lain lewat `.localizer(new PinpointLocalizer(hw, "pinpoint", x, y).swingarmPods().reverse(false, true))` |
| Encoder motor roda (DriveLocalizer) | tank: `.driveEncoders(ticksPerInch, trackWidth)`; mecanum: `.driveEncoders(ticksPerInch, trackWidth, wheelBase)` | tanpa sensor tambahan, tapi roda selip jadi kurang akurat (heading paling melenceng). Mecanum: atur `.driveLateralScale(1.1 ~ 1.3)` |
| Lainnya | `.localizer(Localizer.custom(...))` | OTOS, dll |

`DriveLocalizer.ticksPerInch(ticksPerRev, wheelDiameterIn)` membantu menghitung ticks per inch. Arah encoder diambil dari arah motor, jadi roda harus menghitung naik saat membawa robot maju (pakai `invertMotors` / `invertSides` kalau salah).
Tuning DriveLocalizer: dorong robot lurus 48 inch lalu cek x di dashboard (sesuaikan `ticksPerInch`), putar 10 kali di tempat lalu cek heading (sesuaikan `trackWidth`; roda selip membuat nilai efektifnya lebih besar).

## Tank drive
Ganti builder-nya saja, sisanya (Route, Ivy, tuner, dashboard) tetap sama:
```java
Flow flow = Flow.tank(hardwareMap)
        .leftMotors("fl", "bl")           // jumlah motor per sisi bebas
        .rightMotors("fr", "br")
        .threeWheel("lEnc", "rEnc", "sEnc", 1982, 13.0, -5.0)   // atau .localizer(...)
        .start(Pose.of(24, 72, 0))
        .build();
```
Bedanya dengan mecanum (karena tank tidak bisa strafe):
- Heading di tengah rute **tidak bisa bebas**: robot selalu menghadap arah jalur (`faceBackwards()` = jalan mundur). Heading di `Pose.of(x, y, deg)` hanya dipakai di **ujung** rute / `hold()` (robot berputar di tempat setelah sampai).
- Koreksi cross-track dilakukan dengan membelokkan robot (`Tune.Tank.crossGain`), bukan strafe. Kelokan dibantu `Tune.Tank.turnFF`.
- `hold(pose)` / `turnTo()`: robot menghadap titik, jalan (atau mundur kalau lebih dekat), lalu putar ke heading akhir.
- `manual(f, strafe, turn, field)`: strafe & field-centric diabaikan, selalu robot-centric.
- FlowAutoTune otomatis melewati tes strafe dan mengukur kecepatan putar (mengisi `Tune.Tank.turnFF`).
- Slider baru di dashboard: grup **Tank** (crossGain, maxCrossAngleDeg, turnFF, arriveDist, settleDist).
Dites di simulator JVM (stub SDK): line, S-curve, 3 pose, mundur, hold, turnTo, dorongan 4 inch, dan mulai menghadap arah salah, semuanya berhenti dalam sekitar 0,6 inch dari target. **Belum dites di robot** dan belum dicompile terhadap SDK asli; gain default hanyalah titik awal.

## Pasang - pilih salah satu
**A. Sebagai module (tanpa internet / tanpa JitPack, paling stabil).** Taruh folder `FlowPath/` di dalam project FtcRobotController, lalu di `settings.gradle`:
```groovy
include ':flowpath'
project(':flowpath').projectDir = new File(rootDir, 'FlowPath/flowpath')
include ':flowpath-ivy'
project(':flowpath-ivy').projectDir = new File(rootDir, 'FlowPath/flowpath-ivy')
```
dan di `build.dependencies.gradle`:
```groovy
dependencies {
    implementation project(':flowpath')
    implementation project(':flowpath-ivy')
    implementation 'com.pedropathing.ivy:pedro:1.1.1'    // Ivy (baris resmi dari pedropathing.com/docs/ivy)
}
repositories { maven { url = 'https://mymaven.bylazar.com/releases' } }
```
**B. Dependency + repository (JitPack).** Push repo ini ke GitHub, buat tag (mis. `v0.2.0`), salin `gradlew`, `gradlew.bat`, `gradle/wrapper` dari project FTC ke root repo ini (wrapper tidak bisa kubuatkan), lalu:
```groovy
repositories { maven { url 'https://jitpack.io' }; maven { url = 'https://mymaven.bylazar.com/releases' } }
dependencies {
    implementation 'com.github.USERNAME.FlowPath:flowpath:v0.2.0'
    implementation 'com.github.USERNAME.FlowPath:flowpath-ivy:v0.2.0'
    implementation 'com.pedropathing.ivy:pedro:1.1.1'
}
```
Lalu salin `quickstart/TeamCode/.../flowpath/*.java` ke TeamCode kamu dan edit `Robot.java` (nama motor, ticksPerInch, trackWidth, strafeOffset).

## Tuning (3 langkah, semua dari dashboard)
Dashboard: HP/laptop konek ke WiFi Control Hub -> `http://192.168.43.1:8099`.
1. **FlowAutoTune** - robot jalan maju & strafe full power, ukur `Drive.maxSpeed`, `General.strafeGain`, `Drive.decel`, langsung disimpan.
2. **FlowTuner** - A hold, B line, X curve, Y turn; geser slider sambil lihat grafik error:
   - **Translation.kP/kD** (A): dorong robot, harus balik cepat tanpa goyang. kS membantu ujung gerak yang "macet".
   - **Heading.kP/kD** (A/Y): putar robot dengan tangan / tes turn.
   - **Drive.velKP, decel** (B): lewat target -> turunkan `decel`; berhenti kepagian -> naikkan `decel`.
   - **Curves.centripetalGain** (X): naikkan sampai tidak melebar di tikungan.
3. **Save to robot** (tersimpan di `/sdcard/FIRST/flowpath_params.properties`, dimuat otomatis). Untuk ditempel ke kode: **Copy as Java**, lalu **Reset to code**.

Parameter milikmu ikut jadi slider: `.tunables(MyConstants.class)`; nilai tambahan di tabel telemetry: `Dash.put("arm", pos)`.

## Planner
Tab **Planner** di dashboard (juga jalan kalau `index.html` dibuka langsung di browser): klik field untuk menambah waypoint, drag titik / belah ketupat (control point), atur heading mode, lalu **Copy code** -> kode `Route.from(...)...build()` siap tempel.

## Status verifikasi
Dites (JVM, simulator robot + stub SDK): semua kelas library, hardware, Ivy, dan quickstart compile bersih; simulator: lineTo/curveTo/3-segment/callback/holdEnd(false)/turnTo/faceBackwards/hold+recover dari dorongan/manual; AutoTuner terhadap sim (mengukur 50 in/s dengan benar); server dashboard (SSE, 7 grup parameter, set live); JS dashboard + export Planner dijalankan headless.
**Belum dites:** robot nyata, build Gradle/Android sungguhan, tampilan visual di browser, dan signature Ivy (diambil dari docs resmi Pedro, dites hanya terhadap stub). Gain default adalah titik awal. Stub SDK-ku pernah salah dua tempat (`throws`, `sleep`), jadi error compile kecil terhadap SDK asli masih mungkin; kirim pesan error-nya dan perbaikannya kecil.
