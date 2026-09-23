// ============================================================================
// 最終変更: 2026-09-23 14:48:47 JST (UTC+09:00)
// 変更内容: センサーAPI所要時間と制御周期の計測を追加。取得頻度・制御設定は維持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="MainClass_Blue", group = "0_MATCH")
public class MainClass_Blue extends LinearOpMode {

    private RobotHardware hardware;
    private DriveController driveController;
    private ShooterController shooterController;
    private ClimbController climbController;
    private IntakeController intakeController;
    private RobotLogger logger;
    private ExtendController extendController;
    private FeedController feedController;
    private ImuController imuController;
    private DriveAssist driveAssist;
    private Discharge discharge;
    private Distance distance;

    boolean prevA = false;
    boolean prevGamepad2DpadUp = false;
    boolean prevGamepad2DpadDown = false;
    boolean prevX = false;
    boolean prevY = false;

    boolean prevRB = false;
    double prevLeftT;
    boolean isStart = false;
    // 通常・射撃・登坂方位補助・排出は排他。登坂機構の段階はclimbEnumで別管理する。
    private enum RobotMode { MANUAL, SHOOT, CLIMB, DISCHARGE }
    private RobotMode mode = RobotMode.MANUAL;
    boolean wasShoot = false;
    int climbEnum = 0;
    private final ClimbSequence climbSequence = new ClimbSequence();
    int climbTime = 120;
    boolean timerRunning = false;
    boolean isReverse = false;
    double extendImpact = 0;

    // 通常は左スティックで前後、右スティックで旋回。故障時の切替設定は左右をセットで変更する。
    private static final boolean USE_RIGHT_STICK_TURN = true;

    private static final double RIGHT_STICK_DEADZONE = 0.05;

    private static final boolean USE_LEFT_STICK_TURN = false;

    // 射撃位置合わせ中、この入力を超えたら距離制御より手動前後を優先する。
    private static final double SHOOT_MANUAL_FWD_DEADZONE = 0.15;

    // シューター目標はエンコーダのticks/s。standardSpeedは停止ではなく低速逆転。
    double shootSpeed = 1200;
    double standardSpeed = -100;
    // 各目標方位は開始時のIMUゼロ基準、単位はdeg。Blue/Redは符号が反転する。
    double climbTarget = 16;
    double shootTarget = -90;
    double shootTargetB = -30;
    double dischargeTarget = 120;

    // 登坂の表裏切替には70/110degのヒステリシスを設ける。
    boolean climbFrontSide = true;
    static final double CLIMB_FLIP_OUT  = 110.0;
    static final double CLIMB_FLIP_BACK = 70.0;

    @Override
    public void runOpMode() {

        // 初期化：ハードウェア、各制御、測距スレッド、ログの順に準備する。
        hardware = new RobotHardware();
        hardware.init(hardwareMap);
        driveController = new DriveController(hardware);
        shooterController = new ShooterController(hardware);

        shooterController.setFeedGateForTarget(shootSpeed);
        climbController = new ClimbController(hardware);
        intakeController = new IntakeController(hardware);
        feedController = new FeedController(hardware);
        extendController = new ExtendController(hardware);
        imuController = new ImuController(hardware);
        driveAssist = new DriveAssist();
        distance = new Distance(hardware.distanceSensor, imuController, hardware.reads);

        logger = new RobotLogger(hardware, hardwareMap, imuController, extendController,
                shooterController, distance);

        distance.startReader();

        logger.setIntake(intakeController);
        logger.setDriveAssist(driveAssist);
        logger.init();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime climbTimer = new ElapsedTime();
        ElapsedTime allTimer = new ElapsedTime();

        // MANUALキャッシュは毎周期clearBulkCache()が必須。IMUと測距は別途取得する。
        List<LynxModule> hubList = hardwareMap.getAll(LynxModule.class);
        LynxModule[] hubs = hubList.toArray(new LynxModule[0]);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // シューターFF用の電圧は100msキャッシュ。初回読み取りを妨げる時刻初期値を使わない。
        final LynxModule[] hubsForVolt = hubs;
        shooterController.setVoltageReader(new ShooterController.VoltageReader() {
            private static final long VOLT_CACHE_MS = 100;

            private long lastMs = 0;
            private double cached = 0.0;
            @Override public double getVoltage() {
                long now = System.currentTimeMillis();
                if (now - lastMs < VOLT_CACHE_MS) return cached;
                double v = Double.POSITIVE_INFINITY;
                for (LynxModule h : hubsForVolt) {
                    double x = hardware.reads.voltage(h, SensorTiming.FF_VOLTAGE);
                    if (x > 0) v = Math.min(v, x);
                }
                cached = (v == Double.POSITIVE_INFINITY) ? 0.0 : v;
                lastMs = now;
                return cached;
            }
        });

        // 画面の組み立ては100ms間隔。制御自体は毎周期実行する。
        long lastTelemMs = 0;
        final long TELEM_INTERVAL_MS = 100;

        // 0は手動入力の加速制限なし。閉ループ制御にはこの制限を掛けない。
        SlewLimiter fwdSlew  = new SlewLimiter(0.0);
        SlewLimiter turnSlew = new SlewLimiter(0.0);
        ElapsedTime driveDtTimer = new ElapsedTime();

        // 小さな指令差の通信を省略し、250msごとに再送する。
        CachedPower driveLeftOut  = new CachedPower(hardware.leftMotor,  0.01);
        CachedPower driveRightOut = new CachedPower(hardware.rightMotor, 0.01);

        driveController.setOutputCache(driveLeftOut, driveRightOut);

        telemetry.addLine("################################################");
        telemetry.addData("ビルド", BuildInfo.STAMP);
        telemetry.addLine("  push_build が表示した値と違うなら Restart Robot をやり直す");
        telemetry.addLine("################################################");
        telemetry.addData("Status", "準備完了");
        telemetry.update();

        // START後にIMUをゼロ化。本動作はさらに前後スティック入力で開始する。
        waitForStart();

        long phT0, phT1, phT2, phT3, phT4, phT5;

        long phM1, phM2, phM3;

        long phPrevT0 = 0;
        double periodMs = 0.0;

        double lastHoldMs = 0.0;

        // 目標周期25ms。超過は計数し、制御器のdtには実測値を使う。
        final long LOOP_PERIOD_MS = 25;
        final long LOOP_PERIOD_NS = LOOP_PERIOD_MS * 1_000_000L;
        long loopDeadline   = System.nanoTime();

        // 左右が同じ軸を回すため、片側故障は速度差ではなく電流の偏りで警告する。
        boolean shooterOneSideFault = false;
        String  shooterFaultText    = "";
        double  shooterFaultAtSec   = 0.0;
        int     shooterFaultCount   = 0;

        int  loopCountTotal = 0;
        int  overrunCount   = 0;
        double overrunWorstMs = 0.0;

        boolean lastLeftBumper = false;

        imuController.reset();


        while (opModeIsActive()) {
            phT0 = System.nanoTime();
            periodMs = (phPrevT0 == 0) ? 0.0 : (phT0 - phPrevT0) / 1e6;
            if (phPrevT0 != 0) hardware.timing.record(SensorTiming.LOOP_PERIOD, phPrevT0, phT0, false);
            phPrevT0 = phT0;

            // 周期先頭：バルクキャッシュ更新、IMUキャッシュ無効化、電流等を順番に取得する。
            long cacheStart = System.nanoTime();
            for (int h = 0; h < hubs.length; h++) hubs[h].clearBulkCache();
            hardware.timing.recordSince(SensorTiming.CACHE_CLEAR, cacheStart, false);

            imuController.beginLoop();

            // 測距は前周期の射撃状態に応じて有効化する。別スレッドでもHub通信は共有する。
            distance.setReaderActive((mode == RobotMode.SHOOT));

            logger.readOneCurrent();
            phT1 = System.nanoTime();

            // 入力の取得と通常走行出力の計算。最終出力は後段で制御の優先順位を決めて書く。
            double forward = -gamepad1.left_stick_y;
            double turn;

            if (USE_RIGHT_STICK_TURN && Math.abs(gamepad1.right_stick_x) > RIGHT_STICK_DEADZONE) {
                turn = gamepad1.right_stick_x;
            } else if (USE_LEFT_STICK_TURN) {
                turn = gamepad1.left_stick_x;
            } else {
                turn = 0.0;
            }

            double Angle = imuController.getHeading();
            phT2 = System.nanoTime();

            double driveDt = driveDtTimer.seconds();
            driveDtTimer.reset();
            double forwardSlewed = fwdSlew.calculate(forward, driveDt);
            double turnSlewed    = turnSlew.calculate(turn,    driveDt);
            DriveController.DriveOutput manualOutput =
                    driveController.driveManual(forwardSlewed, turnSlewed);

            boolean imuActive = false;
            double imuTurnOutput = 0.0;

            // 右バンパーの押し直しで展開の停止保護を解除し、再試行を許す。
            if (gamepad1.right_bumper && !prevRB) {
                extendController.clearStallFault();
            }


            // 最近展開が詰まり、射撃速度が不足している場合は展開を休ませる。
            extendController.yieldCurrent =
                    (mode == RobotMode.SHOOT)
                    && extendController.isJamRecent()
                    && !shooterController.isFeedReady();

            // 現在の順序ではシューター・展開の制御がボタン処理より先。新しい指令は次周期に反映する。
            shooterController.shooter_loop();
            phT3 = System.nanoTime();

            if (gamepad2.dpad_up && !prevGamepad2DpadUp) {
                shootSpeed = Math.min(2000, shootSpeed + 100);
                applyShootSpeed();
            }

            if (gamepad2.dpad_down && !prevGamepad2DpadDown) {
                shootSpeed = Math.max(0, shootSpeed - 100);
                applyShootSpeed();
            }

            if (isStart) {
                extendController.extend_loop();
                if (gamepad1.left_bumper) {
                    extendController.extendDown();
                }

                if (gamepad1.y && !prevY) {
                    extendController.extendUpNow();
                }

                // 射撃開始：RTは通常方位、Aは別方位。登坂方位補助中は受け付けない。
                if (gamepad1.right_trigger > 0.1 && (mode != RobotMode.SHOOT) && (mode != RobotMode.CLIMB)) {
                    mode = RobotMode.SHOOT;
                    imuController.setTarget(shootTarget);
                    shooterController.setSpeed(shootSpeed);
                    extendController.extendMiddle();
                }else if(gamepad1.a && !prevA && (mode != RobotMode.SHOOT) && (mode != RobotMode.CLIMB)) {
                    mode = RobotMode.SHOOT;
                    imuController.setTarget(shootTargetB);
                    shooterController.setSpeed(shootSpeed);
                }

                // Xで排出モードを切替。射撃・登坂中は受け付けない。
                if (gamepad1.x && !prevX && (mode != RobotMode.SHOOT) && (mode != RobotMode.CLIMB)) {
                    mode = (mode == RobotMode.DISCHARGE) ? RobotMode.MANUAL : RobotMode.DISCHARGE;

                    if ((mode == RobotMode.DISCHARGE)) {
                        imuController.setTarget(dischargeTarget);
                    }
                }

                if((mode != RobotMode.SHOOT) && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)){
                    mode = RobotMode.CLIMB;
                }

                // 登坂方位補助：現在方位に応じて表側または裏側の目標を選ぶ。
                if ((mode == RobotMode.CLIMB)) {
                    double climbErr = Math.abs(normalizeAngle(Angle - climbTarget));
                    if (climbFrontSide && climbErr > CLIMB_FLIP_OUT) {
                        climbFrontSide = false;
                    } else if (!climbFrontSide && climbErr < CLIMB_FLIP_BACK) {
                        climbFrontSide = true;
                    }
                    imuController.setTarget(climbFrontSide ? climbTarget
                            : (climbTarget > 0 ? climbTarget - 180 : climbTarget + 180));
                }

                // 射撃モードの終了判定。機構出力は下で一括して決める。
                if (mode == RobotMode.SHOOT) {
                    if (gamepad1.right_bumper) {
                        wasShoot = true;
                    } else if (wasShoot) {
                        wasShoot = false;
                        mode = RobotMode.MANUAL;
                    }
                }
                boolean cancel = gamepad1.left_stick_button || gamepad1.right_stick_button;
                if (cancel) {
                    mode = RobotMode.MANUAL;
                    wasShoot = false;
                }
                if (mode != RobotMode.SHOOT) {
                    shooterController.setSpeed(standardSpeed);
                }

                // 通常回収だけで詰まりを検出。排出・射撃・キャンセルへ移ると逆転を解除する。
                boolean normalIntake = !cancel && !gamepad1.b
                        && mode != RobotMode.DISCHARGE && mode != RobotMode.SHOOT;
                if (!normalIntake) {
                    isReverse = false;
                    timerRunning = false;
                } else if (isReverse) {
                    if (timer.milliseconds() >= 300) isReverse = false;
                } else if (Math.abs(hardware.reads.velocity(intakeController.Intake, SensorTiming.VELOCITY_INTAKE)) < 100) {
                    if (!timerRunning) {
                        timer.reset();
                        timerRunning = true;
                    } else if (timer.milliseconds() >= 300) {
                        isReverse = true;
                        timerRunning = false;
                        timer.reset();
                    }
                } else {
                    timerRunning = false;
                }

                // 優先順位：キャンセル → B/X排出 → 射撃ゲート → 詰まり逆転 → 通常回収。
                TransportCommands transport = TransportCommands.resolve(
                        cancel, gamepad1.b, mode == RobotMode.DISCHARGE,
                        mode == RobotMode.SHOOT, gamepad1.right_bumper,
                        shooterController.isFeedReady(), isReverse);
                feedController.setShootMode(transport.shootPower);
                intakeController.setShootMode(transport.shootPower);
                if (transport.intakeDirection < 0) intakeController.Intake_Reverse();
                else if (transport.intakeDirection > 0) intakeController.Intake_Forward();
                else intakeController.Intake_Stop();
                if (transport.feedDirection < 0) feedController.Feed_turn_left();
                else if (transport.feedDirection > 0) feedController.Feed_turn_right();
                else feedController.Feed_turn_stop();
                if (transport.raiseExtend) extendController.extendUpNow();
                shooterController.setFiring(mode == RobotMode.SHOOT
                        && transport.shootPower && transport.feedDirection < 0);

                phM1 = System.nanoTime();

                // IMU制御は1周期1回。距離補正が現在方位を参照するため、測距更新より先に実行する。
                if ((mode == RobotMode.SHOOT) || (mode == RobotMode.CLIMB) || (mode == RobotMode.DISCHARGE)) {
                    imuTurnOutput = imuController.update();
                    imuActive = true;
                }

                if ((mode == RobotMode.SHOOT)) {
                    distance.update();
                }
                phM2 = System.nanoTime();

                // 押し直しで1段進む。キャンセルは下降/上昇の停止段階へ移す。
                // 自動解錠120秒の起点は従来どおりisStart開始時。
                climbEnum = climbSequence.update(gamepad1.left_trigger, gamepad2.left_trigger,
                        cancel, allTimer.seconds() > climbTime, climbTimer.milliseconds());
                switch (climbEnum) {
                    case 0:
                        climbController.climbStop();
                        break;
                    case 1:
                        climbController.climbStop();
                        climbController.climbOpen();
                        break;
                    case 2:
                        if(!climbController.isBottomLimit()){
                            climbController.climbDown();
                        } else {
                            climbController.climbStop();
                        }
                        break;
                    case 3:
                        climbController.climbStop();
                        break;
                    case 4:
                        climbController.climbUp();
                        break;
                    case 5:
                        climbController.climbStop();
                        break;
                }

                DriveController.DriveOutput finalOutput;

                // 射撃中のRBは距離・角度合わせより優先して走行を止め、他機構への電力を確保する。
                boolean cutDriveForShot = (mode == RobotMode.SHOOT) && gamepad1.right_bumper;

                // 自動旋回中も手動補助の角速度履歴を更新し、手動復帰時の差分ジャンプを防ぐ。
                double assistTurnOut = 0.0;
                boolean assistActive = false;
                if (imuActive || cutDriveForShot) {
                    driveAssist.observe(Angle, driveDt);
                    driveAssist.standby();
                } else {

                    assistTurnOut = driveAssist.update(
                            turnSlewed,
                            forwardSlewed,
                            driveController.manualTurnOutput(turnSlewed),
                            Angle, driveDt);
                    assistActive = driveAssist.isActive();
                }

                // 距離制御を使うかどうかは生入力で判定し、手動出力には制限後の入力を使う。
                boolean manualFwdInShoot =
                        (mode == RobotMode.SHOOT) && Math.abs(forward) > SHOOT_MANUAL_FWD_DEADZONE;

                // 優先順位：発射停止 → 距離+IMU → 手動前後+IMU → 手動補助 → 通常走行。
                if (cutDriveForShot) {
                    finalOutput = new DriveController.DriveOutput(0, 0);
                } else if ((mode == RobotMode.SHOOT) && distance.isUsable() && !manualFwdInShoot) {
                    finalOutput = driveController.driveWithDistanceAndImu(
                            distance.getOutput(),
                            imuTurnOutput,
                            distance.getAngleError()
                    );
                } else if (imuActive) {
                    finalOutput = driveController.driveWithImuCorrection(
                            forwardSlewed,
                            imuTurnOutput
                    );
                } else if (assistActive) {

                    finalOutput = driveController.driveWithImuCorrection(
                            forwardSlewed, assistTurnOut);
                } else {

                    finalOutput = manualOutput;
                }

                // 自動制御中は未使用の手動加速制限状態をリセットし、復帰時の出力飛びを防ぐ。
                if (cutDriveForShot || ((mode == RobotMode.SHOOT) && distance.isUsable() && !manualFwdInShoot)) {
                    fwdSlew.reset(0.0);
                    turnSlew.reset(0.0);
                }

                phM3 = System.nanoTime();
                // 左右の走行出力を確定し、各モーターへ1回ずつ指令する。
                driveLeftOut.set(finalOutput.leftPower);
                driveRightOut.set(finalOutput.rightPower);
                phT4 = System.nanoTime();

                double shL = hardware.reads.velocity(shooterController.leftShooter, SensorTiming.VELOCITY_SHOOTER);
                double shR = hardware.reads.velocity(shooterController.rightShooter, SensorTiming.VELOCITY_SHOOTER);
                int cS = (int) ((shL + shR) / 2);

                // テレメトリと診断表示。片側シューター故障の判定もこの間引き内にある。
                boolean showTelem = (System.currentTimeMillis() - lastTelemMs) >= TELEM_INTERVAL_MS;
                if (showTelem) {
                lastTelemMs = System.currentTimeMillis();

                if (shooterOneSideFault) {
                    telemetry.addLine("################################################");
                    telemetry.addData("★★ シューター故障",
                            String.format(Locale.US,
                                    "%sがトルクを出していません（%.1f 秒時点で検知 / %d 回）",
                                    shooterFaultText, shooterFaultAtSec, shooterFaultCount));
                    telemetry.addLine("  配線・コネクタ・モーターを確認すること。");
                    telemetry.addLine("  速度は左右一致するので速度では分かりません。電流で見ます。");
                    telemetry.addLine("################################################");
                }
                telemetry.addData("目標角度", extendController.targetAngle);
                telemetry.addData("現在角度", extendController.current);
                telemetry.addData("現在出力", extendController.output);
                telemetry.addData("現在速度", cS);
                telemetry.addData("詰まり検知タイマー", timerRunning ? timer.milliseconds() : 0);
                telemetry.addData("自動逆転中", isReverse);

                telemetry.addData("シュート目標(gp2 dpad)", String.format(Locale.US,
                        "%.0f   送り許可 %.0f 以上（止め %.0f）",
                        shootSpeed, shooterController.feedStartSpeed,
                        shooterController.feedStopSpeed));
                telemetry.addData("シューター対目標速度誤差", Math.abs(cS - shootSpeed));

                telemetry.addData("送り/収納", (mode != RobotMode.SHOOT) ? "シュートモードではありません"
                        : (shooterController.isFeedReady() ? "許可（速度 OK）"
                        : String.format(Locale.US, "待機  %.0f → %.0f 以上になるまで送りません",
                                shooterController.lastSpeed, shooterController.feedStopSpeed)));

                if (shootSpeed <= shooterController.feedStartSpeed) {
                    telemetry.addLine(String.format(Locale.US,
                            "★ 目標 %.0f がフィード許可速度 %.0f 以下です。この設定では送りません",
                            shootSpeed, shooterController.feedStartSpeed));
                }
                telemetry.addData("シューター速度左", shL);
                telemetry.addData("シューター速度右", shR);

                telemetry.addData("シューター左右差(速度)", Math.abs(shL - shR)
                        + (Math.abs(shL - shR) > 200 ? "  ← 大きい。片側を確認" : ""));

                double scL = logger.getShooterCurL();
                double scR = logger.getShooterCurR();
                double scSum = scL + scR;
                String scWarn = "";
                if (scSum > 1.5) {
                    double ratio = Math.min(scL, scR) / Math.max(scL, scR);
                    if (ratio < 0.25) {
                        scWarn = (scL > scR) ? "  ← 右が働いていない" : "  ← 左が働いていない";
                    } else if (ratio < 0.55) {
                        scWarn = "  ← 偏りが大きい";
                    }
                }

                if (scSum > 1.5 && Math.min(scL, scR) / Math.max(scL, scR) < 0.25) {
                    shooterFaultCount++;
                    if (!shooterOneSideFault) {
                        shooterOneSideFault = true;
                        shooterFaultAtSec   = allTimer.seconds();
                        shooterFaultText    = (scL > scR) ? "右" : "左";
                    }
                }
                telemetry.addData("シューター電流 左/右",
                        String.format(Locale.US, "%.2f / %.2f A%s", scL, scR, scWarn));
                telemetry.addData("クライムタイマー",climbTimer.milliseconds());
                telemetry.addData("ポテンショメーター",extendController.current);
                telemetry.addData("展開誤差",extendController.error);
                telemetry.addData("展開目標角度",extendController.targetAngle);
                telemetry.addData("展開詰まり", extendController.isStuck ? "詰まり" : "正常");

                telemetry.addData("展開保護", (extendController.poteFault ? "ポテンショ異常 " : "")
                        + (extendController.stallFault ? "押し込み過ぎ(右バンパーを押し直す。左バンパーで下げても復帰)" : "")
                        + (!extendController.poteFault && !extendController.stallFault ? "正常" : ""));
                if (extendController.yielding) {
                    telemetry.addLine("★ 展開を止めてシューターへ電流を譲っています"
                            + "（詰まり + 速度未達）。速度が戻れば展開も再開します");
                }
                telemetry.addData("展開FF+PID", extendController.ffOut + " + " + extendController.pidOut);
                telemetry.addData("IMU現在角度", Angle);
                telemetry.addData("IMU目標角度", imuController.getTarget());
                telemetry.addData("IMU誤差", imuController.getError());
                telemetry.addData("IMU角速度", imuController.getRate());
                telemetry.addData("IMU出力内訳", "P " + imuController.pTerm + " I " + imuController.iTerm
                        + " D " + imuController.dTerm + " S " + imuController.sTerm);
                telemetry.addData("IMUループdt[ms]", imuController.getDt() * 1000.0);
                telemetry.addData("ドライブ左出力", finalOutput.leftPower);
                telemetry.addData("ドライブ右出力", finalOutput.rightPower);
                telemetry.addData("IMU補正走行中", imuActive);

                telemetry.addData("ヨー補助", String.format(Locale.US,
                        "%s  目標 %.1f  実角速度 %.1f deg/s  誤差 %.2f  出力 %.3f",
                        driveAssist.getModeText(), driveAssist.getTargetHeading(),
                        driveAssist.getRate(), driveAssist.getError(), driveAssist.getOutput()));
                telemetry.addData("距離", distance.getDistance());
                telemetry.addData("正面距離", distance.getFrontDistance());
                telemetry.addData("距離目標", distance.getTargetDistance());
                telemetry.addData("距離PID出力", distance.getOutput());
                telemetry.addData("角度誤差", distance.getAngleError());

                telemetry.addData("測距スレッド", String.format(Locale.US,
                        "%s  例外 %d 回  最終サンプル %s  生値 %.1f cm",
                        distance.isReaderRunning() ? "稼働" : "停止",
                        distance.getReaderErrors(),
                        distance.getSampleAgeMs() < 0 ? "まだ無い"
                                : (distance.getSampleAgeMs() + " ms前"),
                        distance.getSensorRaw()));
                telemetry.addData("測距の健全性", (distance.isValid() ? "OK" : "異常")
                        + "  連続失敗 " + distance.getFailCount() + " 回"
                        + "  失敗率 " + (int) distance.getFailRate() + "%");
                telemetry.addData("周期オーバーラン",
                        String.format(Locale.US, "%d / %d 回 (%.1f%%)  最大 +%.1f ms  設定 %d ms",
                                overrunCount, loopCountTotal,
                                loopCountTotal > 0 ? 100.0 * overrunCount / loopCountTotal : 0.0,
                                overrunWorstMs, LOOP_PERIOD_MS));
                telemetry.addData("書き込み省略率",
                        "ドライブ " + (int) driveLeftOut.getSkipRate()
                        + "%  展開 " + (int) extendController.getSkipRate()
                        + "%  シューター " + (int) shooterController.getSkipRate()
                        + "%  フィード " + (int) feedController.getSkipRate()
                        + "%  インテーク " + (int) intakeController.getSkipRate()
                        + "%  クライム " + (int) climbController.getSkipRate() + "%");
                telemetry.addData("ログ", logger.isFailed() ? "書き込み失敗"
                        : (logger.getRows() + " 行  " + logger.getPath()));
                telemetry.addData("ドライブ", cutDriveForShot
                        ? "停止中（発射のため電流を譲っています）" : "有効");
                telemetry.addData("前後の制御", (mode != RobotMode.SHOOT) ? "-"
                        : manualFwdInShoot ? "ドライバー（手動優先。距離センサーを降ろしています）"
                        : distance.isUsable() ? "距離センサー"
                        : "ドライバー（測距が使えない）");
                telemetry.update();
                }
                phT5 = System.nanoTime();

                // 区間時間はms。待機時間は前周期分、ログ書き込み時間はロガー側で測る。
                logger.setPhases(
                        (phT1 - phT0) / 1e6,
                        (phT2 - phT1) / 1e6,
                        (phT3 - phT2) / 1e6,
                        (phT4 - phT3) / 1e6,
                        (phT5 - phT4) / 1e6,
                        lastHoldMs);

                logger.setPeriod(periodMs, overrunCount, loopCountTotal);

                logger.setModeParts(
                        (phM1 - phT3) / 1e6,
                        (phM2 - phM1) / 1e6,
                        (phM3 - phM2) / 1e6,
                        (phT4 - phM3) / 1e6);
                // CSVは50ms間隔で記録。モードは排他状態から変換する。
                logger.update(gamepad1, gamepad2,
                        finalOutput.leftPower, finalOutput.rightPower,
                        (mode == RobotMode.SHOOT), (mode == RobotMode.CLIMB), (mode == RobotMode.DISCHARGE), climbEnum);

                prevA = gamepad1.a;

            } else {
                telemetry.addData("展開出力",extendController.output);
                telemetry.addData("展開モーター出力", extendController.extend.getPower());
                telemetry.addData("展開詰まり", extendController.isStuck ? "詰まり" : "正常");
                telemetry.addData("展開逆転",extendController.reverse);
                telemetry.update();
                allTimer.reset();
                if (Math.abs(gamepad1.left_stick_y) > 0.05) {
                    isStart = true;
                    hardware.timing.start();
                }
            }

            // ボタンの前回値を毎周期更新する。
            prevGamepad2DpadUp = gamepad2.dpad_up;
            prevGamepad2DpadDown = gamepad2.dpad_down;
            prevX = gamepad1.x;
            prevY = gamepad1.y;
            prevRB = gamepad1.right_bumper;

            // 周期の残り時間を待つ。超過時は期限を現在時刻へ戻して遅れを持ち越さない。
            long workEnd = System.nanoTime();
            hardware.timing.record(SensorTiming.LOOP_WORK, phT0, workEnd, false);
            if (isStart) {
                int workMode = mode == RobotMode.SHOOT
                        ? (gamepad1.right_bumper ? SensorTiming.WORK_FIRE : SensorTiming.WORK_SHOOT)
                        : mode == RobotMode.CLIMB ? SensorTiming.WORK_CLIMB
                        : mode == RobotMode.DISCHARGE ? SensorTiming.WORK_DISCHARGE : SensorTiming.WORK_MANUAL;
                hardware.timing.record(workMode, phT0, workEnd, false);
            }
            loopCountTotal++;
            if (LOOP_PERIOD_NS > 0) {
                loopDeadline += LOOP_PERIOD_NS;
                long nowNs = System.nanoTime();
                if (nowNs > loopDeadline) {
                    hardware.timing.record(SensorTiming.LOOP_OVERRUN, loopDeadline, nowNs, false);
                    double overMs = (nowNs - loopDeadline) / 1e6;
                    if (overMs > overrunWorstMs) overrunWorstMs = overMs;
                    overrunCount++;
                    loopDeadline = nowNs;
                    lastHoldMs = 0.0;
                } else {

                    long waitStart = System.nanoTime();
                    long remaining = loopDeadline - System.nanoTime();
                    if (remaining > 2_000_000L) {
                        try {
                            Thread.sleep((remaining - 1_500_000L) / 1_000_000L);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                    while (System.nanoTime() < loopDeadline) {
                        if (!opModeIsActive()) break;
                    }
                    lastHoldMs = (System.nanoTime() - waitStart) / 1e6;
                }
            }

        }

        // 通常終了時の後処理。例外経路はtry/finally化が必要。
        distance.stopReader();
        logger.close();
    }

    // 目標速度の変更に送り閾値を追従させる。非射撃中は待機速度を維持する。
    private void applyShootSpeed() {
        shooterController.setFeedGateForTarget(shootSpeed);
        if ((mode == RobotMode.SHOOT)) {
            shooterController.setSpeed(shootSpeed);
        }
    }

    // 角度差を-180～180degへ正規化する。
    private static double normalizeAngle(double deg) {
        while (deg >  180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
