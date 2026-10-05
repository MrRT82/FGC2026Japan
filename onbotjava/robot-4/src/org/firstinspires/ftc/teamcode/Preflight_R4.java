// 2026-10-05: Robot 4 only, review candidate, not hardware validated.
// Reference: live Robot 4 Hub source, hub_reference_20261005/teamcode.
// Removed ranging and touch dependencies; lock directions and climb phases match the Hub source.
// Upload only this file after review. No dependency on match classes.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import java.util.*;

@TeleOp(name="R4_55sec_Check_REVIEW", group="9_DIAGNOSTIC")
public class Preflight_R4 extends LinearOpMode {
    // Conservative screening candidates, NOT validated motor specifications.
    static final String[] MN = {"leftmotor", "rightmotor", "left_shooter", "right_shooter",
        "intakemotor", "feed", "climb", "extendmotor"};
    static final String[] ML = {"走行左", "走行右", "射出左", "射出右", "回収", "搬送", "登坂ローラー", "展開"};
    static final String[] SN = {"rightInside", "rightOutside", "leftInside", "leftOutside", "rightLock", "leftLock"};
    static final String[] SL = {"右内", "右外", "左内", "左外", "右ロック", "左ロック"};
    static final double[] POWER = {.18,.18,.20,.20,.20,.20,.18,.15};
    final DcMotorEx[] motors = new DcMotorEx[8];
    final Servo[] servos = new Servo[6]; // SDK Servo API, physically continuous rotation.
    final String[] motorResult = new String[8];
    final boolean[] servoSeen = new boolean[6];
    final PadCheck[] pads = {new PadCheck(),new PadCheck()};
    final RangeCheck yawRange = new RangeCheck();
    final List<String> problems = new ArrayList<String>();
    final List<LynxModule> hubs = new ArrayList<LynxModule>();
    final List<LynxModule.BulkCachingMode> cacheModes = new ArrayList<LynxModule.BulkCachingMode>();
    AnalogInput pot; IMU imu;
    boolean lockUpCommanded=false, allDownCommanded=false;
    boolean mapped=true, aborted=false, stoppedBad=false, servoMoved=false, motorVisual=false;
    boolean armed=false, sawArmReleased=false, prevA;
    final boolean[] prevConfirm = new boolean[6];
    long startNs, lastPacketNs, lastStamp=Long.MIN_VALUE, lastLoopNs;
    int padId=-1, activeMotor=-1, pulse=-1, lastStage=-1;
    double angle=Double.NaN, voltage=Double.NaN, initialAngle, pulseAngle;
    double pulseStartPos, currentSum; int currentCount; double signedTravel;
    final boolean[] directionOK = new boolean[2];
    boolean motorSkipped=false, pulseEnded=false;
    String phase="準備", abortReason="", sensorProblem="";

    @Override public void runOpMode() {
        Arrays.fill(motorResult,"未実施");
        try {
            discover();
            stopAll();
            telemetry.setMsTransmissionInterval(100);
            telemetry.addLine("Robot 4 / 55秒・2人 / 未実機検証版");
            telemetry.addLine("車輪を浮かせ全機構を支持、ボールなし。展開は10～55度。");
            telemetry.addLine("START後24秒: 2台の全入力。補助: 機体の向き。");
            telemetry.addLine("24秒: 一度LB/RBを離す→26秒までに両方押す。駆動中は押し続ける。");
            telemetry.addLine("離す / B / DS STOPで中止。終了後は目視結果を入力。");
            for(String p:problems) telemetry.addLine(p);
            telemetry.update();
            waitForStart();
            if(isStopRequested()) return;
            startNs=System.nanoTime(); lastLoopNs=startNs;
            while(opModeIsActive() && elapsed()<55 && !aborted) {
                long now=System.nanoTime();
                double t=elapsed();
                if(t>=26 && t<48 && now-lastLoopNs>250_000_000L) abort("制御周期遅延 >250ms");
                lastLoopNs=now;
                if(gamepad1.timestamp!=lastStamp) {lastStamp=gamepad1.timestamp;lastPacketNs=now;}
                for(LynxModule h:hubs) h.clearBulkCache();
                if(t<24) {
                    phase="1/4 入力・センサー: 両スティック一周+押込/全ボタン/LT RT→離す";
                    pads[0].sample(gamepad1); pads[1].sample(gamepad2);
                    sampleSensors();
                } else if(t<26) {
                    phase="2/4 手を離す! GP1 LB/RBを一度離し、両方押して待つ";
                    if(!gamepad1.left_bumper && !gamepad1.right_bumper) sawArmReleased=true;
                    armed=sawArmReleased && gamepad1.left_bumper && gamepad1.right_bumper
                        && neutralForArm(gamepad1) && neutralForArm(gamepad2);
                    padId=gamepad1.id;
                } else if(t<42) {
                    if(lastStage<26) {
                        if(!mapped || !pads[0].ok() || !pads[1].ok() || !armed || padId<0
                            || gamepad2.id<0 || gamepad2.id==padId || padId!=pads[0].id
                            || gamepad2.id!=pads[1].id) abort("入力/機器/開始確認不足: 駆動を省略");
                    }
                    checkDeadman(now);
                    if(!aborted) runMotor((int)((t-26)/2), (t-26)%2);
                } else if(t<48) {
                    if(activeMotor>=0) finishMotor();
                    phase="3/4 ロックのみ上昇→6台下降→停止を目視";
                    checkDeadman(now);
                    if(!aborted) runServos(t-42);
                } else {
                    if(lastStage<48) stopAll();
                    phase="4/4 目視入力: A=全モーター方向/伝達/停止確認";
                    boolean[] c={gamepad1.dpad_up,gamepad1.dpad_right,gamepad1.dpad_left,
                        gamepad1.dpad_down,gamepad1.x,gamepad1.y};
                    if(gamepad1.a && !prevA) motorVisual=true;
                    for(int i=0;i<6;i++) if(c[i]&&!prevConfirm[i]&&servoMoved) servoSeen[i]=true;
                }
                // Preserve input edges through ALL stages; held buttons cannot confirm on entry.
                boolean[] c={gamepad1.dpad_up,gamepad1.dpad_right,gamepad1.dpad_left,
                    gamepad1.dpad_down,gamepad1.x,gamepad1.y};
                System.arraycopy(c,0,prevConfirm,0,6); prevA=gamepad1.a;
                lastStage=(int)t;
                show(false);
                sleep(20);
            }
        } catch(Exception e) {
            // Never expose SDK exception contents (may contain device identity).
            abort("機器/API例外: "+e.getClass().getSimpleName());
        } finally {
            stopAll();
            for(int i=0;i<hubs.size();i++) try {hubs.get(i).setBulkCachingMode(cacheModes.get(i));}
                catch(Exception e) {stoppedBad=true;}
        }
        if(activeMotor>=0 && motorResult[activeMotor].equals("検査中")) motorResult[activeMotor]="未完了: 中止";
        while(opModeIsActive()) {show(true); sleep(100);}
    }

    private <T> T lookup(Class<? extends T> type,String name) {
        try {return hardwareMap.get(type,name);} catch(Exception e) {
            mapped=false; problems.add("要確認: "+name+" 設定/通信"); return null;
        }
    }
    private void discover() {
        // Register all devices before configuration so finally can stop partial initialization.
        for(int i=0;i<8;i++) motors[i]=lookup(DcMotorEx.class,MN[i]);
        for(int i=0;i<6;i++) servos[i]=lookup(Servo.class,SN[i]);
        stopAll();
        for(int i=0;i<8;i++) if(motors[i]!=null) {
            motors[i].setDirection(i==1||i==3||i==4 ? DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(i==2||i==3 ? DcMotor.ZeroPowerBehavior.FLOAT:DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for(int i=0;i<6;i++) if(servos[i]!=null)
            servos[i].setDirection(Policy.servoReversed(i) ? Servo.Direction.REVERSE:Servo.Direction.FORWARD);
        pot=lookup(AnalogInput.class,"potentiometer");
        imu=lookup(IMU.class,"imu");
        if(imu!=null && !imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)))) {
            mapped=false; problems.add("要確認: IMU初期化失敗");
        }
        for(LynxModule h:hardwareMap.getAll(LynxModule.class)) {
            cacheModes.add(h.getBulkCachingMode()); hubs.add(h);
            h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    private double elapsed() {return (System.nanoTime()-startNs)/1e9;}
    private void sampleSensors() {
        try {
            if(imu!=null) {
                double y=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                yawRange.sample(y,-180,180);
            }
            angle=readAngle(); voltage=readVoltage();
        } catch(Exception e) {sensorProblem="要確認: センサー読取例外";}
    }
    private double readAngle() {
        if(pot==null) return Double.NaN;
        double max=pot.getMaxVoltage(), v=pot.getVoltage();
        return finite(max)&&max>0&&finite(v)&&v>0.05&&v<max-.05 ? v/max*135:Double.NaN;
    }
    private double readVoltage() {
        double v=Double.POSITIVE_INFINITY;
        for(VoltageSensor s:hardwareMap.voltageSensor) {
            double x=s.getVoltage(); if(finite(x)&&x>0) v=Math.min(v,x);
        }
        return v;
    }
    private void checkDeadman(long now) {
        if(!gamepad1.left_bumper || !gamepad1.right_bumper || gamepad1.b
            || gamepad1.id!=padId || now-lastPacketNs>750_000_000L) abort("操作解除/B/通信更新なし: 中止");
    }
    private boolean neutralForArm(Gamepad g) {
        return Math.abs(g.left_stick_x)<=.1&&Math.abs(g.left_stick_y)<=.1
            &&Math.abs(g.right_stick_x)<=.1&&Math.abs(g.right_stick_y)<=.1
            &&g.left_trigger<=.1&&g.right_trigger<=.1&&!g.a&&!g.b&&!g.x&&!g.y
            &&!g.dpad_up&&!g.dpad_down&&!g.dpad_left&&!g.dpad_right
            &&!g.left_stick_button&&!g.right_stick_button;
    }
    private void drive(int i,double p) {
        // Recheck immediately before every nonzero hardware write, including after slow reads.
        checkDeadman(System.nanoTime());
        if(!aborted && opModeIsActive() && Policy.motorWindow(elapsed(),i,pulse)) motors[i].setPower(p);
        else motors[i].setPower(0);
    }
    private void runMotor(int index,double local) {
        if(index!=activeMotor) {
            if(activeMotor>=0) finishMotor();
            stopAll(); activeMotor=index; pulse=-1; pulseEnded=false; motorSkipped=false;
            directionOK[0]=false;directionOK[1]=false;
            motorResult[index]="検査中";
            if(index==2||index==3) {
                if(Math.abs(motors[2].getVelocity())>15 || Math.abs(motors[3].getVelocity())>15) {
                    motorSkipped=true; motorResult[index]="要確認: 同軸の残留回転";
                }
            }
            if(index==7) {
                initialAngle=readAngle();
                if(!Policy.angleReady(initialAngle)) {motorSkipped=true;motorResult[index]="未実施: 展開を10～55度へ";}
            }
        }
        phase="2/4 "+ML[index]+" / LB+RB保持 / Bで中止";
        if(motorSkipped) return;
        // Shooter one direction only: isolate electrical contribution, avoid reversing an inertial shaft.
        boolean shooter=index==2||index==3;
        int nextPulse=local<1?0:1;
        if(shooter && nextPulse==1) {endPulse(index);return;}
        if(nextPulse!=pulse) {
            endPulse(index);
            pulse=nextPulse; pulseEnded=false;
            pulseStartPos=motors[index].getCurrentPosition();
            pulseAngle=index==7?readAngle():0; currentSum=0;currentCount=0;signedTravel=0;
        }
        double within=local-nextPulse;
        if(within>=.55 || pulseEnded) {endPulse(index);return;}
        voltage=readVoltage();
        double amps=motors[index].getCurrent(CurrentUnit.AMPS);
        double limit=index==7?1.2:2.5;
        if(!Policy.electricalOK(voltage,amps,limit)) {abort(ML[index]+" 電圧/電流異常");return;}
        if(index==7) {
            angle=readAngle();
            if(!Policy.angleSafe(angle) || Math.abs(angle-initialAngle)>5) {abort("展開角度範囲/変位超過");return;}
            if(Math.abs(angle-pulseAngle)>=2) {endPulse(index);return;}
        }
        if(within>=.15) {currentSum+=amps;currentCount++;}
        signedTravel=(motors[index].getCurrentPosition()-pulseStartPos)*(pulse==0?1:-1);
        drive(index,POWER[index]*(pulse==0?1:-1));
    }
    private void endPulse(int i) {
        if(pulse<0||pulseEnded) return;
        motors[i].setPower(0); pulseEnded=true;
        signedTravel=(motors[i].getCurrentPosition()-pulseStartPos)*(pulse==0?1:-1);
        double mean=currentCount==0?Double.NaN:currentSum/currentCount;
        directionOK[pulse]=Policy.responseOK(signedTravel,mean,currentCount);
        // Encoder + potentiometer must both react. Do not infer correct physical polarity here.
        if(i==7) directionOK[pulse] &= finite(readAngle())&&Math.abs(readAngle()-pulseAngle)>=.3;
    }
    private void finishMotor() {
        int i=activeMotor;
        if(i<0) return;
        if(!motorSkipped) {
            endPulse(i);
            boolean ok=directionOK[0]&&((i==2||i==3)||directionOK[1]);
            motorResult[i]=ok?"反応確認":"要確認: 配線/回転/電流/試験出力";
        }
        activeMotor=-1; pulse=-1;
    }
    private void runServos(double local) {
        // Current Hub: UP moves locks only; DOWN moves climb and locks at equal command.
        // Restrict extension below 60 degrees; do not run the full match climb sequence.
        angle=readAngle();
        if(!Policy.angleSafe(angle)) {abort("登坂前/中の展開角度: 5～60度外");return;}
        double delta=local<.6?.15:local<1.6?0:local<2.2?-.15:0;
        if(delta!=0) {
            voltage=readVoltage();
            if(!finite(voltage)||voltage<11.0) {abort("サーボ試験: 電圧異常");return;}
        }
        for(int i=0;i<6;i++) {
            if(delta!=0) checkDeadman(System.nanoTime());
            if(aborted||!opModeIsActive()||elapsed()>=48) {stopAll();return;}
            // A slow read/write must not extend a pulse beyond its absolute schedule.
            double actualDelta=Policy.servoDelta(elapsed());
            if(actualDelta!=delta) {stopAll();return;}
            servos[i].setPosition(Policy.servoPosition(i,elapsed()));
        }
        if(delta>0) lockUpCommanded=true;
        if(delta<0) allDownCommanded=true;
        if(local>=2.2) servoMoved=lockUpCommanded&&allDownCommanded;
    }
    private void abort(String reason) {
        if(!aborted) {abortReason=reason;aborted=true;}
        stopAll();
    }
    private void stopAll() {
        for(DcMotorEx m:motors) if(m!=null) try {m.setPower(0);} catch(Exception e) {stoppedBad=true;}
        for(Servo s:servos) if(s!=null) try {s.setPosition(.5);} catch(Exception e) {stoppedBad=true;}
        if(stoppedBad) {aborted=true;abortReason="停止指令失敗: DS STOP/主電源を確認";}
    }
    private void show(boolean done) {
        telemetry.clearAll();
        telemetry.addData("R4",done?"終了・出力停止 / DS STOPで閉じる":String.format(Locale.US,"残り %.0f秒",Math.max(0,55-elapsed())));
        telemetry.addLine(aborted?"中止: "+abortReason:done?
            (screenComplete()?"簡易検査成立（負荷性能は未検証）":"要確認あり / 未実施を正常扱いしない"):phase);
        telemetry.addData("GP1",pads[0].missing());telemetry.addData("GP2",pads[1].missing());
        telemetry.addData("IMU",yawRange.ok(5)?"反応確認":"未確認: 向きを5度以上変える");
        telemetry.addData("電圧/展開角",String.format(Locale.US,"%.1f V / %.1f 度",voltage,angle));
        int motorPass=0;for(String result:motorResult)if(result.equals("反応確認"))motorPass++;
        telemetry.addData("モーター",motorPass+" / 8 反応確認");
        for(int i=0;i<8;i++) {
            if(done&&!motorResult[i].equals("反応確認") || !done&&i==activeMotor)
                telemetry.addData(ML[i],motorResult[i]);
        }
        telemetry.addData("モーター目視(A)",motorVisual?"確認済":"未確認: 方向/伝達/停止");
        String[] keys={"↑","→","←","↓","X","Y"};
        StringBuilder s=new StringBuilder();
        for(int i=0;i<6;i++) s.append(SL[i]).append(servoSeen[i]?":確認 ":"("+keys[i]+"):未 ");
        telemetry.addLine(s.toString());
        if(!sensorProblem.isEmpty()) telemetry.addLine(sensorProblem);
        for(String p:problems) telemetry.addLine(p);
        telemetry.update();
    }
    private boolean screenComplete() {
        if(aborted||stoppedBad||!mapped||!motorVisual||!sensorProblem.isEmpty()
            ||!pads[0].ok()||!pads[1].ok()
            ||!yawRange.ok(5)) return false;
        for(String s:motorResult) if(!s.equals("反応確認")) return false;
        for(boolean b:servoSeen) if(!b) return false;
        return true;
    }
    static boolean finite(double x) {return !Double.isNaN(x)&&!Double.isInfinite(x);}

    // Pure decision helpers, executable in PC tests without physical hardware.
    public static class Policy {
        public static boolean motorWindow(double t,int motor,int direction) {
            if(!finite(t)||motor<0||motor>7||direction<0||direction>1)return false;
            if((motor==2||motor==3)&&direction==1)return false;
            double begin=26+motor*2+direction;
            return t>=begin&&t<begin+.55;
        }
        public static double servoDelta(double t) {
            return t>=42&&t<42.6?.15:t>=43.6&&t<44.2?-.15:0;
        }
        public static boolean servoReversed(int i) {return i==0||i==3||i==5;}
        public static double servoPosition(int i,double t) {
            double delta=servoDelta(t);
            return .5+(i<4&&delta>0?0:delta);
        }
        public static boolean angleReady(double a) {return finite(a)&&a>=10&&a<=55;}
        public static boolean angleSafe(double a) {return finite(a)&&a>5&&a<60;}
        public static boolean electricalOK(double v,double a,double limit) {
            return finite(v)&&v>=11&&finite(a)&&a>=0&&a<=limit;
        }
        public static boolean responseOK(double ticks,double amps,int samples) {
            return finite(ticks)&&ticks>=4&&finite(amps)&&amps>=.03&&samples>=3;
        }
    }
    public static class RangeCheck {
        double first=Double.NaN,maxDelta; int samples,invalid;
        public void sample(double x,double low,double high) {
            if(!finite(x)||x<low||x>high) {invalid++;return;}
            if(!finite(first)) first=x;
            double delta=x-first;
            if(low==-180) {while(delta>180)delta-=360;while(delta< -180)delta+=360;}
            maxDelta=Math.max(maxDelta,Math.abs(delta));samples++;
        }
        public boolean ok(double change){return samples>=5&&maxDelta>=change&&invalid==0;}
    }
    public static class PadCheck {
        final int[] buttons=new int[12]; final int[] axes=new int[4]; final int[] triggers=new int[2];
        static final String[] BN={"A","B","X","Y","↑","↓","←","→","LB","RB","L3","R3"};
        static final String[] AN={"左X","左Y","右X","右Y"};
        int id=Integer.MIN_VALUE; boolean invalid=false;
        void sample(Gamepad g) {
            if(g.id<0) {return;}
            if(id==Integer.MIN_VALUE) id=g.id;
            if(id!=g.id) invalid=true;
            boolean[] b={g.a,g.b,g.x,g.y,g.dpad_up,g.dpad_down,g.dpad_left,g.dpad_right,
                g.left_bumper,g.right_bumper,g.left_stick_button,g.right_stick_button};
            double[] a={g.left_stick_x,g.left_stick_y,g.right_stick_x,g.right_stick_y};
            double[] tr={g.left_trigger,g.right_trigger};
            sampleValues(b,a,tr);
        }
        public void sampleValues(boolean[] b,double[] a,double[] tr) {
            for(int i=0;i<12;i++) buttons[i]=buttonStep(buttons[i],b[i]);
            for(int i=0;i<4;i++) {
                if(!finite(a[i])||Math.abs(a[i])>1) {invalid=true;continue;}
                if(axes[i]==0&&Math.abs(a[i])<=.1) axes[i]=1;
                if((axes[i]&1)!=0) {
                    if(a[i]>=.8) axes[i]|=2;if(a[i]<=-.8) axes[i]|=4;
                    if((axes[i]&7)==7&&Math.abs(a[i])<=.1) axes[i]|=8;
                }
            }
            for(int i=0;i<2;i++) {
                if(!finite(tr[i])||tr[i]<0||tr[i]>1) {invalid=true;continue;}
                if(triggers[i]==0&&tr[i]<=.1)triggers[i]=1;
                else if(triggers[i]==1&&tr[i]>=.8)triggers[i]=2;
                else if(triggers[i]==2&&tr[i]<=.1)triggers[i]=3;
            }
        }
        static int buttonStep(int s,boolean p) {
            return s==0&&!p?1:s==1&&p?2:s==2&&!p?3:s;
        }
        public boolean valuesOK() {
            if(invalid)return false;
            for(int v:buttons)if(v!=3)return false;
            for(int v:axes)if(v!=15)return false;
            for(int v:triggers)if(v!=3)return false;
            return true;
        }
        boolean ok(){return id>=0&&valuesOK();}
        String missing() {
            if(invalid)return "要確認: 入力値/割当変更";
            if(id<0)return "未接続/未割当";
            if(ok())return "反応確認";
            StringBuilder s=new StringBuilder("残り ");
            for(int i=0;i<12;i++)if(buttons[i]!=3)s.append(BN[i]).append(' ');
            for(int i=0;i<4;i++)if(axes[i]!=15)s.append(AN[i]).append(' ');
            for(int i=0;i<2;i++)if(triggers[i]!=3)s.append(i==0?"LT ":"RT ");
            return s.toString();
        }
    }
}
