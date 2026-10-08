package org.firstinspires.ftc.teamcode;
import java.util.Arrays;

/** PC-only tests; never upload this file to OnBot Java. */
public class PreflightPolicyTest {
    static int checks;
    static void check(boolean condition,String name) {
        checks++; if(!condition) throw new AssertionError(name);
    }
    public static void main(String[] args) {
        check(!Preflight_R4.Policy.motorWindow(25.99,0,0),"no drive before motor schedule");
        check(Preflight_R4.Policy.motorWindow(26,0,0),"first motor start");
        check(!Preflight_R4.Policy.motorWindow(26.55,0,0),"pulse deadline");
        check(!Preflight_R4.Policy.motorWindow(28,0,0),"old motor cannot run in next slot");
        check(!Preflight_R4.Policy.motorWindow(31,2,1),"no inertial shooter reversal");
        check(!Preflight_R4.Policy.motorWindow(55,7,1),"absolute finish deadline");
        check(!Preflight_R4.Policy.motorWindow(Double.NaN,0,0),"bad time forbids drive");
        check(Preflight_R4.Policy.servoDelta(42)==.15,"servo forward start");
        check(Preflight_R4.Policy.servoDelta(42.6)==0,"servo forward deadline");
        check(Preflight_R4.Policy.servoDelta(43.6)==-.15,"servo reverse start");
        check(Preflight_R4.Policy.servoDelta(44.2)==0,"servo reverse deadline");
        check(Preflight_R4.Policy.servoDelta(55)==0,"servo finish");
        check(!Preflight_R4.Policy.angleReady(Double.NaN),"NaN angle");
        check(!Preflight_R4.Policy.angleReady(Double.POSITIVE_INFINITY),"Infinite angle");
        check(!Preflight_R4.Policy.angleReady(9.99),"low starting angle");
        check(Preflight_R4.Policy.angleReady(10),"start boundary low");
        check(Preflight_R4.Policy.angleReady(55),"start boundary high");
        check(!Preflight_R4.Policy.angleReady(55.01),"high starting angle");
        check(!Preflight_R4.Policy.angleSafe(5),"low hard limit");
        check(!Preflight_R4.Policy.angleSafe(60),"high hard limit");
        check(Preflight_R4.Policy.angleSafe(40),"valid angle");
        check(!Preflight_R4.Policy.electricalOK(Double.NaN,0,1.2),"NaN battery");
        check(!Preflight_R4.Policy.electricalOK(Double.POSITIVE_INFINITY,0,1.2),"no voltage sensor");
        check(!Preflight_R4.Policy.electricalOK(10.99,.1,1.2),"undervoltage");
        check(!Preflight_R4.Policy.electricalOK(12,Double.NaN,1.2),"NaN current");
        check(!Preflight_R4.Policy.electricalOK(12,-.1,1.2),"negative current");
        check(!Preflight_R4.Policy.electricalOK(12,1.21,1.2),"overcurrent");
        check(Preflight_R4.Policy.electricalOK(11,1.2,1.2),"electrical boundaries");
        check(!Preflight_R4.Policy.responseOK(100,0,10),"back-driven/disconnected powered motor");
        check(!Preflight_R4.Policy.responseOK(0,1,10),"stalled motor or missing encoder");
        check(!Preflight_R4.Policy.responseOK(-100,.2,10),"reversed encoder");
        check(!Preflight_R4.Policy.responseOK(10,.2,2),"too few samples");
        check(!Preflight_R4.Policy.responseOK(10,Double.NaN,10),"unknown current");
        check(Preflight_R4.Policy.responseOK(4,.03,3),"response boundary");
        for(int i=0;i<6;i++) {
            check(Preflight_R4.Policy.servoPosition(i,42.1)==(i<4?.5:.65),"UP only locks "+i);
            check(Preflight_R4.Policy.servoPosition(i,43.8)==.35,"DOWN all six equally "+i);
            check(Preflight_R4.Policy.servoPosition(i,44.2)==.5,"STOP all six "+i);
            check(Preflight_R4.Policy.servoReversed(i)==(i==0||i==3||i==5),"Hub directions "+i);
        }
        Preflight_R4.RangeCheck yaw=new Preflight_R4.RangeCheck();
        for(int i=0;i<5;i++)yaw.sample(179,-180,180);
        yaw.sample(-179,-180,180);check(!yaw.ok(5),"yaw wrap is only 2 degrees");
        yaw.sample(-174,-180,180);check(yaw.ok(5),"actual yaw change");
        yaw.sample(Double.NaN,-180,180);check(!yaw.ok(5),"invalid yaw latches review");
        Preflight_R4.RangeCheck frozen=new Preflight_R4.RangeCheck();
        for(int i=0;i<10;i++)frozen.sample(30,-180,180);
        check(!frozen.ok(5),"frozen IMU");
        Preflight_R4.PadCheck pad=new Preflight_R4.PadCheck();
        boolean[] buttons=new boolean[12];double[] axes=new double[4],triggers=new double[2];
        check(!pad.valuesOK(),"empty input");
        pad.sampleValues(buttons,axes,triggers);
        Arrays.fill(buttons,true);Arrays.fill(axes,1);Arrays.fill(triggers,1);
        pad.sampleValues(buttons,axes,triggers);check(!pad.valuesOK(),"held buttons not pass");
        Arrays.fill(axes,-1);pad.sampleValues(buttons,axes,triggers);
        Arrays.fill(buttons,false);Arrays.fill(axes,0);Arrays.fill(triggers,0);
        pad.sampleValues(buttons,axes,triggers);check(pad.valuesOK(),"complete control cycle");
        axes[0]=Double.NaN;pad.sampleValues(buttons,axes,triggers);
        check(!pad.valuesOK(),"invalid analog after previous pass");
        Preflight_R4.PadCheck stuck=new Preflight_R4.PadCheck();
        Arrays.fill(buttons,true);Arrays.fill(axes,1);Arrays.fill(triggers,1);
        for(int i=0;i<10;i++)stuck.sampleValues(buttons,axes,triggers);
        check(!stuck.valuesOK(),"stuck at start");
        System.out.println("PASS: "+checks+" decision checks (not a hardware test)");
    }
}
