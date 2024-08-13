package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Predicate;

//All measurements are in seconds
public class TimeManager {
    private static double initTime;
    private static double currentTime = -1;
    private static double deltaTime = 1;

    private static List<Predicate<Double>> tasks = new ArrayList<>();

    //Called at the end of every tick
    public static void onTick(double time) {
        if (currentTime != -1) deltaTime = time - currentTime;
        else initTime = time;
        currentTime = time;
        tasks.removeIf(task -> task.test(currentTime));
    }

    public static double getTime(boolean includeInit) {
        return currentTime - (includeInit ? 0 : initTime);
    }

    //Time it took to run the LAST cycle
    public static double getDeltaTime() {
        return deltaTime;
    }

    public static void hookTick(Predicate<Double> task) {
        tasks.add(task);
    }

    public static void hookFuture(double offset, Predicate<Double> task) {
        final double scheduleTime = getTime(false);
        //Do not simplify! task.test() must only run when the first condition is met
        //TODO maybe simplifiying is possible since there is no point in checking the second condition if the first is true?
        tasks.add(time -> (time > scheduleTime + offset ? task.test(time) : false));
    }

    public static void hookPeriodic(double interval, Predicate<Double> task) {
        AtomicInteger iterations = new AtomicInteger(0);
        final double scheduleTime = getTime(false);
        tasks.add(time -> {
            if (time > scheduleTime + interval * iterations.get()) {
                iterations.getAndIncrement();
                return task.test(time);
            }
            return false;
        });
    }

    public static void reset() {
        currentTime = -1;
        deltaTime = 1;
        tasks.clear();
    }
}
