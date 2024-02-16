package frc.robot;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class LiveDoubleBinding {
    DoubleSubscriber valueSubscriber;
//TODO: ask about this stuff 
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable shuffleboardTable = inst.getTable("Shuffleboard");

    public LiveDoubleBinding(String tabName, String key, Double defaultValue, Consumer<NetworkTableEvent> listener) {

        Shuffleboard.getTab(tabName).add(key, defaultValue).withWidget(BuiltInWidgets.kNumberSlider);
        valueSubscriber = shuffleboardTable.getDoubleTopic(tabName + "/" + key).subscribe(defaultValue);

        if (listener != null) {
            inst.addListener(
                    valueSubscriber,
                    EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                    listener);
        }
    }

    public LiveDoubleBinding(String tabName, String key, Double defaultValue) {
        Shuffleboard.getTab(tabName).add(key, defaultValue).withWidget(BuiltInWidgets.kNumberSlider);
        valueSubscriber = shuffleboardTable.getDoubleTopic(key).subscribe(defaultValue);
    }

    public DoubleSubscriber getSubscriber() {
        return valueSubscriber;
    }

    public double getDouble() {
        return valueSubscriber.get();
    }

    public double getDouble(double defaultValue) {
        return valueSubscriber.get(defaultValue);
    }
}