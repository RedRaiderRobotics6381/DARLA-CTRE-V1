package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CANdleSubSystem;

public class CANdleCmd {
    public static class ConfigBrightness extends InstantCommand {
        public ConfigBrightness(CANdleSubSystem candleSubSystem, double brightnessPercent) {
            super(() -> candleSubSystem.configBrightness(brightnessPercent), candleSubSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    public static class ConfigLosBehavior extends InstantCommand {
        public ConfigLosBehavior(CANdleSubSystem candleSubSystem, boolean disableWhenLos) {
            super(() -> candleSubSystem.configLos(disableWhenLos), candleSubSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    public static class ConfigStatusLedBehavior extends InstantCommand {
        public ConfigStatusLedBehavior(CANdleSubSystem candleSubSystem, boolean disableWhile) {
            super(() -> candleSubSystem.configStatusLedBehavior(disableWhile), candleSubSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    public static class PrintVBat extends InstantCommand {
        public PrintVBat(CANdleSubSystem candleSubSystem) {
            super(() -> System.out.println("Vbat is " + candleSubSystem.getVbat() + "V"), candleSubSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    public static class Print5V extends InstantCommand {
        public Print5V(CANdleSubSystem candleSubSystem) {
            super(() -> System.out.println("5V is " + candleSubSystem.get5V() + "V"), candleSubSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    public static class PrintCurrent extends InstantCommand {
        public PrintCurrent(CANdleSubSystem candleSubSystem) {
            super(() -> System.out.println("Current is " + candleSubSystem.getCurrent() + "A"), candleSubSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    public static class PrintTemperature extends InstantCommand {
        public PrintTemperature(CANdleSubSystem candleSubSystem) {
            super(() -> System.out.println("Temperature is " + candleSubSystem.getTemperature() + "C"), candleSubSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
}
