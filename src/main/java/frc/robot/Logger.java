package frc.robot;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings({ "unused" })
public final class Logger {
        private static final PowerDistribution POWER = new PowerDistribution(
                        RobotConstants.PORTS.CAN.POWER_DISTRIBUTION_HUB, ModuleType.kRev);
        private static final DataLog LOG = DataLogManager.getLog();
        private static final Map<String, Publisher> publishers = new HashMap<>();
        private static final Map<String, Object> entries = new HashMap<>();

        private Logger() {
        }

        // =========================================================
        // INIT
        // =========================================================

        public static void start() {

                DataLogManager.start();

                // automatic NT logging
                DataLogManager.logNetworkTables(true);

                // console output
                DataLogManager.logConsoleOutput(true);

                // DS + joystick logging
                DriverStation.startDataLog(LOG, true);
        }

        // =========================================================
        // METADATA
        // =========================================================

        public static void recordMetadata(String key, String value) {
                StringLogEntry entry = new StringLogEntry(LOG, "/Metadata/" + key);

                entry.append(value);
        }

        // =========================================================
        // RECORD OUTPUT
        // =========================================================

        @SuppressWarnings({ "unchecked", "rawtypes" })
        public static void recordOutput(String key, Object value) {

                if (value == null) {
                        return;
                }

                try {

                        // =====================================================
                        // DOUBLE
                        // =====================================================

                        if (value instanceof Double d) {

                                getDouble(key).append(d);

                                DoublePublisher pub = (DoublePublisher) publishers.computeIfAbsent(
                                                key,
                                                k -> NetworkTableInstance.getDefault()
                                                                .getDoubleTopic("/" + key)
                                                                .publish());

                                pub.set(d);
                        }

                        // =====================================================
                        // BOOLEAN
                        // =====================================================

                        else if (value instanceof Boolean b) {

                                getBoolean(key).append(b);

                                BooleanPublisher pub = (BooleanPublisher) publishers.computeIfAbsent(
                                                key,
                                                k -> NetworkTableInstance.getDefault()
                                                                .getBooleanTopic("/" + key)
                                                                .publish());

                                pub.set(b);
                        }

                        // =====================================================
                        // STRING
                        // =====================================================

                        else if (value instanceof String s) {

                                getString(key).append(s);

                                StringPublisher pub = (StringPublisher) publishers.computeIfAbsent(
                                                key,
                                                k -> NetworkTableInstance.getDefault()
                                                                .getStringTopic("/" + key)
                                                                .publish());

                                pub.set(s);
                        }

                        // =====================================================
                        // INTEGER
                        // =====================================================

                        else if (value instanceof Integer i) {

                                getInteger(key).append(i);

                                IntegerPublisher pub = (IntegerPublisher) publishers.computeIfAbsent(
                                                key,
                                                k -> NetworkTableInstance.getDefault()
                                                                .getIntegerTopic("/" + key)
                                                                .publish());

                                pub.set(i);
                        }

                        // =====================================================
                        // DOUBLE ARRAY
                        // =====================================================

                        else if (value instanceof double[] arr) {

                                getDoubleArray(key).append(arr);

                                DoubleArrayPublisher pub = (DoubleArrayPublisher) publishers.computeIfAbsent(
                                                key,
                                                k -> NetworkTableInstance.getDefault()
                                                                .getDoubleArrayTopic("/" + key)
                                                                .publish());

                                pub.set(arr);
                        }

                        // =====================================================
                        // STRUCT SERIALIZABLE
                        // =====================================================

                        else if (value instanceof StructSerializable) {

                                Struct struct = extractStruct(value.getClass());

                                if (struct != null) {

                                        // LOGGING
                                        StructLogEntry entry = (StructLogEntry) entries.computeIfAbsent(
                                                        key,
                                                        k -> StructLogEntry.create(
                                                                        LOG,
                                                                        "/" + key,
                                                                        struct));

                                        entry.append(value);

                                        // NETWORKTABLES
                                        StructPublisher pub = (StructPublisher) publishers.computeIfAbsent(
                                                        "NT_" + key,
                                                        k -> NetworkTableInstance.getDefault()
                                                                        .getStructTopic(
                                                                                        "/" + key,
                                                                                        struct)
                                                                        .publish());

                                        pub.set(value);
                                }
                        }

                        // =====================================================
                        // FALLBACK
                        // =====================================================

                        else {

                                String s = value.toString();

                                getString(key).append(s);

                                StringPublisher pub = (StringPublisher) publishers.computeIfAbsent(
                                                key,
                                                k -> NetworkTableInstance.getDefault()
                                                                .getStringTopic("/" + key)
                                                                .publish());

                                pub.set(s);
                        }

                } catch (Exception e) {

                        DriverStation.reportError(
                                        "Logger failed for key: " + key,
                                        e.getStackTrace());
                }
        }

        // =========================================================
        // AUTO ROBOT STATS
        // =========================================================

        public static void logRobotStats() {
                recordOutput("Power/BatteryVoltage",
                                RobotController.getBatteryVoltage());

                recordOutput("Power/Brownout",
                                RobotController.isBrownedOut());

                recordOutput("RIO/InputVoltage",
                                RobotController.getInputVoltage());

                recordOutput("RIO/InputCurrent",
                                RobotController.getInputCurrent());

                recordOutput("RIO/CPUTemp",
                                RobotController.getCPUTemp());

                CANStatus can = RobotController.getCANStatus();

                recordOutput("CAN/Utilization",
                                can.percentBusUtilization);

                recordOutput("CAN/TxFullCount",
                                can.txFullCount);

                recordOutput("CAN/ReceiveErrorCount",
                                can.receiveErrorCount);

                recordOutput("DS/Enabled",
                                DriverStation.isEnabled());

                recordOutput("DS/Autonomous",
                                DriverStation.isAutonomous());

                recordOutput("DS/MatchTime",
                                DriverStation.getMatchTime());

                recordOutput("PDH/Temperature", POWER.getTemperature());
                recordOutput("PDH/SwitchableChannel", POWER.getSwitchableChannel());
                recordOutput("PDH/TotalCurrent", POWER.getTotalCurrent());
                recordOutput("PDH/TotalEnergy", POWER.getTotalEnergy());
                recordOutput("PDH/TotalPower", POWER.getTotalPower());
                recordOutput("PDH/Version", POWER.getVersion());
                recordOutput("PDH/Voltage", POWER.getVoltage());
                recordOutput("PDH/Faults", POWER.getFaults());
                recordOutput("PDH/Currents", POWER.getAllCurrents());
        }

        // =========================================================
        // ENTRY HELPERS
        // =========================================================

        private static DoubleLogEntry getDouble(String key) {
                return (DoubleLogEntry) entries.computeIfAbsent(
                                key,
                                k -> new DoubleLogEntry(LOG, "/" + key));
        }

        private static IntegerLogEntry getInteger(String key) {
                return (IntegerLogEntry) entries.computeIfAbsent(
                                key,
                                k -> new IntegerLogEntry(LOG, "/" + key));
        }

        private static BooleanLogEntry getBoolean(String key) {
                return (BooleanLogEntry) entries.computeIfAbsent(
                                key,
                                k -> new BooleanLogEntry(LOG, "/" + key));
        }

        private static StringLogEntry getString(String key) {
                return (StringLogEntry) entries.computeIfAbsent(
                                key,
                                k -> new StringLogEntry(LOG, "/" + key));
        }

        private static DoubleArrayLogEntry getDoubleArray(String key) {
                return (DoubleArrayLogEntry) entries.computeIfAbsent(
                                key,
                                k -> new DoubleArrayLogEntry(LOG, "/" + key));
        }

        private static BooleanArrayLogEntry getBooleanArray(String key) {
                return (BooleanArrayLogEntry) entries.computeIfAbsent(
                                key,
                                k -> new BooleanArrayLogEntry(LOG, "/" + key));
        }

        private static IntegerArrayLogEntry getIntegerArray(String key) {
                return (IntegerArrayLogEntry) entries.computeIfAbsent(
                                key,
                                k -> new IntegerArrayLogEntry(LOG, "/" + key));
        }

        private static StringArrayLogEntry getStringArray(String key) {
                return (StringArrayLogEntry) entries.computeIfAbsent(
                                key,
                                k -> new StringArrayLogEntry(LOG, "/" + key));
        }

        // =========================================================
        // STRUCT EXTRACTION
        // =========================================================

        @SuppressWarnings("rawtypes")
        private static Struct extractStruct(Class<?> clazz) {

                try {

                        Field field = clazz.getField("struct");

                        return (Struct) field.get(null);

                } catch (Exception e) {
                        return null;
                }
        }
}