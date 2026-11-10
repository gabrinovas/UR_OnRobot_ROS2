#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "ur_onrobot_control/ur_onrobot_safety_watchdog.hpp"

class SafetyWatchdogUnitTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void SetUp() override {
        rclcpp::NodeOptions options;
        options.append_parameter_override("onrobot_type", "2fg7");
        options.append_parameter_override("auto_recover", false);
        node = std::make_shared<UrOnrobotSafetyWatchdog>(options);
    }

    std::shared_ptr<UrOnrobotSafetyWatchdog> node;
};

TEST_F(SafetyWatchdogUnitTest, StringConversionHelpers) {
    EXPECT_EQ(UrOnrobotSafetyWatchdog::safetyModeToString(ur_dashboard_msgs::msg::SafetyMode::NORMAL), "NORMAL");
    EXPECT_EQ(UrOnrobotSafetyWatchdog::safetyModeToString(ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP), "PROTECTIVE_STOP");
    EXPECT_EQ(UrOnrobotSafetyWatchdog::safetyModeToString(ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP), "ROBOT_EMERGENCY_STOP");
    EXPECT_EQ(UrOnrobotSafetyWatchdog::safetyModeToString(ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP), "SYSTEM_EMERGENCY_STOP");
    EXPECT_EQ(UrOnrobotSafetyWatchdog::safetyModeToString(99), "UNKNOWN (99)");

    EXPECT_EQ(UrOnrobotSafetyWatchdog::robotModeToString(ur_dashboard_msgs::msg::RobotMode::RUNNING), "RUNNING");
    EXPECT_EQ(UrOnrobotSafetyWatchdog::robotModeToString(ur_dashboard_msgs::msg::RobotMode::POWER_OFF), "POWER_OFF");
    EXPECT_EQ(UrOnrobotSafetyWatchdog::robotModeToString(ur_dashboard_msgs::msg::RobotMode::IDLE), "IDLE");

    EXPECT_EQ(UrOnrobotSafetyWatchdog::watchdogStateToString(WatchdogState::NORMAL), "NORMAL");
    EXPECT_EQ(UrOnrobotSafetyWatchdog::watchdogStateToString(WatchdogState::PROTECTIVE_STOP_ACTIVE), "PROTECTIVE_STOP_ACTIVE");
    EXPECT_EQ(UrOnrobotSafetyWatchdog::watchdogStateToString(WatchdogState::EMERGENCY_STOP_ACTIVE), "EMERGENCY_STOP_ACTIVE");
    EXPECT_EQ(UrOnrobotSafetyWatchdog::watchdogStateToString(WatchdogState::RECOVERING), "RECOVERING");
}

TEST_F(SafetyWatchdogUnitTest, InitialStateNominal) {
    EXPECT_EQ(node->getState(), WatchdogState::NORMAL);
    EXPECT_FALSE(node->isFailSafeHoldEngaged());
    EXPECT_EQ(node->getProtectiveStopCount(), 0u);
    EXPECT_EQ(node->getEstopCount(), 0u);
    EXPECT_EQ(node->getRecoveryCount(), 0u);
}

TEST_F(SafetyWatchdogUnitTest, ProtectiveStopEngagesHoldAndTransitionsState) {
    node->triggerSafetyMode(ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP);

    EXPECT_EQ(node->getState(), WatchdogState::PROTECTIVE_STOP_ACTIVE);
    EXPECT_TRUE(node->isFailSafeHoldEngaged());
    EXPECT_EQ(node->getProtectiveStopCount(), 1u);

    // Intento de rearme bloqueado mientras UR siga en parada
    bool recovery_allowed = node->executeRecovery();
    EXPECT_FALSE(recovery_allowed);
    EXPECT_EQ(node->getState(), WatchdogState::PROTECTIVE_STOP_ACTIVE);
}

TEST_F(SafetyWatchdogUnitTest, EmergencyStopHaltAndTransitionsState) {
    node->triggerSafetyMode(ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP);

    EXPECT_EQ(node->getState(), WatchdogState::EMERGENCY_STOP_ACTIVE);
    EXPECT_TRUE(node->isFailSafeHoldEngaged());
    EXPECT_EQ(node->getEstopCount(), 1u);
}

TEST_F(SafetyWatchdogUnitTest, RecoverySequenceRestoresNominalState) {
    // 1. Simular Protective Stop
    node->triggerSafetyMode(ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP);
    EXPECT_EQ(node->getState(), WatchdogState::PROTECTIVE_STOP_ACTIVE);

    // 2. Operario rearma brazo UR a NORMAL
    node->triggerSafetyMode(ur_dashboard_msgs::msg::SafetyMode::NORMAL);

    // 3. Ejecutar rearme de celda
    bool recovered = node->executeRecovery();
    EXPECT_TRUE(recovered);
    EXPECT_EQ(node->getState(), WatchdogState::NORMAL);
    EXPECT_FALSE(node->isFailSafeHoldEngaged());
    EXPECT_EQ(node->getRecoveryCount(), 1u);
}

TEST_F(SafetyWatchdogUnitTest, DiagnosticsOutputMatchesStatus) {
    diagnostic_updater::DiagnosticStatusWrapper stat;
    node->produceDiagnostics(stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_NE(stat.message.find("Watchdog Operativo"), std::string::npos);

    // Verificar tras inyectar fallo
    node->triggerSafetyMode(ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP);
    diagnostic_updater::DiagnosticStatusWrapper stat_pstop;
    node->produceDiagnostics(stat_pstop);
    EXPECT_EQ(stat_pstop.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

    // Verificar tras inyectar E-Stop
    node->triggerSafetyMode(ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP);
    diagnostic_updater::DiagnosticStatusWrapper stat_estop;
    node->produceDiagnostics(stat_estop);
    EXPECT_EQ(stat_estop.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}
