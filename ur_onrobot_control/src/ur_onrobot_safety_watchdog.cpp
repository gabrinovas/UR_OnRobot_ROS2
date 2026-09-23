#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <ur_dashboard_msgs/msg/safety_mode.hpp>
#include <ur_dashboard_msgs/msg/robot_mode.hpp>

#include <mutex>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

enum class WatchdogState
{
  NORMAL = 0,
  PROTECTIVE_STOP_ACTIVE = 1,
  EMERGENCY_STOP_ACTIVE = 2,
  RECOVERING = 3
};

class UrOnrobotSafetyWatchdog : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit UrOnrobotSafetyWatchdog(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ur_onrobot_safety_watchdog", options),
    updater_(this),
    state_(WatchdogState::NORMAL),
    current_safety_mode_(ur_dashboard_msgs::msg::SafetyMode::NORMAL),
    current_robot_mode_(ur_dashboard_msgs::msg::RobotMode::RUNNING),
    fail_safe_hold_engaged_(false),
    last_finger_width_(0.035),
    last_vacuum_a_(0.0),
    last_vacuum_b_(0.0),
    has_telemetry_(false),
    protective_stop_count_(0),
    estop_count_(0),
    recovery_count_(0)
  {
    // 1. Declarar parámetros configurables
    auto declare_param_if_not = [this](const std::string & name, const auto & default_val) {
      if (!this->has_parameter(name)) {
        this->declare_parameter(name, default_val);
      }
    };
    declare_param_if_not("safety_mode_topic", std::string("/io_and_status_controller/safety_mode"));
    declare_param_if_not("robot_mode_topic", std::string("/io_and_status_controller/robot_mode"));
    declare_param_if_not("joint_states_topic", std::string("/merged_joint_states"));
    declare_param_if_not("onrobot_type", std::string("2fg7"));
    declare_param_if_not("gripper_action_topic", std::string(""));
    declare_param_if_not("auto_recover", false);
    declare_param_if_not("fail_safe_effort", 140.0);
    declare_param_if_not("diagnostic_period", 1.0);

    safety_mode_topic_ = get_parameter("safety_mode_topic").as_string();
    robot_mode_topic_ = get_parameter("robot_mode_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    onrobot_type_ = get_parameter("onrobot_type").as_string();
    auto_recover_ = get_parameter("auto_recover").as_bool();
    fail_safe_effort_ = get_parameter("fail_safe_effort").as_double();
    double diag_period = get_parameter("diagnostic_period").as_double();

    std::string explicit_action_topic = get_parameter("gripper_action_topic").as_string();
    if (!explicit_action_topic.empty()) {
      gripper_action_topic_ = explicit_action_topic;
    } else {
      if (onrobot_type_ == "vgc10") {
        gripper_action_topic_ = "/onrobot/gripper_channel_a_controller/gripper_cmd";
      } else {
        gripper_action_topic_ = "/onrobot/gripper_action_controller/gripper_cmd";
      }
    }

    RCLCPP_INFO(get_logger(), "=== UR + OnRobot Safety Watchdog Inicializado ===");
    RCLCPP_INFO(get_logger(), "Efector: %s | Safety Topic: %s | Robot Mode Topic: %s",
      onrobot_type_.c_str(), safety_mode_topic_.c_str(), robot_mode_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Gripper Action: %s | Auto-recover: %s",
      gripper_action_topic_.c_str(), auto_recover_ ? "true" : "false");

    // 2. Configurar Diagnostic Updater
    updater_.setHardwareID("UR Safety Watchdog (" + onrobot_type_ + ")");
    updater_.add("Safety & Interlock Monitor", this, &UrOnrobotSafetyWatchdog::produceDiagnostics);

    // 3. Suscriptores de estado del robot Universal Robots
    safety_mode_sub_ = create_subscription<ur_dashboard_msgs::msg::SafetyMode>(
      safety_mode_topic_, rclcpp::QoS(10),
      std::bind(&UrOnrobotSafetyWatchdog::safetyModeCallback, this, std::placeholders::_1));

    robot_mode_sub_ = create_subscription<ur_dashboard_msgs::msg::RobotMode>(
      robot_mode_topic_, rclcpp::QoS(10),
      std::bind(&UrOnrobotSafetyWatchdog::robotModeCallback, this, std::placeholders::_1));

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10),
      std::bind(&UrOnrobotSafetyWatchdog::jointStateCallback, this, std::placeholders::_1));

    // 4. Clientes de acción y servicios
    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_topic_);
    reset_power_client_ = create_client<std_srvs::srv::Trigger>("/onrobot/reset_power");

    // 5. Servicios de rearme y simulación de fallos (para testing interactivo y en Docker)
    recover_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/recover_system",
      std::bind(&UrOnrobotSafetyWatchdog::handleRecoverSystem, this, std::placeholders::_1, std::placeholders::_2));

    inject_pstop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/inject_protective_stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        ur_dashboard_msgs::msg::SafetyMode mock_msg;
        mock_msg.mode = ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP;
        safetyModeCallback(std::make_shared<ur_dashboard_msgs::msg::SafetyMode>(mock_msg));
        res->success = true;
        res->message = "Inyección de Protective Stop aplicada correctamente.";
      });

    inject_estop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/inject_emergency_stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        ur_dashboard_msgs::msg::SafetyMode mock_msg;
        mock_msg.mode = ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP;
        safetyModeCallback(std::make_shared<ur_dashboard_msgs::msg::SafetyMode>(mock_msg));
        res->success = true;
        res->message = "Inyección de Emergency Stop aplicada correctamente.";
      });

    inject_clear_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/inject_clear_stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        ur_dashboard_msgs::msg::SafetyMode mock_msg;
        mock_msg.mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
        safetyModeCallback(std::make_shared<ur_dashboard_msgs::msg::SafetyMode>(mock_msg));
        res->success = true;
        res->message = "Inyección de estado NORMAL aplicada correctamente.";
      });

    // 6. Timer para diagnóstico periódico
    diag_timer_ = create_wall_timer(
      std::chrono::duration<double>(diag_period),
      [this]() { updater_.force_update(); });

    last_telemetry_time_ = now();
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto & name = msg->name[i];
      if (name.find("finger_width") != std::string::npos && i < msg->position.size()) {
        last_finger_width_ = msg->position[i];
        has_telemetry_ = true;
      }
      if (name.find("vacuum_channel_a") != std::string::npos && i < msg->position.size()) {
        last_vacuum_a_ = msg->position[i];
        has_telemetry_ = true;
      }
      if (name.find("vacuum_channel_b") != std::string::npos && i < msg->position.size()) {
        last_vacuum_b_ = msg->position[i];
        has_telemetry_ = true;
      }
    }
    if (has_telemetry_) {
      last_telemetry_time_ = now();
    }
  }

  void safetyModeCallback(const ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    uint8_t prev_mode = current_safety_mode_;
    current_safety_mode_ = msg->mode;

    if (current_safety_mode_ == prev_mode) {
      return;
    }

    RCLCPP_WARN(get_logger(), "Transición de modo de seguridad UR: %s -> %s",
      safetyModeToString(prev_mode).c_str(), safetyModeToString(current_safety_mode_).c_str());

    // 1. Detección de Protective Stop (colisión / esfuerzo excesivo)
    if (current_safety_mode_ == ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP) {
      state_ = WatchdogState::PROTECTIVE_STOP_ACTIVE;
      protective_stop_count_++;
      engageProtectiveStopHold();
    }
    // 2. Detección de Emergency Stop
    else if (current_safety_mode_ == ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP ||
             current_safety_mode_ == ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP) {
      state_ = WatchdogState::EMERGENCY_STOP_ACTIVE;
      estop_count_++;
      engageEmergencyStopHalt();
    }
    // 3. Retorno a estado NORMAL
    else if (current_safety_mode_ == ur_dashboard_msgs::msg::SafetyMode::NORMAL) {
      if (state_ == WatchdogState::PROTECTIVE_STOP_ACTIVE || state_ == WatchdogState::EMERGENCY_STOP_ACTIVE) {
        RCLCPP_INFO(get_logger(), "Modo de seguridad NORMAL restablecido en el robot UR.");
        if (auto_recover_) {
          RCLCPP_INFO(get_logger(), "Auto-recover habilitado. Ejecutando re-sincronización y rearme...");
          performRecovery();
        } else {
          RCLCPP_INFO(get_logger(), "Auto-recover deshabilitado. Enclavamiento mantenido. Llame a ~/recover_system para rearmar.");
        }
      }
    }
  }

  void robotModeCallback(const ur_dashboard_msgs::msg::RobotMode::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_robot_mode_ = msg->mode;
  }

  void engageProtectiveStopHold()
  {
    fail_safe_hold_engaged_ = true;
    last_event_time_ = now();

    RCLCPP_ERROR(get_logger(), "¡PROTECTIVE STOP DETECTADO! Activando enclavamiento de seguridad para '%s'.",
      onrobot_type_.c_str());

    if (!gripper_action_client_->wait_for_action_server(500ms)) {
      RCLCPP_ERROR(get_logger(), "Servidor de acción de pinza '%s' no disponible para emitir enclavamiento.",
        gripper_action_topic_.c_str());
      return;
    }

    auto goal = GripperCommand::Goal();

    if (onrobot_type_ == "vgc10") {
      // VGC10: Enclavar vacío en máximo rendimiento (100%) para evitar desprendimiento
      goal.command.position = 1.0;
      goal.command.max_effort = 80.0;
      held_value_desc_ = "Vacío forzado 100% (Fail-Safe Hold)";
      RCLCPP_INFO(get_logger(), "Enviando comando de retención de vacío: Canal A = 1.0 (80.0 kPa)");
    } else {
      // 2FG7 / 3FG15: Congelar posición actual con máxima fuerza para activar retención mecánica
      double hold_pos;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        hold_pos = last_finger_width_;
      }
      goal.command.position = hold_pos;
      goal.command.max_effort = fail_safe_effort_;
      char buf[64];
      snprintf(buf, sizeof(buf), "Congelado en %.1f mm (Fuerza=%.0f N)", hold_pos * 1000.0, fail_safe_effort_);
      held_value_desc_ = buf;
      RCLCPP_INFO(get_logger(), "Enviando comando de congelación de posición: %.4f m (%.1f N)", hold_pos, fail_safe_effort_);
    }

    gripper_action_client_->async_send_goal(goal);
  }

  void engageEmergencyStopHalt()
  {
    fail_safe_hold_engaged_ = true;
    last_event_time_ = now();
    held_value_desc_ = "E-STOP: Tráfico Modbus suspendido (Circuito de Seguridad Abierto)";

    RCLCPP_ERROR(get_logger(), "¡EMERGENCY STOP (E-STOP) DETECTADO!");
    RCLCPP_ERROR(get_logger(), "Inhibiendo envíos de órdenes al efector para evitar saturación de búferes en Tool I/O.");
  }

  bool performRecovery()
  {
    state_ = WatchdogState::RECOVERING;
    RCLCPP_INFO(get_logger(), "Ejecutando secuencia de recuperación y rearme...");

    // 1. Invocar reinicio de energía del efector
    if (reset_power_client_->wait_for_service(1s)) {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = reset_power_client_->async_send_request(req);
      RCLCPP_INFO(get_logger(), "Petición de reseteo de energía enviada a /onrobot/reset_power.");
    } else {
      RCLCPP_WARN(get_logger(), "Servicio /onrobot/reset_power no disponible. Omitiendo reset eléctrico.");
    }

    // 2. Liberar enclavamiento y retornar a estado normal
    state_ = WatchdogState::NORMAL;
    fail_safe_hold_engaged_ = false;
    recovery_count_++;
    held_value_desc_ = "Operación Nominal";
    RCLCPP_INFO(get_logger(), "¡Secuencia de recuperación completada con éxito! Sistema en estado NORMAL.");
    return true;
  }

  void handleRecoverSystem(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    RCLCPP_INFO(get_logger(), "Solicitud de rearme de sistema recibida por servicio ~/recover_system.");

    if (current_safety_mode_ != ur_dashboard_msgs::msg::SafetyMode::NORMAL) {
      res->success = false;
      res->message = "No se puede rearmar: El robot UR aún no está en modo NORMAL (modo actual: " +
                     safetyModeToString(current_safety_mode_) + ")";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    bool success = performRecovery();
    res->success = success;
    res->message = success ? "Sistema UR + OnRobot rearmado y operativo" : "Fallo durante la recuperación";
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock_state(state_mutex_);
    std::lock_guard<std::mutex> lock_data(data_mutex_);

    auto now_time = now();
    double telemetry_age = (has_telemetry_) ? (now_time - last_telemetry_time_).seconds() : 999.0;

    stat.add("Gripper Model", onrobot_type_);
    stat.add("Watchdog State", watchdogStateToString(state_));
    stat.add("UR Safety Mode", safetyModeToString(current_safety_mode_));
    stat.add("UR Robot Mode", robotModeToString(current_robot_mode_));
    stat.add("Fail-Safe Hold Engaged", fail_safe_hold_engaged_ ? "YES" : "NO");
    stat.add("Enclavamiento Status", held_value_desc_);
    stat.addf("Telemetry Age (s)", "%.3f", telemetry_age);
    stat.add("Protective Stop Events", protective_stop_count_);
    stat.add("Emergency Stop Events", estop_count_);
    stat.add("Successful Recoveries", recovery_count_);

    if (onrobot_type_ == "vgc10") {
      stat.addf("Vacuum Channel A", "%.1f %%", last_vacuum_a_ * 100.0);
      stat.addf("Vacuum Channel B", "%.1f %%", last_vacuum_b_ * 100.0);
    } else {
      stat.addf("Finger Width (mm)", "%.2f", last_finger_width_ * 1000.0);
    }

    if (state_ == WatchdogState::EMERGENCY_STOP_ACTIVE) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "EMERGENCY STOP ACTIVO: Circuito de seguridad abierto. Tráfico de efector inhibido.");
    } else if (state_ == WatchdogState::PROTECTIVE_STOP_ACTIVE) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "PROTECTIVE STOP ACTIVO: Enclavamiento de retención (Fail-Safe Hold) activado.");
    } else if (telemetry_age > 3.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "ALERTA: Telemetría de articulaciones demorada (> 3.0s)");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Watchdog Operativo: Robot y Efector en condiciones normales de seguridad.");
    }
  }

  static std::string safetyModeToString(uint8_t mode)
  {
    switch (mode) {
      case ur_dashboard_msgs::msg::SafetyMode::NORMAL: return "NORMAL";
      case ur_dashboard_msgs::msg::SafetyMode::REDUCED: return "REDUCED";
      case ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP: return "PROTECTIVE_STOP";
      case ur_dashboard_msgs::msg::SafetyMode::RECOVERY: return "RECOVERY";
      case ur_dashboard_msgs::msg::SafetyMode::SAFEGUARD_STOP: return "SAFEGUARD_STOP";
      case ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP: return "SYSTEM_EMERGENCY_STOP";
      case ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP: return "ROBOT_EMERGENCY_STOP";
      case ur_dashboard_msgs::msg::SafetyMode::VIOLATION: return "VIOLATION";
      case ur_dashboard_msgs::msg::SafetyMode::FAULT: return "FAULT";
      case ur_dashboard_msgs::msg::SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP: return "AUTO_SAFEGUARD_STOP";
      default: return "UNKNOWN (" + std::to_string(mode) + ")";
    }
  }

  static std::string robotModeToString(int8_t mode)
  {
    switch (mode) {
      case ur_dashboard_msgs::msg::RobotMode::NO_CONTROLLER: return "NO_CONTROLLER";
      case ur_dashboard_msgs::msg::RobotMode::DISCONNECTED: return "DISCONNECTED";
      case ur_dashboard_msgs::msg::RobotMode::CONFIRM_SAFETY: return "CONFIRM_SAFETY";
      case ur_dashboard_msgs::msg::RobotMode::BOOTING: return "BOOTING";
      case ur_dashboard_msgs::msg::RobotMode::POWER_OFF: return "POWER_OFF";
      case ur_dashboard_msgs::msg::RobotMode::POWER_ON: return "POWER_ON";
      case ur_dashboard_msgs::msg::RobotMode::IDLE: return "IDLE";
      case ur_dashboard_msgs::msg::RobotMode::BACKDRIVE: return "BACKDRIVE";
      case ur_dashboard_msgs::msg::RobotMode::RUNNING: return "RUNNING";
      case ur_dashboard_msgs::msg::RobotMode::UPDATING_FIRMWARE: return "UPDATING_FIRMWARE";
      default: return "UNKNOWN (" + std::to_string(mode) + ")";
    }
  }

  static std::string watchdogStateToString(WatchdogState state)
  {
    switch (state) {
      case WatchdogState::NORMAL: return "NORMAL";
      case WatchdogState::PROTECTIVE_STOP_ACTIVE: return "PROTECTIVE_STOP_ACTIVE";
      case WatchdogState::EMERGENCY_STOP_ACTIVE: return "EMERGENCY_STOP_ACTIVE";
      case WatchdogState::RECOVERING: return "RECOVERING";
      default: return "UNKNOWN";
    }
  }

  std::string safety_mode_topic_;
  std::string robot_mode_topic_;
  std::string joint_states_topic_;
  std::string onrobot_type_;
  std::string gripper_action_topic_;
  bool auto_recover_;
  double fail_safe_effort_;

  diagnostic_updater::Updater updater_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  rclcpp::Subscription<ur_dashboard_msgs::msg::SafetyMode>::SharedPtr safety_mode_sub_;
  rclcpp::Subscription<ur_dashboard_msgs::msg::RobotMode>::SharedPtr robot_mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_power_client_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr inject_pstop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr inject_estop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr inject_clear_srv_;

  std::mutex state_mutex_;
  WatchdogState state_;
  uint8_t current_safety_mode_;
  int8_t current_robot_mode_;
  bool fail_safe_hold_engaged_;
  std::string held_value_desc_{"Operación Nominal"};
  rclcpp::Time last_event_time_;

  std::mutex data_mutex_;
  double last_finger_width_;
  double last_vacuum_a_;
  double last_vacuum_b_;
  bool has_telemetry_;
  rclcpp::Time last_telemetry_time_;

  size_t protective_stop_count_;
  size_t estop_count_;
  size_t recovery_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<UrOnrobotSafetyWatchdog>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
