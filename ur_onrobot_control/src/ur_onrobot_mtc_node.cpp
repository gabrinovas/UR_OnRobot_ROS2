#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/container.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <thread>
#include <chrono>

using namespace moveit::task_constructor;

class UrOnrobotMtcNode : public rclcpp::Node
{
public:
  UrOnrobotMtcNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("ur_onrobot_mtc_node", options)
  {
    // 1. Declarar parámetros configurables si no han sido pre-declarados
    if (!has_parameter("arm_group_name")) declare_parameter<std::string>("arm_group_name", "ur_manipulator");
    if (!has_parameter("gripper_group_name")) declare_parameter<std::string>("gripper_group_name", "gripper");
    if (!has_parameter("gripper_open_state")) declare_parameter<std::string>("gripper_open_state", "open");
    if (!has_parameter("gripper_close_state")) declare_parameter<std::string>("gripper_close_state", "closed");
    if (!has_parameter("hand_frame")) declare_parameter<std::string>("hand_frame", "gripper_tcp");
    if (!has_parameter("world_frame")) declare_parameter<std::string>("world_frame", "world");
    if (!has_parameter("object_name")) declare_parameter<std::string>("object_name", "workpiece_box");
    if (!has_parameter("object_dimensions")) declare_parameter<std::vector<double>>("object_dimensions", {0.04, 0.04, 0.07});
    if (!has_parameter("pick_pose")) declare_parameter<std::vector<double>>("pick_pose", {0.10, -0.45, 0.07, 0.0, 3.14159, 0.0});
    if (!has_parameter("place_pose")) declare_parameter<std::vector<double>>("place_pose", {0.30, -0.45, 0.07, 0.0, 3.14159, 0.0});
    if (!has_parameter("approach_distance")) declare_parameter<double>("approach_distance", 0.08);
    if (!has_parameter("lift_distance")) declare_parameter<double>("lift_distance", 0.12);
    if (!has_parameter("retreat_distance")) declare_parameter<double>("retreat_distance", 0.10);
    if (!has_parameter("execute_on_startup")) declare_parameter<bool>("execute_on_startup", false);
    if (!has_parameter("spawn_table_if_missing")) declare_parameter<bool>("spawn_table_if_missing", true);

    // Obtener valores de parámetros
    arm_group_name_ = get_parameter("arm_group_name").as_string();
    gripper_group_name_ = get_parameter("gripper_group_name").as_string();
    gripper_open_state_ = get_parameter("gripper_open_state").as_string();
    gripper_close_state_ = get_parameter("gripper_close_state").as_string();
    hand_frame_ = get_parameter("hand_frame").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    object_name_ = get_parameter("object_name").as_string();
    object_dimensions_ = get_parameter("object_dimensions").as_double_array();
    pick_pose_ = get_parameter("pick_pose").as_double_array();
    place_pose_ = get_parameter("place_pose").as_double_array();
    approach_distance_ = get_parameter("approach_distance").as_double();
    lift_distance_ = get_parameter("lift_distance").as_double();
    retreat_distance_ = get_parameter("retreat_distance").as_double();
    execute_on_startup_ = get_parameter("execute_on_startup").as_bool();
    spawn_table_ = get_parameter("spawn_table_if_missing").as_bool();

    RCLCPP_INFO(get_logger(), "=== Nodo MTC Pick & Place UR + OnRobot Inicializado ===");
    RCLCPP_INFO(get_logger(), "Arm Group: %s | Gripper Group: %s (%s -> %s)",
      arm_group_name_.c_str(), gripper_group_name_.c_str(),
      gripper_open_state_.c_str(), gripper_close_state_.c_str());
    RCLCPP_INFO(get_logger(), "TCP Frame: %s | World Frame: %s | Objeto: %s",
      hand_frame_.c_str(), world_frame_.c_str(), object_name_.c_str());

    // 2. Servicio para disparo bajo demanda
    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "~/execute_task",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        (void)req;
        RCLCPP_INFO(get_logger(), "Petición de ejecución de Pick & Place recibida por servicio.");
        bool success = runPickAndPlace(true);
        res->success = success;
        res->message = success ? "Pick & Place ejecutado con éxito" : "Fallo en la planificación/ejecución MTC";
      });

    // 3. Hilo de ejecución automática al inicio si está habilitado
    if (execute_on_startup_) {
      startup_thread_ = std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(get_logger(), "Iniciando ejecución automática de Pick & Place MTC...");
        runPickAndPlace(true);
      });
    }
  }

  ~UrOnrobotMtcNode() override
  {
    if (startup_thread_.joinable()) {
      startup_thread_.join();
    }
  }

  void setupPlanningScene()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    // Limpiar objeto previo si existía
    psi.removeCollisionObjects({object_name_});
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // Objeto objetivo (workpiece)
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = world_frame_;
    object.id = object_name_;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = object_dimensions_[0];
    primitive.dimensions[primitive.BOX_Y] = object_dimensions_[1];
    primitive.dimensions[primitive.BOX_Z] = object_dimensions_[2];

    geometry_msgs::msg::Pose pose;
    pose.position.x = pick_pose_[0];
    pose.position.y = pick_pose_[1];
    pose.position.z = pick_pose_[2] + (object_dimensions_[2] / 2.0); // apoyado sobre superficie

    tf2::Quaternion q;
    q.setRPY(pick_pose_[3], pick_pose_[4], pick_pose_[5]);
    pose.orientation = tf2::toMsg(q);

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;
    collision_objects.push_back(object);

    // Opcional: Superficie de soporte base si no existe entorno de celda
    if (spawn_table_) {
      moveit_msgs::msg::CollisionObject table;
      table.header.frame_id = world_frame_;
      table.id = "support_table";

      shape_msgs::msg::SolidPrimitive table_prim;
      table_prim.type = table_prim.BOX;
      table_prim.dimensions = {0.6, 0.4, 0.05};

      geometry_msgs::msg::Pose table_pose;
      table_pose.position.x = 0.20;
      table_pose.position.y = -0.45;
      table_pose.position.z = pick_pose_[2] - 0.025; // inmediatamente bajo la pieza
      table_pose.orientation.w = 1.0;

      table.primitives.push_back(table_prim);
      table.primitive_poses.push_back(table_pose);
      table.operation = table.ADD;
      collision_objects.push_back(table);
    }

    psi.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(get_logger(), "Objeto de colisión '%s' insertado en la escena en [%.2f, %.2f, %.2f]",
      object_name_.c_str(), pose.position.x, pose.position.y, pose.position.z);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  Task createTask()
  {
    Task task;
    task.stages()->setName("UR + OnRobot Pick & Place Task");
    task.loadRobotModel(shared_from_this());

    // Configuración de Solvers con parametrización temporal explícita
    auto time_param = std::make_shared<trajectory_processing::IterativeParabolicTimeParameterization>();

    auto sampling_planner = std::make_shared<solvers::JointInterpolationPlanner>();
    sampling_planner->setTimeParameterization(time_param);
    
    auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(shared_from_this(), "ompl");
    pipeline_planner->setProperty("max_velocity_scaling_factor", 0.3);
    pipeline_planner->setProperty("max_acceleration_scaling_factor", 0.3);
    pipeline_planner->setProperty("num_planning_attempts", 5u);
    pipeline_planner->setTimeParameterization(time_param);

    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.2);
    cartesian_planner->setMaxAccelerationScalingFactor(0.2);
    cartesian_planner->setStepSize(0.002);
    cartesian_planner->setMinFraction(0.9);
    cartesian_planner->setTimeParameterization(time_param);

    // Asignar propiedades globales a la tarea
    task.setProperty("group", arm_group_name_);
    task.setProperty("ik_frame", hand_frame_);

    Stage* allow_collision_ptr = nullptr;
    Stage* pick_stage_ptr = nullptr;

    // ==========================================
    // ETAPA 1: Captura de Estado Actual
    // ==========================================
    {
      auto current_state = std::make_unique<stages::CurrentState>("current_state");
      task.add(std::move(current_state));
    }

    // ==========================================
    // ETAPA 2: Abrir Pinza / Desactivar Vacío
    // ==========================================
    {
      auto open_hand = std::make_unique<stages::MoveTo>("open_gripper", sampling_planner);
      open_hand->setGroup(gripper_group_name_);
      open_hand->setGoal(gripper_open_state_);
      task.add(std::move(open_hand));
    }

    // ==========================================
    // ETAPA 3: Permitir Colisión Pinza - Objeto
    // ==========================================
    {
      auto allow_collision = std::make_unique<stages::ModifyPlanningScene>("allow_collision_grasp");
      allow_collision->allowCollisions(object_name_, true);
      allow_collision_ptr = allow_collision.get();
      task.add(std::move(allow_collision));
    }

    // ==========================================
    // ETAPA 4: Conectar hacia Aproximación de Agarre
    // ==========================================
    {
      auto connect = std::make_unique<stages::Connect>(
        "move_to_pick",
        stages::Connect::GroupPlannerVector{{arm_group_name_, pipeline_planner}});
      connect->setTimeout(15.0);
      connect->properties().configureInitFrom(Stage::PARENT);
      task.add(std::move(connect));
    }

    // ==========================================
    // ETAPAS 5 - 9: Contenedor Serial de Recogida (Pick)
    // ==========================================
    {
      auto pick_container = std::make_unique<SerialContainer>("pick_object");

      // 5. Aproximación Cartesiana lineal hacia el objeto (eje -Z)
      {
        auto approach = std::make_unique<stages::MoveRelative>("approach_object", cartesian_planner);
        approach->setGroup(arm_group_name_);
        approach->properties().set("marker_ns", "approach");
        approach->setIKFrame(hand_frame_);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = world_frame_;
        vec.vector.z = -1.0;
        approach->setDirection(vec);
        approach->setMinMaxDistance(0.01, approach_distance_);
        pick_container->add(std::move(approach));
      }

      // 6. Generar Postura de Agarre e IK
      {
        auto grasp_alternatives = std::make_unique<Alternatives>("generate_grasp_pose");
        // Muestrear orientaciones de aproximación en yaw (0, 90, 180, 270 grados)
        for (double yaw_offset : {0.0, 1.570796, 3.141592, -1.570796}) {
          auto grasp_pose = std::make_unique<stages::GeneratePose>("grasp_candidate");
          geometry_msgs::msg::PoseStamped grasp_msg;
          grasp_msg.header.frame_id = world_frame_;
          grasp_msg.pose.position.x = pick_pose_[0];
          grasp_msg.pose.position.y = pick_pose_[1];
          grasp_msg.pose.position.z = pick_pose_[2] + object_dimensions_[2] + 0.01; // punto de contacto superior

          tf2::Quaternion q;
          q.setRPY(pick_pose_[3], pick_pose_[4], pick_pose_[5] + yaw_offset);
          grasp_msg.pose.orientation = tf2::toMsg(q);

          grasp_pose->setPose(grasp_msg);
          grasp_pose->setMonitoredStage(allow_collision_ptr);
          grasp_alternatives->add(std::move(grasp_pose));
        }

        auto ik = std::make_unique<stages::ComputeIK>("compute_grasp_ik", std::move(grasp_alternatives));
        ik->setGroup(arm_group_name_);
        ik->setIKFrame(hand_frame_);
        ik->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
        ik->setMaxIKSolutions(8);
        ik->setMinSolutionDistance(0.05);
        pick_container->add(std::move(ik));
      }

      // 7. Cerrar pinza / Activar vacío
      {
        auto close_hand = std::make_unique<stages::MoveTo>("close_gripper", sampling_planner);
        close_hand->setGroup(gripper_group_name_);
        close_hand->setGoal(gripper_close_state_);
        pick_container->add(std::move(close_hand));
      }

      // 8. Adjuntar objeto al TCP cinemático
      {
        auto attach = std::make_unique<stages::ModifyPlanningScene>("attach_object");
        attach->attachObject(object_name_, hand_frame_);
        pick_container->add(std::move(attach));
      }

      // 9. Elevación Cartesiana lineal (+Z)
      {
        auto lift = std::make_unique<stages::MoveRelative>("lift_object", cartesian_planner);
        lift->setGroup(arm_group_name_);
        lift->properties().set("marker_ns", "lift");
        lift->setIKFrame(hand_frame_);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = world_frame_;
        vec.vector.z = 1.0;
        lift->setDirection(vec);
        lift->setMinMaxDistance(0.02, lift_distance_);
        pick_container->add(std::move(lift));
      }

      pick_stage_ptr = pick_container.get();
      task.add(std::move(pick_container));
    }

    // ==========================================
    // ETAPA 10: Tránsito libre de colisiones hacia Zona de Descarga (Place)
    // ==========================================
    {
      auto transit = std::make_unique<stages::Connect>(
        "transit_to_place",
        stages::Connect::GroupPlannerVector{{arm_group_name_, pipeline_planner}});
      transit->setTimeout(15.0);
      transit->properties().configureInitFrom(Stage::PARENT);
      task.add(std::move(transit));
    }

    // ==========================================
    // ETAPAS 11 - 15: Contenedor Serial de Descarga (Place)
    // ==========================================
    {
      auto place_container = std::make_unique<SerialContainer>("place_object");

      // 11. Descenso Cartesiano lineal hacia la zona de descarga (-Z)
      {
        auto lower = std::make_unique<stages::MoveRelative>("lower_object", cartesian_planner);
        lower->setGroup(arm_group_name_);
        lower->properties().set("marker_ns", "lower");
        lower->setIKFrame(hand_frame_);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = world_frame_;
        vec.vector.z = -1.0;
        lower->setDirection(vec);
        lower->setMinMaxDistance(0.01, 0.08);
        place_container->add(std::move(lower));
      }

      // 12. Generar Postura de Descarga e IK
      {
        auto place_alternatives = std::make_unique<Alternatives>("generate_place_pose");
        for (double yaw_offset : {0.0, 1.570796, 3.141592, -1.570796}) {
          auto place_pose = std::make_unique<stages::GeneratePose>("place_candidate");
          geometry_msgs::msg::PoseStamped place_msg;
          place_msg.header.frame_id = world_frame_;
          place_msg.pose.position.x = place_pose_[0];
          place_msg.pose.position.y = place_pose_[1];
          place_msg.pose.position.z = place_pose_[2] + object_dimensions_[2] + 0.01;

          tf2::Quaternion q_place;
          q_place.setRPY(place_pose_[3], place_pose_[4], place_pose_[5] + yaw_offset);
          place_msg.pose.orientation = tf2::toMsg(q_place);

          place_pose->setPose(place_msg);
          place_pose->setMonitoredStage(pick_stage_ptr);
          place_alternatives->add(std::move(place_pose));
        }

        auto place_ik = std::make_unique<stages::ComputeIK>("compute_place_ik", std::move(place_alternatives));
        place_ik->setGroup(arm_group_name_);
        place_ik->setIKFrame(hand_frame_);
        place_ik->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
        place_ik->setMaxIKSolutions(8);
        place_ik->setMinSolutionDistance(0.05);
        place_container->add(std::move(place_ik));
      }

      // 13. Abrir pinza / Desactivar vacío para liberar
      {
        auto release_hand = std::make_unique<stages::MoveTo>("open_gripper_release", sampling_planner);
        release_hand->setGroup(gripper_group_name_);
        release_hand->setGoal(gripper_open_state_);
        place_container->add(std::move(release_hand));
      }

      // 14. Desvincular objeto de la pinza
      {
        auto detach = std::make_unique<stages::ModifyPlanningScene>("detach_object");
        detach->detachObject(object_name_, hand_frame_);
        place_container->add(std::move(detach));
      }

      // 15. Retirada Cartesiana vertical (+Z)
      {
        auto retreat = std::make_unique<stages::MoveRelative>("retreat_from_object", cartesian_planner);
        retreat->setGroup(arm_group_name_);
        retreat->properties().set("marker_ns", "retreat");
        retreat->setIKFrame(hand_frame_);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = world_frame_;
        vec.vector.z = 1.0;
        retreat->setDirection(vec);
        retreat->setMinMaxDistance(0.02, retreat_distance_);
        place_container->add(std::move(retreat));
      }

      // 16. Restaurar matriz de colisiones estricta (una vez alejada la pinza)
      {
        auto forbid_collision = std::make_unique<stages::ModifyPlanningScene>("forbid_collision_place");
        forbid_collision->allowCollisions(object_name_, false);
        place_container->add(std::move(forbid_collision));
      }

      task.add(std::move(place_container));
    }

    // ==========================================
    // ETAPA 17: Retorno a Postura Home
    // ==========================================
    {
      auto home = std::make_unique<stages::MoveTo>("return_home", pipeline_planner);
      home->setGroup(arm_group_name_);
      home->setGoal("home");
      home->restrictDirection(stages::MoveTo::FORWARD);
      task.add(std::move(home));
    }

    return task;
  }

  bool runPickAndPlace(bool execute_trajectory = true)
  {
    RCLCPP_INFO(get_logger(), "Configurando escena y preparando MTC Task...");
    setupPlanningScene();

    Task task;
    try {
      task = createTask();
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Error al construir pipeline MTC: %s", ex.what());
      return false;
    }

    try {
      task.init();
    } catch (const InitStageException& ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "Fallo en la inicialización de etapas MTC: " << ex);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Planificando soluciones cinemáticas con MoveIt Task Constructor...");
    if (!task.plan(10)) {
      RCLCPP_ERROR(get_logger(), "No se encontraron soluciones válidas para el pipeline de Pick & Place.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "¡Planificación completada con éxito! Soluciones encontradas: %zu",
      task.solutions().size());

    // Publicar la mejor solución para visualización en RViz
    task.introspection().publishSolution(*task.solutions().front());

    if (execute_trajectory) {
      RCLCPP_INFO(get_logger(), "Ejecutando trayectoria calculada sobre el controlador activo...");
      auto execution_result = task.execute(*task.solutions().front());
      if (execution_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Error durante la ejecución de la trayectoria (código: %d)",
          execution_result.val);
        return false;
      }
      RCLCPP_INFO(get_logger(), "¡Trayectoria de Pick & Place ejecutada SATISFACTORIAMENTE!");
    }

    return true;
  }

private:
  std::string arm_group_name_;
  std::string gripper_group_name_;
  std::string gripper_open_state_;
  std::string gripper_close_state_;
  std::string hand_frame_;
  std::string world_frame_;
  std::string object_name_;
  std::vector<double> object_dimensions_;
  std::vector<double> pick_pose_;
  std::vector<double> place_pose_;
  double approach_distance_;
  double lift_distance_;
  double retreat_distance_;
  bool execute_on_startup_;
  bool spawn_table_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  std::thread startup_thread_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<UrOnrobotMtcNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
