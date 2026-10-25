#include "r5_steering_controller/r5_steering_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

namespace r5_steering_controller
{

using controller_interface::CallbackReturn;
using controller_interface::return_type;

CallbackReturn R5SteeringController::on_init()
{
  // 先让父类完成基础初始化（URDF 解析、父类参数、订阅者、里程计配置）。
  // 继承式插件的铁律：任何自定义初始化都必须在父类成功之后。
  const auto ret = AckermannSteeringController::on_init();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // R5 专属参数用普通 rclcpp 参数（父类走 generate_parameter_library，
  // 这里故意用轻量方式，演示插件读参数的最小闭环）。
  auto node = get_node();

  // ⚠️ 实测（本机 controller_manager 2.54.3）：加载控制器时，YAML 里属于
  // 本控制器的参数会被 controller_manager 直接预声明到节点上。
  // 所以必须“先查再声明”，否则 declare_parameter 抛出
  // ParameterAlreadyDeclaredException，且异常会穿透 pluginlib 边界
  // 把整个 gzserver abort 掉（不只是这个控制器加载失败）。
  auto ensure_double = [node](const std::string & name, double fallback) -> double
  {
    if (!node->has_parameter(name))
    {
      node->declare_parameter<double>(name, fallback);
    }
    return node->get_parameter(name).as_double();
  };
  auto ensure_bool = [node](const std::string & name, bool fallback) -> bool
  {
    if (!node->has_parameter(name))
    {
      node->declare_parameter<bool>(name, fallback);
    }
    return node->get_parameter(name).as_bool();
  };

  max_accel_ = ensure_double("r5.max_accel", max_accel_);
  max_decel_ = ensure_double("r5.max_decel", max_decel_);
  a_lat_max_ = ensure_double("r5.a_lat_max", a_lat_max_);
  max_steer_mech_ = ensure_double("r5.max_steer_mech", max_steer_mech_);
  max_steer_rate_ = ensure_double("r5.max_steer_rate", max_steer_rate_);
  speed_dependent_limit_ = ensure_bool("r5.speed_dependent_limit", speed_dependent_limit_);

  // 参数合法性自检：不合法直接拒绝 configure，比运行时出怪行为好排查得多
  if (max_accel_ <= 0.0 || max_decel_ <= 0.0 || a_lat_max_ <= 0.0 ||
      max_steer_mech_ <= 0.0 || max_steer_rate_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "r5_steering_controller: 参数必须为正数 (max_accel=%.3f max_decel=%.3f "
      "a_lat_max=%.3f max_steer_mech=%.3f max_steer_rate=%.3f)",
      max_accel_, max_decel_, a_lat_max_, max_steer_mech_, max_steer_rate_);
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "r5_steering_controller 初始化完成: max_accel=%.2f max_decel=%.2f "
    "a_lat_max=%.2f max_steer_rate=%.2f",
    max_accel_, max_decel_, a_lat_max_, max_steer_rate_);
  return CallbackReturn::SUCCESS;
}

return_type R5SteeringController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 父类照常执行：重载参考 → 里程计 FK → get_commands IK → 写命令接口 → 发布。
  // 我们不做参考层干预（见头文件“为什么放在命令层”的教训说明）。
  const auto ret =
    AckermannSteeringController::update_and_write_commands(time, period);
  if (ret != return_type::OK)
  {
    return ret;
  }

  const double dt = std::max(period.seconds(), 1e-4);

  // 命令接口布局（front_steering 时）：
  //   [0 .. n_rear-1]            后轮 velocity 指令
  //   [n_rear .. n_rear+n_steer-1] 前轮 position 指令
  const size_t n_rear = params_.rear_wheels_names.size();
  const size_t n_steer = params_.front_wheels_names.size();
  if (n_rear + n_steer > command_interfaces_.size())
  {
    return return_type::ERROR;
  }

  // 首周期从关节反馈初始化斜坡基准，避免“从 0 突跳到当前值”
  if (last_traction_cmds_.size() != n_rear)
  {
    last_traction_cmds_.resize(n_rear);
    for (size_t i = 0; i < n_rear; ++i)
    {
      double fb = state_interfaces_[i].get_value();  // velocity 反馈
      last_traction_cmds_[i] = std::isnan(fb) ? 0.0 : fb;
    }
  }
  if (last_steer_cmds_.size() != n_steer)
  {
    last_steer_cmds_.resize(n_steer);
    for (size_t i = 0; i < n_steer; ++i)
    {
      double fb = state_interfaces_[n_rear + i].get_value();  // position 反馈
      last_steer_cmds_[i] = std::isnan(fb) ? 0.0 : fb;
    }
  }

  // ---------- ① 驱动轮速度斜坡【命令层】 ----------
  // 底盘加速度限值换算到轮子角加速度：v = ω·r ⇒ dω/dt = a / r
  // （量纲检查：[m/s²] / [m] = [rad/s²]。第一版误写成 a*r，
  //   斜坡慢了 r² 倍，是验收测试抓出来的——限幅器必须配量纲自检。）
  // 直行时左右同速，整形后底盘加速度精确等于 max_accel；转弯进入的
  // 瞬态按轮独立整形，与真实底盘“每个电机各有一个速度环”的行为一致。
  const double r_wheel = std::max(ackermann_params_.rear_wheels_radius, 1e-4);
  const double dw_acc = max_accel_ / r_wheel * dt;
  const double dw_dec = max_decel_ / r_wheel * dt;
  double v_sum = 0.0;
  for (size_t i = 0; i < n_rear; ++i)
  {
    double raw = command_interfaces_[i].get_value();
    if (std::isnan(raw))
    {
      last_traction_cmds_[i] = 0.0;
      continue;
    }
    raw = std::clamp(raw, last_traction_cmds_[i] - dw_dec, last_traction_cmds_[i] + dw_acc);
    command_interfaces_[i].set_value(raw);
    last_traction_cmds_[i] = raw;
    v_sum += raw;
  }
  // 限幅后的指令车速（用于速度相关转向限幅），而不是参考车速——
  // 限幅器作用的对象必须是“实际会被执行”的量，否则自欺欺人。
  const double v_cmd = std::fabs(v_sum / static_cast<double>(n_rear)) * r_wheel;

  // ---------- ② 速度相关转向限幅【命令层】 ----------
  // a_y = v² * tan(φ) / L ≤ a_lat_max  ⇒  tan(φ_cap) = a_lat_max * L / v²
  // 低速时上式发散，退化为机械限位。作用在轮角上是近似（见头文件说明）。
  double phi_cap = max_steer_mech_;
  if (speed_dependent_limit_ && v_cmd > 0.05)
  {
    const double L = std::max(ackermann_params_.wheelbase, 1e-3);
    phi_cap = std::min(phi_cap, std::atan(a_lat_max_ * L / (v_cmd * v_cmd)));
  }

  // ---------- ③ 转向幅度 + 速率限幅【命令层】 ----------
  const double max_delta = max_steer_rate_ * dt;
  for (size_t i = 0; i < n_steer; ++i)
  {
    double raw = command_interfaces_[n_rear + i].get_value();
    if (std::isnan(raw))
    {
      double fb = state_interfaces_[n_rear + i].get_value();
      last_steer_cmds_[i] = std::isnan(fb) ? 0.0 : fb;
      continue;
    }
    // 先夹幅度（侧向加速度 / 机械限位）
    raw = std::clamp(raw, -phi_cap, phi_cap);
    // 再夹速率（舵机响应）
    raw = std::clamp(
      raw, last_steer_cmds_[i] - max_delta, last_steer_cmds_[i] + max_delta);
    command_interfaces_[n_rear + i].set_value(raw);
    last_steer_cmds_[i] = raw;
  }

  return return_type::OK;
}

}  // namespace r5_steering_controller

PLUGINLIB_EXPORT_CLASS(
  r5_steering_controller::R5SteeringController,
  controller_interface::ChainableControllerInterface)
