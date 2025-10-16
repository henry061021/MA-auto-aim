#include "cboard.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <iomanip>
namespace io
{
CBoard::CBoard(const std::string & config_path , const std::string& mode_str)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000),
   current_mode_(mode_str),
   can_(read_yaml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1))
// 注意: callback的运行会早于Cboard构造函数的完成
{
  // 1. 解析模式字符串，转换为CommMode枚举
    if (current_mode_ == "can") {
      // 直接初始化或赋值 can_ 成员变量
      tools::logger()->info("[Cboard] Waiting for q...");
      queue_.pop(data_ahead_);
      queue_.pop(data_behind_);
      tools::logger()->info("[Cboard] Opened.");
    } 
    else if (current_mode_ == "serial") 
    {
      auto yaml = tools::load(config_path);
      auto com_port = tools::read<std::string>(yaml, "com_port");
      try {
        serial_.setPort(com_port);
        serial_.open();
      } catch (const std::exception & e) {
          tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
          exit(1);
      } 
        thread_ = std::thread(&CBoard::read_thread, this);
        queue_.pop();
        tools::logger()->info("[Gimbal] First q received.");
    } 
    else {
        throw std::invalid_argument("Invalid communication mode");
    }
}

CBoard::~CBoard()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command)
{
  if(current_mode_ == "can") {
    can_frame frame;
    frame.can_id = send_canid_;
    frame.can_dlc = 8;
    frame.data[0] = (command.control) ? 1 : 0;
    frame.data[1] = (command.shoot) ? 1 : 0;
    frame.data[2] = (int16_t)(command.yaw * 1e2) >> 8;
    frame.data[3] = (int16_t)(command.yaw * 1e2);
    frame.data[4] = (int16_t)(command.pitch * 1e2) >> 8;
    frame.data[5] = (int16_t)(command.pitch * 1e2);
    frame.data[6] = (int16_t)(command.horizon_distance * 1e2) >> 8;
    frame.data[7] = (int16_t)(command.horizon_distance * 1e2);
    try {
       can_.write(&frame);
    } catch (const std::exception & e) {
      tools::logger()->warn("{}", e.what());
    }
  }
  else if(current_mode_ == "serial") {
    tx_data_.mode = command.control ? (command.shoot ? 2 : 1) : 0;
    tx_data_.yaw = command.yaw;
    tx_data_.pitch = command.pitch;
    tx_data_.crc16 = tools::get_crc16(
      reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));
  
    try {
      serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    } catch (const std::exception & e) {
      tools::logger()->warn("[CBoard] Failed to write serial: {}", e.what());
    }
  }
  
}

void CBoard::callback(const can_frame & frame)
{
  auto timestamp = std::chrono::steady_clock::now();

  if (frame.can_id == quaternion_canid_) {
    auto w = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e3;
    auto x = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e3;
    auto y = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e3;
    auto z = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e3;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({{w, x, y, z}, timestamp});
  }

  else if (frame.can_id == bullet_speed_canid_) {
    bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    mode = Mode(frame.data[2]);
    shoot_mode = ShootMode(frame.data[3]);
    ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }

  return yaml["can_interface"].as<std::string>();
}

bool CBoard::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void CBoard::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

void CBoard::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'M' || rx_data_.head[1] != 'A') continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
          error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

//     if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
//     tools::logger()->debug("[Gimbal] CRC16 check failed.");
    
//     // -------------------------- 新增：打印原始接收数据 --------------------------
//     // 1. 将 rx_data_ 转为 uint8_t*，获取原始字节流
//     const uint8_t* raw_data = reinterpret_cast<const uint8_t*>(&rx_data_);
//     // 2. 获取接收数据的总长度（rx_data_ 结构体的字节数，即实际接收的字节数）
//     const size_t data_len = sizeof(rx_data_);
    
//     // 3. 拼接原始数据的十六进制字符串（避免多次日志调用，更高效）
//     std::stringstream raw_data_str;
//     raw_data_str << "[Gimbal] Received raw data (Hex): ";
//     for (size_t i = 0; i < data_len; ++i) {
//         // 以 "0xXX " 格式拼接，不足两位补0（如 0x0A 而非 0xA）
//         raw_data_str << "0x" << std::hex << std::setw(2) << std::setfill('0') 
//                     << static_cast<int>(raw_data[i]) << " ";
//     }
//     // 4. 打印原始数据（用 debug 级别，和原日志级别一致）
//     tools::logger()->debug(raw_data_str.str());
//     // --------------------------------------------------------------------------
    
//     continue; // 校验失败，跳过后续处理
// }


    error_count = 0;
    // Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[0], rx_data_.q[2], rx_data_.q[3]);
    auto timestamp = std::chrono::steady_clock::now();
    auto w = (int16_t)(rx_data_.q[0] << 8 | rx_data_.q[1]) / 1e3;
    auto x = (int16_t)(rx_data_.q[2] << 8 | rx_data_.q[3]) / 1e3;
    auto y = (int16_t)(rx_data_.q[4] << 8 | rx_data_.q[5]) / 1e3;
    auto z = (int16_t)(rx_data_.q[6] << 8 | rx_data_.q[7]) / 1e3;
    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      continue;
    }
    queue_.push({{w,x,y,z},timestamp});

    std::lock_guard<std::mutex> lock(mutex_);
    bullet_speed = rx_data_.bullet_speed;
    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
    // state_.yaw = rx_data_.yaw;
    // state_.yaw_vel = rx_data_.yaw_vel;
    // state_.pitch = rx_data_.pitch;
    // state_.pitch_vel = rx_data_.pitch_vel;
    // state_.bullet_speed = rx_data_.bullet_speed;
    // state_.bullet_count = rx_data_.bullet_count;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

}  // namespace io