// 用 USART1 打印 IMU 数据
#include "usart_task.h"

#include "cmsis_os.h"
#include "imu_task.h"
#include "usart.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace
{
constexpr uint32_t kPrintPeriodMs = 20U;
constexpr uint32_t kUartTimeoutMs = 20U;
constexpr long kFixedPointScale = 1000L;

char tx_buffer[192];

long float_to_milli(float value)
{
  const float scaled = value * static_cast<float>(kFixedPointScale);
  return static_cast<long>((scaled >= 0.0F) ? (scaled + 0.5F) : (scaled - 0.5F));
}

std::size_t append_text(char * buffer, std::size_t size, std::size_t offset, const char * text)
{
  const int written = std::snprintf(buffer + offset, size - offset, "%s", text);
  if (written <= 0) {
    return offset;
  }

  const std::size_t remaining = size - offset;
  if (static_cast<std::size_t>(written) >= remaining) {
    return size - 1U;
  }

  return offset + static_cast<std::size_t>(written);
}

std::size_t append_fixed(char * buffer, std::size_t size, std::size_t offset, float value)
{
  long scaled = float_to_milli(value);
  const char * sign = "";
  if (scaled < 0L) {
    sign = "-";
    scaled = -scaled;
  }

  const int written = std::snprintf(
    buffer + offset, size - offset, "%s%ld.%03ld", sign, scaled / kFixedPointScale,
    scaled % kFixedPointScale);
  if (written <= 0) {
    return offset;
  }

  const std::size_t remaining = size - offset;
  if (static_cast<std::size_t>(written) >= remaining) {
    return size - 1U;
  }

  return offset + static_cast<std::size_t>(written);
}

std::size_t append_vector3(
  char * buffer, std::size_t size, std::size_t offset, const ImuVector3Data & value)
{
  offset = append_fixed(buffer, size, offset, value.x);
  offset = append_text(buffer, size, offset, ",");
  offset = append_fixed(buffer, size, offset, value.y);
  offset = append_text(buffer, size, offset, ",");
  return append_fixed(buffer, size, offset, value.z);
}

std::size_t build_imu_line(char * buffer, std::size_t size, const ImuTaskData & data)
{
  std::size_t offset = 0U;

  offset = append_text(buffer, size, offset, "rpy_rad:");
  offset = append_fixed(buffer, size, offset, data.euler.roll_rad);
  offset = append_text(buffer, size, offset, ",");
  offset = append_fixed(buffer, size, offset, data.euler.pitch_rad);
  offset = append_text(buffer, size, offset, ",");
  offset = append_fixed(buffer, size, offset, data.euler.yaw_rad);
  offset = append_text(buffer, size, offset, " gyro_rad_s:");
  offset = append_vector3(buffer, size, offset, data.gyro_rad_s);
  offset = append_text(buffer, size, offset, " acc_mps2:");
  offset = append_vector3(buffer, size, offset, data.acc_mps2);
  offset = append_text(buffer, size, offset, " temp_c:");
  offset = append_fixed(buffer, size, offset, data.temp_c);
  offset = append_text(buffer, size, offset, "\r\n");

  return offset;
}
}  // namespace

extern "C" void usart_task(void)
{
  while (true) {
    ImuTaskData data = {};
    if (imu_task_get_data(&data)) {
      const std::size_t length = build_imu_line(tx_buffer, sizeof(tx_buffer), data);
      HAL_UART_Transmit(
        &huart1, reinterpret_cast<uint8_t *>(tx_buffer), static_cast<uint16_t>(length),
        kUartTimeoutMs);
    }

    osDelay(kPrintPeriodMs);
  }
}
