// 用 USART1 向 SerialPlot 输出遥控器摇杆数据
#include "plot_task.h"

#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "remote_task.h"

namespace
{
constexpr uint32_t kPlotPeriodMs = 5U;

sp::Plotter plotter(&huart1, false);
}  // namespace

extern "C" void plot_task(void)
{
  while (true) {
    RemoteTaskData data = {};
    if (remote_task_get_data(&data)) {
      plotter.plot(data.ch_rh, data.ch_rv, data.ch_lh, data.ch_lv, data.ch_lu);
    }
    else {
      plotter.plot(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    }

    osDelay(kPlotPeriodMs);
  }
}
