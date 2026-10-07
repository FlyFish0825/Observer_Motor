#!/usr/bin/env python3
"""编译真实board_adc.c做主机回归，不连接硬件；用--cc指定本机C编译器。"""
import argparse
from pathlib import Path
import subprocess
import tempfile

HAL = r"""
#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } ADC_HandleTypeDef;
extern uint16_t test_cal[3];
#define VREFINT_CAL_ADDR (&test_cal[0])
#define TEMPSENSOR_CAL1_ADDR (&test_cal[1])
#define TEMPSENSOR_CAL2_ADDR (&test_cal[2])
#define VREFINT_CAL_VREF 3000U
#define TEMPSENSOR_CAL_VREFANALOG 3000U
#endif
"""
ADC = r"""
#include "stm32g4xx_hal.h"
extern ADC_HandleTypeDef hadc1;
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_ADCEx_RegularStop(ADC_HandleTypeDef *);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
"""
TEST = r"""
#include "board_adc.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
ADC_HandleTypeDef hadc1;
uint16_t test_cal[3] = {1662, 1040, 1375};
static uint16_t samples[3];
static unsigned rank, stops;
static int fail_start = -1, fail_poll = -1;
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *a) {
  assert(a == &hadc1); assert(rank < 3);
  return (int)rank == fail_start ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *a, uint32_t t) {
  (void)a; assert(t == 10); return (int)rank == fail_poll ? HAL_TIMEOUT : HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *a) { (void)a; return samples[rank++]; }
HAL_StatusTypeDef HAL_ADCEx_RegularStop(ADC_HandleTypeDef *a) {
  (void)a; ++stops; rank = 0; return HAL_OK;
}
static HAL_StatusTypeDef update(uint16_t bus, uint16_t temp, uint16_t ref) {
  samples[0]=bus; samples[1]=temp; samples[2]=ref; rank=0;
  return BoardAdc_Update();
}
static void unchanged(BoardAdcMeasurements_t old, float scale) {
  assert(memcmp(&old, BoardAdc_GetMeasurements(), sizeof old) == 0);
  assert(BoardAdc_GetVoltageScale() == scale);
}
int main(void) {
  assert(BoardAdc_GetVoltageScale() == 0);
  /* 同一个30℃标定点，在3.0 V与3.3 V供电下都必须恢复约30℃。 */
  assert(update(1000,1040,1662) == HAL_OK);
  assert(fabsf(BoardAdc_GetMeasurements()->temperature_c-30) < 0.001f);
  assert(update(909,945,1511) == HAL_OK);
  assert(fabsf(BoardAdc_GetMeasurements()->temperature_c-30) < 0.2f);
  /* 第二标定点必须恢复130℃，不能沿用旧版HAL的110℃。 */
  assert(update(1000,1375,1662) == HAL_OK);
  assert(fabsf(BoardAdc_GetMeasurements()->temperature_c-130) < 0.001f);
  /* 现场回归：旧算法报56℃，补偿后回到30℃附近，且保留小数。 */
  assert(update(163,1046,1671) == HAL_OK);
  BoardAdcMeasurements_t old = *BoardAdc_GetMeasurements();
  float scale = BoardAdc_GetVoltageScale();
  assert(fabsf(old.vref_mv-2983.842f) < 0.01f);
  assert(fabsf(old.temperature_c-30.1093f) < 0.01f);
  assert(fabsf(old.vbus_voltage-2.44701f) < 0.001f);
  assert(fabsf(old.vbus_voltage-163*scale) < 0.00001f);
  /* 第三个Rank失败或无效标定均不能发布半组数据或破坏电压系数。 */
  for(int r=0;r<3;++r) {
    fail_start=r; unsigned previous=stops;
    assert(update(2000,1100,1600)==HAL_ERROR); assert(stops==previous+1);
    unchanged(old,scale); fail_start=-1;
    fail_poll=r; previous=stops;
    assert(update(2000,1100,1600)==HAL_TIMEOUT); assert(stops==previous+1);
    unchanged(old,scale); fail_poll=-1;
  }
  for(unsigned i=0;i<3;++i) {
    uint16_t saved=test_cal[i]; test_cal[i]=0;
    assert(update(2000,1100,1600)==HAL_ERROR); unchanged(old,scale);
    test_cal[i]=saved;
  }
  test_cal[2]=1040;
  assert(update(2000,1100,1600)==HAL_ERROR); unchanged(old,scale);
  test_cal[2]=1375;
  for(unsigned raw=0;raw<=4095;raw+=4095) {
    assert(update(2000,1100,raw)==HAL_ERROR); unchanged(old,scale);
    assert(update(2000,raw,1600)==HAL_ERROR); unchanged(old,scale);
  }
  assert(update(2000,1100,100)==HAL_ERROR); unchanged(old,scale);
  puts("board_adc_host_test: PASS");
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="board-adc-test-") as folder:
        temp = Path(folder)
        for name, content in (("stm32g4xx_hal.h", HAL), ("adc.h", ADC),
                              ("test.c", TEST)):
            (temp / name).write_text(content, encoding="utf-8")
        exe = temp / "test.exe"
        subprocess.run([args.cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I", str(temp), "-I", str(root / "Core/Inc"),
                        str(root / "Core/Src/board_adc.c"), str(temp / "test.c"),
                        "-lm", "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
