/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>    /* snprintf：格式化時間字串與統計報表 */
#include <string.h>   /* strlen / strcmp / strncmp：字串長度與指令比對 */
#include <stdint.h>   /* uint8_t / uint32_t 等固定寬度整數型別 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
  * 單筆警報歷史紀錄。
  * 兩個欄位都以「秒」為單位，顯示時再用 format_time() 轉成 00h 00m 00s。
  */
typedef struct
{
  uint32_t start_sec;     /* 觸發時的系統運行秒數（從自我檢測完成後開始累計） */
  uint32_t duration_sec;  /* 這次警報持續的秒數（從觸發到 2FA 解除） */
} AlarmRecord_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * ============================================================================
 *  程式架構與硬體時基總覽
 * ============================================================================
 *
 *  【系統時脈】
 *    HSI 16 MHz → PLLM /1 → PLLN x20 = 320 MHz → PLLR /2 = 160 MHz (SYSCLK)
 *    AHB、APB1、APB2 的分頻都是 /1，所以 TIM2、TIM3 的輸入時脈都是 160 MHz。
 *
 *  【TIM2：LED 的 PWM】（輸出腳依 CubeMX 分配，Nucleo 上通常是 PA5 / LD2）
 *    預除頻 159 → 160 MHz / 160 = 1 MHz
 *    週期 (ARR) 999 → 1000 個計數 → PWM 頻率 1 kHz
 *    佔空比 = CCR / 1000
 *      CCR = 100  → 10%（待機低亮度）
 *      CCR = 1000 → 100%（CCR 大於 ARR 時輸出恆為高電位，即全亮）
 *      CCR = 0    → 0%（熄滅）
 *
 *  【TIM3：100 ms 系統節拍】
 *    預除頻 15999 → 160 MHz / 16000 = 10 kHz
 *    週期 (ARR) 999 → 1000 個計數 → 10 Hz，即每 100 ms 溢位一次
 *    本程式不開 TIM3 中斷，而是在主迴圈輪詢「更新旗標」(UIF) 來取得節拍，
 *    這樣就不必修改 stm32g4xx_it.c。
 *
 *  【運行架構：主迴圈 + 輪詢 + 一個中斷】
 *    - 按鈕 (PB11)：EXTI 下緣中斷，ISR 只設旗標 button_event。
 *    - 100 ms 節拍：輪詢 TIM3 更新旗標，呼叫 on_tick()。
 *    - 串口接收：輪詢 LPUART1 的 RXNE 旗標，逐字元組成一行指令。
 *    - 以上三者都集中在 service()，所有等待迴圈 (wait_ms) 都會持續呼叫它，
 *      所以等待期間按鈕、串口與節拍都不會漏掉。
 *
 *  【資料流程】
 *    按鈕 ISR → button_event → service() → trigger_alarm()
 *    串口字元 → rx_buf → process_line() → handle_alarm_line()/handle_normal_line()
 *    TIM3 節拍 → on_tick() → 更新運行時間、SOS 節奏、5 秒驗證倒數
 * ============================================================================
 */

/* ---- LED 亮度（TIM2 通道 1 的 CCR 值，ARR = 999） ---- */
#define LED_DIM            100U    /* 10%：待機低亮度 */
#define LED_FULL           1000U   /* 100%：CCR > ARR，輸出恆為高電位 */

/* ---- 蜂鳴器（主動式，高電位響、低電位靜音，只做開關） ---- */
#define BUZZER_PORT        GPIOC
#define BUZZER_PIN         GPIO_PIN_8

/* ---- 時間與驗證參數 ---- */
#define SOS_CYCLE_UNITS    35U     /* 一個 SOS 循環 = 35 個 100 ms 單位 = 3.5 秒 */
#define AUTH_WINDOW_TICKS  50U     /* 第二步的 5 秒視窗 = 50 個 100 ms 單位 */
#define AUTH_CODE_STR      "1234"  /* 寫死的授權碼 */

/* ---- 串口與歷史紀錄 ---- */
#define RX_BUF_SIZE        32U     /* 接收緩衝區大小；含結尾 '\0'，所以最多存 31 個字元 */
#define HISTORY_MAX        3U      /* 只保留最近 3 次警報紀錄 */

/* ---- 按鈕 ---- */
#define DEBOUNCE_MS        50U     /* 防彈跳時間：50 ms 內的重複下緣一律忽略 */

/* ---- 串口訊息字串（逐字照規格，勿任意修改） ----
 *  \r\n 為回車加換行；結尾帶 _FMT 的字串含 %s，要搭配 snprintf 使用。
 */
#define MSG_POST_START   "\r\n[POST] Starting Hardware Self-Test...\r\n"
#define MSG_POST_PASS    "[POST] Hardware Self-Test PASSED: LED: OK | Buzzer: OK | EXTI: OK\r\n"
#define MSG_WELCOME      "\r\n[SYSTEM] Secure Remote Monitoring Initialized.\r\nCommands: 'stats', 'cmd test', or alarm prompt for 2FA unlock.\r\n"
#define MSG_STATUS_FMT   "\n[STATUS] System: NORMAL | Uptime: %s | Status: SECURE\r\n"
#define MSG_ALARM_TRIG   "\r\n[ALARM] SOS Triggered! SECURE AUTH REQUIRED.\r\n-> Type: clear <auth_code>\r\n"
#define MSG_ALARM_ACTIVE "\n\n[ALARM] System: SOS ACTIVE! | Waiting for 2FA clearance...\r\n"
#define MSG_STEP1_OK     "\r\n[SECURITY] Step 1 Passed! Enter 'r' or 'R' within 5 seconds to clear SOS...\r\n"
#define MSG_BAD_CODE     "\r\n[ERROR] Invalid Auth Code! Access Denied.\r\n"
#define MSG_NEED_CODE    "\r\n[WARNING] Alarm Active. Please enter internal auth code first.\r\n"
#define MSG_BAD_STEP2    "\r\n[ERROR] Incorrect input! Expected 'r' or 'R'.\r\n"
#define MSG_WIN_EXPIRED  "\r\n[WARNING] 5-Second Window Expired! Please re-enter internal auth code.\r\n"
#define MSG_CLEARED_FMT  "\r\n[INFO] 2FA Verified Successfully! SOS Cleared. (Duration: %s)\r\n"
#define MSG_TEST_START   "\r\n[CMD] Executing remote test: LED blink x2 & Buzzer beep...\r\n"
#define MSG_TEST_DONE    "[CMD] Test sequence completed successfully.\r\n"
#define MSG_UNKNOWN      "\r\n[INFO] Normal state. Available commands: stats, cmd test\r\n"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* ---- 系統時基 ----
 * 每個 100 ms 節拍加 1。運行秒數 = uptime_ticks / 10。
 * 只有自我檢測完成、TIM3 啟動後才會開始累計。
 */
static uint32_t uptime_ticks = 0;

/* ---- 警報狀態 ---- */
/* 是否處於警報中。會被主迴圈與按鈕 ISR 同時讀取，所以宣告為 volatile。 */
static volatile uint8_t  alarm_active = 0;
static uint32_t alarm_ticks = 0;          /* 本次警報累計的 100 ms 單位數（除以 10 即持續秒數） */
static uint32_t alarm_phase = 0;          /* 目前在 SOS 循環中的位置 (0 ~ 34) */
static uint32_t alarm_start_sec = 0;      /* 本次警報觸發時的系統運行秒數 */
static uint32_t total_triggers = 0;       /* 開機以來的總觸發次數 */

/* ---- 兩步驟驗證 ---- */
static uint8_t  auth_step = 0;            /* 0 = 等授權碼 (clear 1234)；1 = 等確認字元 (r/R) */
static uint32_t auth_window_ticks = 0;    /* 第二步剩餘的 100 ms 單位數，倒數到 0 即逾時 */

/* ---- 歷史紀錄 ----
 * history[0] 永遠是最新一筆，數字越大越舊。
 * history_count 為目前有效筆數 (0 ~ HISTORY_MAX)。
 */
static AlarmRecord_t history[HISTORY_MAX];
static uint8_t history_count = 0;

/* ---- 按鈕事件 ----
 * button_event 由 EXTI 中斷設為 1，主迴圈 service() 取走後清 0，
 * 兩邊都會存取，所以用 volatile。
 * button_last_ms 記錄上次有效按下的 HAL_GetTick() 時間，用於防彈跳。
 */
static volatile uint8_t  button_event = 0;
static volatile uint32_t button_last_ms = 0;

/* ---- 串口接收緩衝 ---- */
static char    rx_buf[RX_BUF_SIZE];   /* 逐字元累積目前這一行 */
static uint8_t rx_len = 0;            /* 緩衝區目前已存的字元數 */
static uint8_t rx_overflow = 0;       /* 1 = 本行已超長，持續丟棄直到收到換行 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void uart_print(const char *s);
static void format_time(char *buf, size_t n, uint32_t secs);
static void led_set(uint32_t duty);
static void buzzer_set(uint8_t on);
static uint8_t sos_is_on(uint32_t unit);
static void on_tick(void);
static void tick_poll(void);
static void delay_ticking(uint32_t ms);
static void trigger_alarm(void);
static void clear_alarm(void);
static void handle_alarm_line(const char *line);
static void handle_normal_line(const char *line);
static void process_line(const char *line);
static void uart_poll_rx(void);
static void service(void);
static uint8_t wait_ms(uint32_t ms);
static void print_stats(void);
static void run_remote_test(void);
static void button_isr(uint16_t pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==========================================================================
 *  基本輸出工具
 * ========================================================================== */

/**
  * @brief  從串口送出一個以 '\0' 結尾的字串（阻塞式）。
  * @param  s: 要送出的字串
  * @note   逾時設 200 ms。以 115200 bps 送約 100 個字元只需約 9 ms，
  *         遠小於逾時，實務上不會被截斷。
  */
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

/**
  * @brief  把秒數格式化成固定格式「兩位數時、分、秒」，例如 00h 05m 12s。
  * @param  buf:  輸出緩衝區
  * @param  n:    緩衝區大小
  * @param  secs: 總秒數
  * @note   %lu 搭配 unsigned long 轉型，避免 32 位元平台上的格式不一致警告。
  *         小時超過 99 時會自然變成三位數，不會被截斷。
  */
static void format_time(char *buf, size_t n, uint32_t secs)
{
  snprintf(buf, n, "%02luh %02lum %02lus",
           (unsigned long)(secs / 3600U),          /* 時 */
           (unsigned long)((secs % 3600U) / 60U),  /* 分 */
           (unsigned long)(secs % 60U));           /* 秒 */
}

/**
  * @brief  設定 LED 亮度（直接寫入 TIM2 通道 1 的比較值）。
  * @param  duty: 0 = 熄滅、LED_DIM = 10%、LED_FULL = 全亮
  */
static void led_set(uint32_t duty)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
}

/**
  * @brief  開關蜂鳴器。主動式蜂鳴器高電位響，所以 on=1 輸出高電位。
  */
static void buzzer_set(uint8_t on)
{
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ==========================================================================
 *  SOS 摩斯密碼節奏
 * ========================================================================== */

/**
  * @brief  判斷 SOS 循環中的某個單位是否為「亮 / 響」的時段。
  * @param  u: 循環中的單位索引 (0 ~ 34)，每單位 100 ms
  * @retval 1 = LED 全亮且蜂鳴器響；0 = LED 熄滅且蜂鳴器靜音
  *
  * 一個循環 35 個單位 (3.5 秒)，各區間皆「含起點、不含終點」：
  *
  *   單位:  0 1 2 3 4 5 6 7 8 9 ...
  *   S 短:  ■ ■ _ ■ ■ _ ■ ■ _        [0,2) [3,5) [6,8)    各 0.2 秒
  *   O 長:  [9,12) [13,16) [17,20)                         各 0.3 秒
  *   S 短:  [21,23) [24,26) [27,29)                        各 0.2 秒
  *   其餘 (含 29 ~ 34 的最後 0.6 秒) 為休止。
  *
  * 每個「亮」之間留 1 個單位的靜音，用來分隔字母內的各個音。
  */
static uint8_t sos_is_on(uint32_t u)
{
  /* S：三短 */
  if (u < 2U)               return 1;
  if (u >= 3U  && u < 5U)   return 1;
  if (u >= 6U  && u < 8U)   return 1;
  /* O：三長 */
  if (u >= 9U  && u < 12U)  return 1;
  if (u >= 13U && u < 16U)  return 1;
  if (u >= 17U && u < 20U)  return 1;
  /* S：三短 */
  if (u >= 21U && u < 23U)  return 1;
  if (u >= 24U && u < 26U)  return 1;
  if (u >= 27U && u < 29U)  return 1;
  /* 其餘為休止 */
  return 0;
}

/* ==========================================================================
 *  100 ms 節拍處理
 * ========================================================================== */

/**
  * @brief  每 100 ms 執行一次的節拍處理，負責三件事：
  *         1. 累計系統運行時間
  *         2. 警報期間推進 SOS 節奏並驅動 LED／蜂鳴器
  *         3. 第二步驗證的 5 秒倒數
  */
static void on_tick(void)
{
  /* 1. 系統運行時間（不論是否在警報中都累計） */
  uptime_ticks++;

  if (alarm_active)
  {
    /* 警報持續時間：每個節拍加 1，解除時除以 10 換成秒 */
    alarm_ticks++;

    /* 2. 推進 SOS 循環位置，在 0 ~ 34 之間循環 */
    alarm_phase = (alarm_phase + 1U) % SOS_CYCLE_UNITS;
    if (sos_is_on(alarm_phase))
    {
      led_set(LED_FULL);   /* 亮的時段：LED 全亮 + 蜂鳴器響 */
      buzzer_set(1);
    }
    else
    {
      led_set(0);          /* 休止時段：LED 熄滅 + 蜂鳴器靜音 */
      buzzer_set(0);
    }

    /* 3. 第二步的 5 秒倒數（只在 auth_step == 1 時進行） */
    if (auth_step == 1U)
    {
      if (auth_window_ticks > 0U)
      {
        auth_window_ticks--;
      }
      if (auth_window_ticks == 0U)
      {
        /* 時間到：退回第一步，使用者需重新輸入授權碼 */
        auth_step = 0U;
        uart_print(MSG_WIN_EXPIRED);
      }
    }
  }
}

/**
  * @brief  輪詢 TIM3 的更新旗標 (UIF) 來取得 100 ms 節拍。
  * @note   TIM3 每 100 ms 溢位一次並設起 UIF；這裡發現旗標就清除並呼叫 on_tick()。
  *         旗標只有 1 位元，若主程式被阻塞超過 100 ms 沒來檢查，
  *         多個節拍會合併成一個。本程式的阻塞操作（串口輸出）都在
  *         十幾毫秒內，所以不會發生。
  */
static void tick_poll(void)
{
  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE))
  {
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    on_tick();
  }
}

/**
  * @brief  阻塞延遲 ms 毫秒，期間仍持續累計節拍。
  * @note   用於 cmd test：延遲期間不能讓運行時間停止累計，
  *         所以不能直接用 HAL_Delay()。此函式不處理按鈕與串口。
  */
static void delay_ticking(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ms)   /* 減法寫法可正確處理計數溢位 */
  {
    tick_poll();
  }
}

/* ==========================================================================
 *  警報觸發與解除
 * ========================================================================== */

/**
  * @brief  進入 SOS 警報狀態（只會在一般狀態下被呼叫）。
  * @note   依規格做這幾件事：總次數加 1、記下起始時間、重設持續時間計數、
  *         把驗證流程重設回第一步、輸出警報訊息。
  */
static void trigger_alarm(void)
{
  total_triggers++;                        /* 總觸發次數加 1 */
  alarm_start_sec = uptime_ticks / 10U;    /* 記下此刻的系統運行秒數 */
  alarm_ticks = 0;                         /* 重新開始計算持續時間 */
  auth_step = 0;                           /* 驗證回到第一步 */
  auth_window_ticks = 0;

  /* 把循環位置設為最後一格 (34)。下一個節拍 (最多 100 ms 後) 會翻到第 0 格，
   * 這樣第一個單位從節拍邊界開始，每個單位都剛好 100 ms。 */
  alarm_phase = SOS_CYCLE_UNITS - 1U;

  led_set(0);        /* 先熄滅，等第一個節拍再依 SOS 節奏驅動 */
  buzzer_set(0);
  alarm_active = 1;  /* 最後才設旗標，確保上面的初始化已完成 */
  uart_print(MSG_ALARM_TRIG);
}

/**
  * @brief  2FA 驗證成功後解除警報，並把本次紀錄存入歷史。
  */
static void clear_alarm(void)
{
  char tbuf[24];                           /* 暫存時間字串 */
  char out[96];                            /* 暫存完整輸出訊息 */
  uint32_t duration = alarm_ticks / 10U;   /* 100 ms 單位數 ÷ 10，取整數秒 */
  uint8_t i;

  /* 先回到一般狀態：警報結束、驗證歸零、LED 回待機亮度、蜂鳴器關閉 */
  alarm_active = 0;
  auth_step = 0;
  auth_window_ticks = 0;
  led_set(LED_DIM);
  buzzer_set(0);

  /* 存入歷史紀錄：新的放在最前面，舊的依序往後移，最舊的被擠掉。
   * 從尾端往前搬，避免覆蓋尚未搬動的資料。 */
  for (i = HISTORY_MAX - 1U; i > 0U; i--)
  {
    history[i] = history[i - 1U];
  }
  history[0].start_sec = alarm_start_sec;
  history[0].duration_sec = duration;
  if (history_count < HISTORY_MAX)   /* 有效筆數最多 3 */
  {
    history_count++;
  }

  /* 輸出解除成功訊息與持續時間 */
  format_time(tbuf, sizeof(tbuf), duration);
  snprintf(out, sizeof(out), MSG_CLEARED_FMT, tbuf);
  uart_print(out);
}

/* ==========================================================================
 *  指令處理
 * ========================================================================== */

/**
  * @brief  警報期間的輸入處理（兩步驟驗證狀態機）。
  * @param  line: 一行已去除換行的指令字串
  *
  *  第一步 (auth_step == 0)：
  *    - "clear 1234"        → 進入第二步，啟動 5 秒視窗
  *    - "clear " + 其他內容 → 授權碼錯誤，仍留在第一步
  *    - 其他任何內容        → 提醒先輸入授權碼，仍留在第一步
  *  第二步 (auth_step == 1)：
  *    - "r" 或 "R"          → 解除警報
  *    - 其他任何內容        → 提示輸入錯誤，仍留在第二步，倒數繼續
  */
static void handle_alarm_line(const char *line)
{
  if (auth_step == 0U)
  {
    /* 以 "clear " 開頭（含一個空格）才視為在輸入授權碼 */
    if (strncmp(line, "clear ", 6) == 0)
    {
      /* line + 6 跳過 "clear "，剩下的部分必須完全等於授權碼 */
      if (strcmp(line + 6, AUTH_CODE_STR) == 0)
      {
        auth_step = 1U;                       /* 進入第二步 */
        auth_window_ticks = AUTH_WINDOW_TICKS;/* 開始 5 秒倒數 */
        uart_print(MSG_STEP1_OK);
      }
      else
      {
        uart_print(MSG_BAD_CODE);             /* 數字錯誤或非數字 */
      }
    }
    else
    {
      uart_print(MSG_NEED_CODE);              /* 沒有以 clear 開頭 */
    }
  }
  else
  {
    /* 第二步：必須剛好是單一字元 r 或 R（line[1] 必須是結尾 '\0'） */
    if ((line[0] == 'r' || line[0] == 'R') && line[1] == '\0')
    {
      clear_alarm();
    }
    else
    {
      uart_print(MSG_BAD_STEP2);              /* 留在第二步，倒數繼續 */
    }
  }
}

/**
  * @brief  輸出統計報表：總觸發次數、目前運行時間、最近 3 次警報紀錄。
  * @note   報表分多次送出而不是一次組成大字串，可以省 RAM，
  *         每行也只需要一個 128 位元組的暫存區。
  */
static void print_stats(void)
{
  char tbuf[24];    /* 時間字串暫存 1 */
  char tbuf2[24];   /* 時間字串暫存 2（歷史紀錄一行要同時顯示兩個時間） */
  char line[128];   /* 單行輸出暫存 */
  uint8_t i;

  /* 報表前面有一個空行 */
  uart_print("\r\n");
  uart_print("========================================\r\n");
  uart_print("       SYSTEM TELEMETRY & STATS         \r\n");
  uart_print("========================================\r\n");

  /* 總觸發次數 */
  snprintf(line, sizeof(line), "- Total SOS Triggers    : %lu\r\n", (unsigned long)total_triggers);
  uart_print(line);

  /* 目前系統運行時間 */
  format_time(tbuf, sizeof(tbuf), uptime_ticks / 10U);
  snprintf(line, sizeof(line), "- System Uptime         : %s\r\n", tbuf);
  uart_print(line);

  uart_print("----------------------------------------\r\n");
  uart_print(" Recent SOS History (Max 3):\r\n");

  if (history_count == 0U)
  {
    /* 還沒有任何已解除的警報 */
    uart_print(" (No alarm history recorded yet)\r\n");
  }
  else
  {
    /* 編號 [1] 為最新，對應 history[0] */
    for (i = 0; i < history_count; i++)
    {
      format_time(tbuf, sizeof(tbuf), history[i].start_sec);
      format_time(tbuf2, sizeof(tbuf2), history[i].duration_sec);
      snprintf(line, sizeof(line), " [%u] Triggered @ %s | Duration: %s\r\n",
               (unsigned)(i + 1U), tbuf, tbuf2);
      uart_print(line);
    }
  }
  uart_print("========================================\r\n");
}

/**
  * @brief  遠端測試：LED 全亮並蜂鳴器響 200 ms、恢復並靜音 200 ms，共兩次。
  * @note   使用 delay_ticking() 讓運行時間在測試期間不中斷。
  *         測試期間不處理按鈕與串口輸入，約 0.8 秒後恢復正常。
  */
static void run_remote_test(void)
{
  uint8_t i;

  uart_print(MSG_TEST_START);
  for (i = 0; i < 2U; i++)
  {
    led_set(LED_FULL);       /* 亮 + 響 200 ms */
    buzzer_set(1);
    delay_ticking(200);
    led_set(LED_DIM);        /* 恢復待機亮度 + 靜音 200 ms */
    buzzer_set(0);
    delay_ticking(200);
  }
  uart_print(MSG_TEST_DONE);
}

/**
  * @brief  一般狀態（非警報）的指令處理。
  * @note   指令必須完全相符、區分大小寫；"cmd test" 中間是一個空格。
  */
static void handle_normal_line(const char *line)
{
  if (strcmp(line, "stats") == 0)
  {
    print_stats();
  }
  else if (strcmp(line, "cmd test") == 0)
  {
    run_remote_test();
  }
  else
  {
    /* 其他任何內容：顯示正常狀態與可用指令 */
    uart_print(MSG_UNKNOWN);
  }
}

/**
  * @brief  依目前狀態把整行輸入分派給警報處理或一般處理。
  *         警報期間 stats、cmd test 都不可用。
  */
static void process_line(const char *line)
{
  if (alarm_active)
  {
    handle_alarm_line(line);
  }
  else
  {
    handle_normal_line(line);
  }
}

/* ==========================================================================
 *  串口接收（輪詢 RXNE）
 * ========================================================================== */

/**
  * @brief  輪詢 LPUART1，把收到的字元組成一行，收到換行時交給 process_line()。
  *
  *  接收規則：
  *   - 每收到一個字元就存進 rx_buf。
  *   - 收到 '\r' 或 '\n'，且緩衝區不是空的，才把整行交出並清空緩衝區。
  *     因此 CRLF 的第二個字元到達時緩衝區已是空的，會被自然忽略。
  *   - 緩衝區大小 32，最多存 31 個字元；超過時整行丟棄，
  *     持續忽略後續字元，直到下一個換行才重新開始。
  */
static void uart_poll_rx(void)
{
  /* RXNE = 接收資料暫存器非空，表示有字元可讀；一次把積壓的全部讀完 */
  while (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE))
  {
    /* 讀 RDR 會同時清除 RXNE 旗標；只取低 8 位元（8 data bits） */
    char c = (char)(hlpuart1.Instance->RDR & 0xFFU);

    if (c == '\r' || c == '\n')
    {
      if (rx_overflow)
      {
        /* 這一行先前已超長：收到換行時把它整行丟棄，重新開始 */
        rx_overflow = 0;
        rx_len = 0;
      }
      else if (rx_len > 0U)
      {
        /* 正常的一行：補上字串結尾、清空長度，再交給指令解析。
         * 先把 rx_len 歸零再呼叫 process_line()，這樣處理期間
         * 緩衝區已是乾淨的。 */
        rx_buf[rx_len] = '\0';
        rx_len = 0;
        process_line(rx_buf);
      }
      /* 緩衝區為空（例如 CRLF 的第二個字元或單獨按 Enter）→ 直接忽略 */
    }
    else if (!rx_overflow)
    {
      if (rx_len < (RX_BUF_SIZE - 1U))
      {
        rx_buf[rx_len++] = c;   /* 保留最後 1 格給結尾 '\0' */
      }
      else
      {
        /* 已經存滿 31 個字元還有新字元：判定超長，整行丟棄，
         * 之後的字元一律忽略，直到下一個換行。 */
        rx_overflow = 1;
        rx_len = 0;
      }
    }
    /* rx_overflow 為 1 時，非換行字元在此被直接丟棄 */
  }

  /* 清除可能的串口錯誤旗標。若發生 overrun (ORE) 等錯誤而不清除，
   * 接收器可能卡住，之後收不到任何字元。 */
  if (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_ORE) ||   /* 溢位：資料來不及讀 */
      __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_FE)  ||   /* 框架錯誤 */
      __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_NE)  ||   /* 雜訊錯誤 */
      __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_PE))      /* 同位檢查錯誤 */
  {
    __HAL_UART_CLEAR_FLAG(&hlpuart1, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                     UART_CLEAR_NEF  | UART_CLEAR_PEF);
  }
}

/* ==========================================================================
 *  背景服務與等待迴圈
 * ========================================================================== */

/**
  * @brief  統一的背景服務：節拍、按鈕、串口。
  *         所有等待迴圈都反覆呼叫它，確保等待期間不會漏掉任何事件。
  */
static void service(void)
{
  /* 1. 100 ms 節拍（時間、SOS 節奏、5 秒倒數） */
  tick_poll();

  /* 2. 按鈕事件：ISR 只設旗標，真正的觸發動作在這裡（主迴圈環境）執行，
   *    避免在中斷裡做串口輸出等耗時操作。 */
  if (button_event)
  {
    button_event = 0;
    if (!alarm_active)      /* 再檢查一次：警報中按按鈕會被忽略 */
    {
      trigger_alarm();
    }
  }

  /* 3. 串口輸入 */
  uart_poll_rx();
}

/**
  * @brief  等待 ms 毫秒，期間持續執行 service()。
  * @param  ms: 等待時間（毫秒）
  * @retval 1 = 等待期間警報狀態改變（被觸發或被解除），提前結束
  *         0 = 時間到，狀態沒有改變
  * @note   主迴圈用它實現「可被打斷的等待」：一旦狀態改變，
  *         呼叫端就放棄原本的流程，回到迴圈開頭依新狀態重新開始。
  */
static uint8_t wait_ms(uint32_t ms)
{
  uint8_t initial = alarm_active;     /* 記下進入時的狀態，用來偵測變化 */
  uint32_t t0 = HAL_GetTick();

  while ((HAL_GetTick() - t0) < ms)
  {
    service();
    if (alarm_active != initial)
    {
      return 1;                        /* 警報被觸發或解除，立刻離開 */
    }
  }
  return 0;                            /* 完整等完 */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* ======================================================================
   *  開機自我檢測 (POST)，總長約 7.5 秒
   *    LED 全亮 1.5 秒 → 蜂鳴器 (響 1.5 秒 + 停 1.5 秒) x 2
   *  此階段 TIM3 尚未啟動，所以運行時間還沒開始累計，可直接用 HAL_Delay()。
   * ====================================================================== */

  buzzer_set(0);                               /* 確保蜂鳴器一開始是靜音 */
  led_set(0);                                  /* 先把比較值設為 0，避免啟動瞬間閃一下 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);    /* 啟動 LED 的 PWM 輸出 */

  uart_print(MSG_POST_START);

  /* LED 全亮 1.5 秒，再降回 10% 待機亮度 */
  led_set(LED_FULL);
  HAL_Delay(1500);
  led_set(LED_DIM);

  /* 蜂鳴器：響 1.5 秒、停 1.5 秒，連續兩次 */
  for (uint8_t i = 0; i < 2U; i++)
  {
    buzzer_set(1);
    HAL_Delay(1500);
    buzzer_set(0);
    HAL_Delay(1500);
  }

  /* 注意：訊息會寫 EXTI OK，但這裡沒有真的測試按鈕，只是固定文字（依規格） */
  uart_print(MSG_POST_PASS);

  /* ======================================================================
   *  啟動 100 ms 節拍
   *  自我檢測完成後才啟動 TIM3，所以運行時間從這裡才開始累計。
   * ====================================================================== */
  __HAL_TIM_SET_COUNTER(&htim3, 0);            /* 計數器歸零，第一個節拍剛好 100 ms 後 */
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);/* 清掉可能殘留的更新旗標 */
  HAL_TIM_Base_Start(&htim3);                  /* 不帶中斷的啟動，旗標由主迴圈輪詢 */

  /* 清掉自我檢測期間可能殘留的事件，避免一開機就誤觸警報或誤解析指令 */
  button_event = 0;                            /* 檢測期間若有按鈕 ISR 觸發，丟棄 */
  rx_len = 0;
  rx_overflow = 0;
  while (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE))
  {
    (void)hlpuart1.Instance->RDR;              /* 讀出並丟棄檢測期間收到的字元 */
  }
  __HAL_UART_CLEAR_FLAG(&hlpuart1, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                   UART_CLEAR_NEF  | UART_CLEAR_PEF);

  uart_print(MSG_WELCOME);                     /* 歡迎訊息與可用指令提示 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    char tbuf[24];   /* 時間字串暫存 */
    char out[96];    /* 狀態回報訊息暫存 */

    /* ==================================================================
     *  警報狀態
     *  每約 5 秒輸出一次提示。期間 LED/蜂鳴器由 on_tick() 依 SOS 節奏驅動，
     *  串口輸入與 2FA 驗證由 wait_ms() 內的 service() 處理。
     * ================================================================== */
    if (alarm_active)
    {
      if (wait_ms(5000))
      {
        /* 等待中警報被解除了：不再輸出「警報持續中」，
         * 直接回到迴圈開頭，改走一般狀態。 */
        continue;
      }
      uart_print(MSG_ALARM_ACTIVE);
      continue;
    }

    /* ==================================================================
     *  一般狀態，每圈約 5 秒：
     *    等 4.5 秒 → LED 全亮 0.5 秒 → 回到 10% → 輸出狀態訊息
     *  兩段等待都可被按鈕打斷（打斷後 wait_ms 回傳 1，立刻離開）。
     * ================================================================== */

    /* 先等 4.5 秒（LED 維持 10% 待機亮度） */
    if (wait_ms(4500))
    {
      continue;     /* 被按鈕打斷：警報已啟動，回迴圈開頭 */
    }

    /* LED 全亮 0.5 秒，作為「系統運作中」的心跳閃爍 */
    led_set(LED_FULL);
    if (wait_ms(500))
    {
      continue;     /* 被打斷：警報已接手控制 LED，所以這裡不要再動 LED */
    }
    led_set(LED_DIM);   /* 正常結束：LED 回到 10% */

    /* 輸出一行狀態訊息，內含系統運行時間 */
    format_time(tbuf, sizeof(tbuf), uptime_ticks / 10U);
    snprintf(out, sizeof(out), MSG_STATUS_FMT, tbuf);
    uart_print(out);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ==========================================================================
 *  按鈕中斷處理 (PB11，下緣觸發)
 * ========================================================================== */

/**
  * @brief  按鈕下緣中斷的實際處理。
  * @param  pin: 觸發中斷的腳位
  * @note   按鈕接 PB11 與 GND、內部上拉，按下時為低電位，所以用下緣觸發。
  *         ISR 內只做「防彈跳 + 設旗標」，不做串口輸出等耗時操作，
  *         真正的警報觸發由主迴圈的 service() 執行。
  */
static void button_isr(uint16_t pin)
{
  if (pin == GPIO_PIN_11)
  {
    uint32_t now = HAL_GetTick();

    /* 防彈跳：距離上次有效按下超過 50 ms 才算數，
     * 濾掉機械按鈕按下瞬間的連續抖動。 */
    if ((now - button_last_ms) > DEBOUNCE_MS)
    {
      button_last_ms = now;

      /* 警報中再按按鈕會被忽略：只有一般狀態才設旗標 */
      if (!alarm_active)
      {
        button_event = 1;
      }
    }
  }
}

/**
  * @brief  EXTI 下緣回呼函式（較新版本的 G4 HAL 會呼叫這個）。
  * @note   不同版本的 STM32G4 HAL 回呼函式名稱不同，
  *         下面兩個都提供，哪個被呼叫都能正常運作。
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  button_isr(GPIO_Pin);
}

/**
  * @brief  EXTI 通用回呼函式（較舊版本的 G4 HAL 會呼叫這個）。
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  button_isr(GPIO_Pin);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
