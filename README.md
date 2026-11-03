# SOS 緊急警報系統（STM32 + 兩步驟驗證 + 串口遠端監控）

這是一個跑在 STM32G474re 開發板上的小型安全監控程式。平常 LED 以低亮度待機，並定期從串口回報系統狀態；按下實體按鈕就進入 SOS 警報，LED 與蜂鳴器依摩斯密碼「三短、三長、三短」循環；要解除警報，必須在串口完成兩步驟驗證（先輸入授權碼，再於 5 秒內輸入確認字元）。系統會記住最近 3 次警報，並可用串口指令查詢統計或遠端測試 LED 與蜂鳴器。


---

## 硬體與開發環境

開發板使用 STM32G4 系列（NUCLEO-G474RE），以 STM32CubeIDE 加 HAL 函式庫開發，程式碼由 CubeMX 產生框架，自訂內容只寫在 `USER CODE` 區塊內。

需要的零件與接法：

- **LED**：1 顆，接在 TIM2 通道 1 的 PWM 輸出腳（Nucleo 板上通常是 PA5 的 LD2，實際腳位以 CubeMX 分配為準）
- **蜂鳴器**：1 個主動式蜂鳴器，接 PC8；高電位響、低電位靜音，只做開關，不調頻率
- **按鈕**：1 個，接 PB11 與 GND，使用內部上拉，按下時為低電位
- **串口**：LPUART1，TX 為 PA2、RX 為 PA3（Nucleo 板上通常已接到 ST-LINK 的虛擬串口）
- **電腦端**：任一串口終端機（Tera Term、PuTTY、minicom 皆可），設定 115200、8 data bits、無同位檢察、1 stop bit；送出指令時按 Enter（CR、LF、CRLF 都可以）

## 周邊設定

**系統時脈**：內部 HSI 16 MHz 為來源，經 PLL（M 除 1、N 乘 20、R 除 2）得到 160 MHz 系統時脈；AHB、APB1、APB2 都不分頻。

**LPUART1**：115200 bps、8N1、收發皆啟用、無硬體流控，FIFO 模式關閉。啟用 LPUART1 的全域中斷，並以中斷方式「一次收 1 個字元」。

**TIM2（LED 亮度）**：內部時脈，預除頻 159、自動重載值 999，所以 PWM 頻率約 1 kHz。通道 1 為 PWM 模式 1、極性為高。亮度只用三個比較值：0 代表熄滅、100 代表約 10%（待機亮度）、999 代表全亮。

**TIM3（系統節拍）**：內部時脈，預除頻 15999、自動重載值 999，所以每 100 ms 產生一次更新中斷。啟用 TIM3 的全域中斷。運行時間、警報節奏、驗證倒數都靠這個節拍。

**PB11（按鈕）**：外部中斷，下降緣觸發，內部上拉。啟用 EXTI line [15:10] 中斷，優先權 Preemption 1、Sub 0。

**PC8（蜂鳴器）**：推挽輸出、不上下拉、低速，初始為低電位。

---

## 功能說明

### 開機自我檢測

開機後先初始化周邊，接著由串口輸出自我檢測開始的訊息。LED 先全亮約 1.5 秒再降回 10%；然後蜂鳴器響 1.5 秒、停 1.5 秒，連續做兩次。最後輸出檢測通過的訊息（內容會寫 LED、蜂鳴器、EXTI 都 OK，但程式其實沒有真的檢測按鈕，只是固定文字）。整個自我檢測約 7.5 秒。檢測完成後才啟動 100 ms 節拍，所以運行時間從這時才開始累計，之後輸出歡迎訊息並提示可用指令。

### 一般狀態

LED 維持 10% 亮度，蜂鳴器不響。主迴圈每圈約 5 秒：先等 4.5 秒（每 100 ms 檢查一次是否被按鈕打斷，是就立刻離開），接著 LED 全亮 0.5 秒（同樣可被打斷），再回到 10%，最後從串口輸出一行狀態訊息，內含系統已運行時間。時間格式固定為「兩位數時、分、秒」，例如 `00h 05m 12s`。

### SOS 警報

警報只會在一般狀態下被按鈕觸發；警報中再按按鈕會被忽略。觸發當下會做這幾件事：總觸發次數加 1、記下當下的系統運行秒數作為這次警報的起始時間、開始計算警報持續時間、把驗證流程重設回第一步，並從串口輸出警報觸發訊息，提示需要輸入授權碼。

警報期間，LED 與蜂鳴器同步依摩斯密碼 SOS 的節奏動作。以 100 ms 為 1 個單位，一個循環共 35 個單位（3.5 秒），從循環開始算起：

- 第 0–2、3–5、6–8 個單位為「短」（各 0.2 秒），代表 S
- 第 9–12、13–16、17–20 個單位為「長」（各 0.3 秒），代表 O
- 第 21–23、24–26、27–29 個單位為「短」（各 0.2 秒），代表 S
- 其餘時間為休止（含最後 0.6 秒）

（區間皆為含起點、不含終點。）亮的時段 LED 全亮、蜂鳴器響；其餘時段 LED 熄滅、蜂鳴器靜音。警報期間主迴圈每隔約 5 秒會從串口輸出一次「警報持續中，等待 2FA 解除」的提示。

### 兩步驟驗證解除

第一步，使用者要輸入 `clear 1234`。授權碼是寫死的 1234；輸入錯誤的數字（或非數字）會回報拒絕存取，輸入其他任何內容會提醒先輸入授權碼，兩者都停留在第一步。

第二步，授權碼正確後系統給 5 秒時間，必須在 5 秒內輸入 `r` 或 `R`。輸入其他內容會提示輸入錯誤，但仍留在第二步，倒數繼續。如果 5 秒內沒有輸入，系統自動退回第一步並提示時間已過，使用者需重新輸入授權碼。

驗證成功時：警報解除、驗證回到第一步、LED 回到 10% 亮度、蜂鳴器關閉；警報持續時間以「累計的 100 ms 單位數除以 10」取整數秒；把「起始時間與持續時間」存進歷史紀錄；最後輸出解除成功訊息與持續時間。

### 歷史紀錄

只保留最近 3 次，每筆含「觸發時的運行時間」與「警報持續時間」。新紀錄放在最前面，舊的依序往後移，超過 3 筆時最舊的被擠掉。

### 指令（僅一般狀態可用）

- `stats`：輸出統計報表，包含總觸發次數、目前運行時間，以及最近 3 次警報的編號、觸發時間、持續時間（編號 1 為最新）；還沒有紀錄時顯示「尚無警報紀錄」
- `cmd test`：先輸出開始測試的訊息，然後 LED 全亮並蜂鳴器響 200 ms、再恢復並靜音 200 ms，共做兩次，最後輸出測試完成的訊息
- 其他任何內容：輸出目前為正常狀態與可用指令清單

指令必須完全相符，區分大小寫（`cmd test` 中間是一個空格）。

### 串口接收規則

每收到一個字元就存進緩衝區；收到 `\r` 或 `\n` 且緩衝區不是空的，才把整行交給指令解析並清空緩衝區（因此 CRLF 的第二個字元會被自然忽略）。緩衝區大小為 32 位元組，最多存 31 個字元，超過時整行丟棄、重新開始。

---

## 窗口訊息（請逐字照用）

下列字串中的 `\r\n` 代表回車加換行，`%s` 為時間字串（格式如 `00h 05m 12s`）。訊息都是英文，並帶分類標籤。

```
開機檢測開始：   \r\n[POST] Starting Hardware Self-Test...\r\n
開機檢測通過：   [POST] Hardware Self-Test PASSED: LED: OK | Buzzer: OK | EXTI: OK\r\n
歡迎訊息：       \r\n[SYSTEM] Secure Remote Monitoring Initialized.\r\nCommands: 'stats', 'cmd test', or alarm prompt for 2FA unlock.\r\n
一般狀態回報：   \n[STATUS] System: NORMAL | Uptime: %s | Status: SECURE\r\n
警報觸發：       \r\n[ALARM] SOS Triggered! SECURE AUTH REQUIRED.\r\n-> Type: clear <auth_code>\r\n
警報持續提示：   \n\n[ALARM] System: SOS ACTIVE! | Waiting for 2FA clearance...\r\n
第一步通過：     \r\n[SECURITY] Step 1 Passed! Enter 'r' or 'R' within 5 seconds to clear SOS...\r\n
授權碼錯誤：     \r\n[ERROR] Invalid Auth Code! Access Denied.\r\n
警報中輸入其他： \r\n[WARNING] Alarm Active. Please enter internal auth code first.\r\n
第二步輸入錯誤： \r\n[ERROR] Incorrect input! Expected 'r' or 'R'.\r\n
5 秒視窗過期：   \r\n[WARNING] 5-Second Window Expired! Please re-enter internal auth code.\r\n
解除成功：       \r\n[INFO] 2FA Verified Successfully! SOS Cleared. (Duration: %s)\r\n
測試開始：       \r\n[CMD] Executing remote test: LED blink x2 & Buzzer beep...\r\n
測試結束：       [CMD] Test sequence completed successfully.\r\n
未知指令：       \r\n[INFO] Normal state. Available commands: stats, cmd test\r\n
```

另有一則「倒數已歸零才輸入 r」的訊息：`\r\n[ERROR] 5-Second Timeout Expired! Verification Reset.\r\n`。實際上倒數歸零時系統會先把驗證退回第一步，所以這則幾乎不會出現，可保留也可省略。

`stats` 的輸出格式如下（前面有一個空行；範例數字僅供參考）：

```
========================================
       SYSTEM TELEMETRY & STATS         
========================================
- Total SOS Triggers    : 2
- System Uptime         : 00h 05m 12s
----------------------------------------
 Recent SOS History (Max 3):
 [1] Triggered @ 00h 04m 10s | Duration: 00h 00m 25s
 [2] Triggered @ 00h 01m 03s | Duration: 00h 00m 12s
========================================
```

沒有紀錄時，歷史區那一行改為 ` (No alarm history recorded yet)`（開頭有一個空格）。

---

## 重現

**手動：** 在 CubeIDE 新增專案並選好板子，依「周邊設定」在 `.ioc` 設好時脈、LPUART1、TIM2、TIM3、PB11、PC8 與三個中斷，產生程式碼後，依「功能說明」把邏輯寫進 `main.c` 的 `USER CODE` 區塊（會用到 UART 接收完成、外部中斷、定時器溢位三個回呼函式）。

**交給 AI：** 把這份 README 整份貼過去，再加上這段話即可：

> 請依照上面的規格，為 STM32G4（STM32CubeIDE + HAL）產生完整可編譯的 main.c。自訂程式碼放在 USER CODE 區塊內，串口字串請逐字照抄，註解使用繁體中文。最後列出 CubeMX 中需要確認的設定。

**驗收：**

1. 重設後應看到 LED 亮 1.5 秒、蜂鳴器響兩次，串口依序出現檢測開始、檢測通過、歡迎訊息
2. 之後約每 5 秒 LED 閃一下，串口出現一行狀態訊息
3. 輸入 `stats` 與 `cmd test`，結果符合上面的說明
4. 按下按鈕後，LED 與蜂鳴器依 SOS 節奏動作；輸入 `clear 1234` 後 5 秒內輸入 `r` 可解除，不輸入則 5 秒後退回第一步
5. 解除後再輸入 `stats`，應看到剛才那次的紀錄；連續觸發超過 3 次，只會留下最近 3 筆
