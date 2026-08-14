/*
 * ESP32-WROOM-DA ESP-NOW 接收扬声器
 * 10 个逻辑频道、单按键切换版本
 *
 * Arduino ESP32 Core: 3.x（原程序建议 3.3.10）
 *
 * 重要说明：
 * 1. ESP-NOW / Wi-Fi 射频信道固定为 6。
 * 2. 频道 1～10 是“逻辑频道”。
 * 3. 只有逻辑频道相同的两台对讲机才会播放彼此的音频。
 * 4. 逻辑频道不是加密；不同频道的数据仍通过同一个射频信道发送，
 *    接收端只是在软件中丢弃频道不匹配的数据。
 *
 * 功放：MAX98357A
 *   DIN   -> GPIO14
 *   BCLK  -> GPIO27
 *   LRCLK -> GPIO26
 *   SD    -> GPIO23
 *
 * 按键（对地按下，使用内部上拉）：
 *   CHANNEL -> GPIO0
 *
 * 其他：
 *   WS2812B -> GPIO4
 *
 * 按键操作：
 *   每按一次 CHANNEL：1 -> 2 -> ... -> 10 -> 1
 *   切换后立即生效并保存，重新开机后恢复上次频道。
 *
 * 注意：GPIO0 是 ESP32 启动配置脚。正常运行时可以作为按键，
 * 但上电或复位时不要一直按住，否则可能进入下载模式。
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP_I2S.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

// ============================================================
// 引脚定义
// ============================================================

// MAX98357A
constexpr int SPK_DIN_PIN   = 14;
constexpr int SPK_BCLK_PIN  = 27;
constexpr int SPK_LRCLK_PIN = 26;
constexpr int SPK_SD_PIN    = 23;

// 频道按键
constexpr int CHANNEL_BUTTON_PIN = 0;

// 其他
constexpr int RGB_PIN = 4;

// ============================================================
// 工作参数
// ============================================================

constexpr unsigned long BUTTON_DEBOUNCE_MS = 25;
constexpr unsigned long RECEIVE_INDICATOR_MS = 140;

// 定期发送心跳，让原有带屏设备能够统计本机在线状态。
constexpr unsigned long DEVICE_HEARTBEAT_INTERVAL_MS = 1500;

constexpr uint32_t SAMPLE_RATE = 8000;
constexpr size_t AUDIO_SAMPLES_PER_PACKET = 100;

// 接收音量保持原程序的设置范围和默认值。
constexpr uint16_t SPEAKER_VOLUME_MIN_PERCENT = 0;
constexpr uint16_t SPEAKER_VOLUME_MAX_PERCENT = 200;
constexpr uint16_t DEFAULT_SPEAKER_VOLUME_PERCENT = 100;

// 实际 Wi-Fi / ESP-NOW 射频信道固定为 6。
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 6;

// 数据包过滤使用的逻辑频道。
constexpr uint8_t LOGICAL_CHANNEL_MIN = 1;
constexpr uint8_t LOGICAL_CHANNEL_MAX = 10;
constexpr uint8_t DEFAULT_LOGICAL_CHANNEL = 1;

// 防止其他 ESP-NOW 程序的数据包被误识别。
constexpr uint16_t WALKIE_NETWORK_ID = 0x4453; // "DS"

// ============================================================
// I2S 对象
// ============================================================

I2SClass speakerI2S(I2S_NUM_1);

// ============================================================
// WS2812
// ============================================================

Adafruit_NeoPixel statusLed(
    1,
    RGB_PIN,
    NEO_GRB + NEO_KHZ800
);

enum LedState : uint8_t {
  LED_IDLE,
  LED_RECEIVE,
  LED_ERROR
};

LedState currentLedState = LED_ERROR;

// ============================================================
// ESP-NOW 数据结构
// ============================================================

constexpr uint8_t PACKET_TYPE_AUDIO = 0xA1;
constexpr uint8_t PACKET_TYPE_PRESENCE = 0xA2;
constexpr uint8_t PACKET_VERSION = 2;

struct __attribute__((packed)) PacketHeader {
  uint8_t type;
  uint8_t version;
  uint8_t logicalChannel;
  uint8_t reserved;
  uint16_t networkId;
};

struct __attribute__((packed)) PresencePacket {
  uint8_t type;
  uint8_t version;
  uint8_t logicalChannel;
  uint8_t reserved;
  uint16_t networkId;
};

struct __attribute__((packed)) AudioPacket {
  uint8_t type;
  uint8_t version;
  uint8_t logicalChannel;
  uint8_t reserved;
  uint16_t networkId;
  uint16_t sequence;
  uint16_t sampleCount;
  int16_t samples[AUDIO_SAMPLES_PER_PACKET];
};

static_assert(
    sizeof(AudioPacket) <= 250,
    "AudioPacket exceeds ESP-NOW v1 packet limit"
);

static_assert(
    sizeof(PresencePacket) == sizeof(PacketHeader),
    "PresencePacket header layout mismatch"
);

const uint8_t broadcastAddress[6] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// ============================================================
// 接收队列
// ============================================================

constexpr size_t RX_QUEUE_LENGTH = 12;

struct RxQueueItem {
  uint16_t sequence;
  uint16_t sampleCount;
  int16_t samples[AUDIO_SAMPLES_PER_PACKET];
};

QueueHandle_t rxAudioQueue = nullptr;

// ============================================================
// 全局状态
// ============================================================

volatile uint8_t activeLogicalChannel = DEFAULT_LOGICAL_CHANNEL;
volatile unsigned long lastReceiveTime = 0;

uint8_t selectedLogicalChannel = DEFAULT_LOGICAL_CHANNEL;
uint16_t lastRxSequence = 0;
bool hasReceivedPacket = false;

unsigned long lastHeartbeatTime = 0;

volatile uint16_t speakerVolumePercent =
    DEFAULT_SPEAKER_VOLUME_PERCENT;

bool preferencesReady = false;
Preferences preferences;

// ============================================================
// 按键消抖
// ============================================================

struct DebouncedButton {
  int pin;
  bool stableState;
  bool previousReading;
  unsigned long changeTime;
};

DebouncedButton channelButton = {
  CHANNEL_BUTTON_PIN, HIGH, HIGH, 0
};

bool readButtonPressed(DebouncedButton &button) {
  const unsigned long now = millis();
  const bool reading = digitalRead(button.pin);

  if (reading != button.previousReading) {
    button.previousReading = reading;
    button.changeTime = now;
  }

  if ((now - button.changeTime) >= BUTTON_DEBOUNCE_MS &&
      reading != button.stableState) {
    button.stableState = reading;
    return button.stableState == LOW;
  }

  return false;
}

// ============================================================
// 工具函数
// ============================================================

void setLed(LedState state) {
  if (state == currentLedState) {
    return;
  }

  currentLedState = state;

  switch (state) {
    case LED_IDLE:
      statusLed.setPixelColor(0, statusLed.Color(0, 20, 0));
      break;

    case LED_RECEIVE:
      statusLed.setPixelColor(0, statusLed.Color(0, 0, 30));
      break;

    case LED_ERROR:
    default:
      statusLed.setPixelColor(0, statusLed.Color(30, 0, 30));
      break;
  }

  statusLed.show();
}

int16_t clampToInt16(int32_t value) {
  if (value > 32767) {
    return 32767;
  }

  if (value < -32768) {
    return -32768;
  }

  return static_cast<int16_t>(value);
}

void printMacAddress() {
  uint8_t mac[6] = {0};

  esp_wifi_get_mac(WIFI_IF_STA, mac);

  Serial.printf(
      "本机 STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      mac[0], mac[1], mac[2],
      mac[3], mac[4], mac[5]
  );
}

void sendPresenceHeartbeat() {
  const unsigned long now = millis();

  if (lastHeartbeatTime != 0 &&
      now - lastHeartbeatTime < DEVICE_HEARTBEAT_INTERVAL_MS) {
    return;
  }

  lastHeartbeatTime = now;

  PresencePacket packet;
  packet.type = PACKET_TYPE_PRESENCE;
  packet.version = PACKET_VERSION;
  packet.logicalChannel = activeLogicalChannel;
  packet.reserved = 0;
  packet.networkId = WALKIE_NETWORK_ID;

  esp_now_send(
      broadcastAddress,
      reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet)
  );
}

void clearReceiveQueue() {
  if (rxAudioQueue == nullptr) {
    return;
  }

  RxQueueItem discarded;

  while (xQueueReceive(rxAudioQueue, &discarded, 0) == pdTRUE) {
    // 清空旧数据
  }
}

bool isValidLogicalChannel(uint8_t channel) {
  return channel >= LOGICAL_CHANNEL_MIN &&
         channel <= LOGICAL_CHANNEL_MAX;
}

uint8_t nextLogicalChannel(uint8_t channel) {
  if (channel >= LOGICAL_CHANNEL_MAX) {
    return LOGICAL_CHANNEL_MIN;
  }

  return channel + 1;
}

// ============================================================
// 频道保存与切换
// ============================================================

void loadSavedLogicalChannel() {
  preferencesReady = preferences.begin("dstike-radio", false);

  uint8_t savedChannel = DEFAULT_LOGICAL_CHANNEL;

  if (preferencesReady) {
    savedChannel = preferences.getUChar(
        "channel",
        DEFAULT_LOGICAL_CHANNEL
    );
  } else {
    Serial.println("Preferences 初始化失败，使用默认频道");
  }

  if (!isValidLogicalChannel(savedChannel)) {
    savedChannel = DEFAULT_LOGICAL_CHANNEL;
  }

  uint16_t savedVolume = DEFAULT_SPEAKER_VOLUME_PERCENT;

  if (preferencesReady) {
    savedVolume = preferences.getUShort(
        "spkVolume",
        DEFAULT_SPEAKER_VOLUME_PERCENT
    );
  }

  if (savedVolume < SPEAKER_VOLUME_MIN_PERCENT ||
      savedVolume > SPEAKER_VOLUME_MAX_PERCENT) {
    savedVolume = DEFAULT_SPEAKER_VOLUME_PERCENT;
  }

  activeLogicalChannel = savedChannel;
  selectedLogicalChannel = savedChannel;
  speakerVolumePercent = savedVolume;

  Serial.printf("当前逻辑频道：%u\n", savedChannel);
  Serial.printf("当前喇叭音量：%u%%\n", savedVolume);
}

void confirmLogicalChannel() {
  if (!isValidLogicalChannel(selectedLogicalChannel)) {
    selectedLogicalChannel = DEFAULT_LOGICAL_CHANNEL;
  }

  const uint8_t oldChannel = activeLogicalChannel;
  activeLogicalChannel = selectedLogicalChannel;

  // 切换频道后清掉旧频道残留音频和序号状态。
  clearReceiveQueue();
  hasReceivedPacket = false;
  lastRxSequence = 0;
  lastReceiveTime = 0;
  lastHeartbeatTime = 0;

  if (preferencesReady && oldChannel != activeLogicalChannel) {
    preferences.putUChar("channel", activeLogicalChannel);
  }

  Serial.printf(
      "逻辑频道已确认：%u（Wi-Fi 射频信道仍固定为 %u）\n",
      activeLogicalChannel,
      ESPNOW_WIFI_CHANNEL
  );
}

void updateChannelButton() {
  if (!readButtonPressed(channelButton)) {
    return;
  }

  selectedLogicalChannel =
      nextLogicalChannel(activeLogicalChannel);
  confirmLogicalChannel();
}

// ============================================================
// ESP-NOW 回调
// ============================================================

void onDataReceived(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int length
) {
  if (info == nullptr ||
      data == nullptr ||
      length < static_cast<int>(sizeof(PacketHeader))) {
    return;
  }

  PacketHeader header;
  memcpy(&header, data, sizeof(header));

  if (header.version != PACKET_VERSION ||
      header.networkId != WALKIE_NETWORK_ID ||
      !isValidLogicalChannel(header.logicalChannel)) {
    return;
  }

  if (header.type == PACKET_TYPE_PRESENCE) {
    if (length < static_cast<int>(sizeof(PresencePacket))) {
      return;
    }

    return;
  }

  if (header.type != PACKET_TYPE_AUDIO ||
      length < static_cast<int>(offsetof(AudioPacket, samples)) ||
      length > static_cast<int>(sizeof(AudioPacket))) {
    return;
  }

  AudioPacket packet;
  memset(&packet, 0, sizeof(packet));
  memcpy(&packet, data, length);

  if (packet.sampleCount == 0 ||
      packet.sampleCount > AUDIO_SAMPLES_PER_PACKET) {
    return;
  }

  const size_t expectedLength =
      offsetof(AudioPacket, samples) +
      packet.sampleCount * sizeof(int16_t);

  if (static_cast<size_t>(length) < expectedLength) {
    return;
  }

  // 逻辑频道不同，不播放音频。
  const uint8_t localChannel = activeLogicalChannel;

  if (packet.logicalChannel != localChannel) {
    return;
  }

  RxQueueItem queueItem;
  queueItem.sequence = packet.sequence;
  queueItem.sampleCount = packet.sampleCount;

  memcpy(
      queueItem.samples,
      packet.samples,
      packet.sampleCount * sizeof(int16_t)
  );

  // 回调运行于 Wi-Fi 任务，只负责放入队列。
  // 队列满时丢掉最旧数据，避免延迟不断增加。
  if (xQueueSend(rxAudioQueue, &queueItem, 0) != pdTRUE) {
    RxQueueItem oldItem;
    xQueueReceive(rxAudioQueue, &oldItem, 0);
    xQueueSend(rxAudioQueue, &queueItem, 0);
  }

  lastReceiveTime = millis();
}

bool initializeSpeaker() {
  speakerI2S.setPins(
      SPK_BCLK_PIN,
      SPK_LRCLK_PIN,
      SPK_DIN_PIN,
      -1,
      -1
  );

  const bool result = speakerI2S.begin(
      I2S_MODE_STD,
      SAMPLE_RATE,
      I2S_DATA_BIT_WIDTH_16BIT,
      I2S_SLOT_MODE_MONO,
      I2S_STD_SLOT_LEFT,
      I2S_ROLE_MASTER
  );

  if (!result) {
    Serial.println("MAX98357A I2S 初始化失败");
    return false;
  }

  Serial.println("MAX98357A I2S 初始化成功");
  return true;
}

// ============================================================
// ESP-NOW 初始化
// ============================================================

bool initializeEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_ps(WIFI_PS_NONE);

  const esp_err_t channelResult = esp_wifi_set_channel(
      ESPNOW_WIFI_CHANNEL,
      WIFI_SECOND_CHAN_NONE
  );

  if (channelResult != ESP_OK) {
    Serial.printf(
        "设置 Wi-Fi 信道失败，错误码：%d\n",
        static_cast<int>(channelResult)
    );
    return false;
  }

  printMacAddress();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 初始化失败");
    return false;
  }

  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));

  memcpy(
      peerInfo.peer_addr,
      broadcastAddress,
      sizeof(broadcastAddress)
  );

  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(broadcastAddress)) {
    const esp_err_t result = esp_now_add_peer(&peerInfo);

    if (result != ESP_OK) {
      Serial.printf(
          "添加广播 Peer 失败，错误码：%d\n",
          static_cast<int>(result)
      );
      return false;
    }
  }

  Serial.printf(
      "ESP-NOW 初始化成功，Wi-Fi 射频信道固定为：%u\n",
      ESPNOW_WIFI_CHANNEL
  );

  return true;
}

// ============================================================
// 音频播放任务
// ============================================================

void audioPlaybackTask(void *parameter) {
  (void)parameter;

  RxQueueItem item;
  int16_t playbackSamples[AUDIO_SAMPLES_PER_PACKET];

  while (true) {
    if (xQueueReceive(
            rxAudioQueue,
            &item,
            pdMS_TO_TICKS(30)
        ) == pdTRUE) {

      if (hasReceivedPacket) {
        const uint16_t expected =
            static_cast<uint16_t>(lastRxSequence + 1);

        if (item.sequence != expected) {
          Serial.printf(
              "检测到音频丢包，期望=%u，收到=%u\n",
              expected,
              item.sequence
          );
        }
      }

      lastRxSequence = item.sequence;
      hasReceivedPacket = true;

      for (size_t i = 0; i < item.sampleCount; i++) {
        int32_t sample = item.samples[i];
        const uint16_t currentVolume = speakerVolumePercent;
        sample = sample * currentVolume / 100;
        playbackSamples[i] = clampToInt16(sample);
      }

      setLed(LED_RECEIVE);

      speakerI2S.write(
          reinterpret_cast<const uint8_t *>(playbackSamples),
          item.sampleCount * sizeof(int16_t)
      );
    }

    const unsigned long receiveTime = lastReceiveTime;

    if (receiveTime == 0 ||
        millis() - receiveTime > RECEIVE_INDICATOR_MS) {
      setLed(LED_IDLE);
    }

    vTaskDelay(1);
  }
}

// ============================================================
// Arduino setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=================================");
  Serial.println("ESP32 ESP-NOW 10频道接收扬声器");
  Serial.println("=================================");

  pinMode(CHANNEL_BUTTON_PIN, INPUT_PULLUP);

  // 读取按键初始电平，避免开机后产生一次假按键事件。
  channelButton.stableState = digitalRead(CHANNEL_BUTTON_PIN);
  channelButton.previousReading = channelButton.stableState;

  pinMode(SPK_SD_PIN, OUTPUT);
  digitalWrite(SPK_SD_PIN, HIGH);

  statusLed.begin();
  statusLed.clear();
  statusLed.show();
  setLed(LED_ERROR);

  loadSavedLogicalChannel();

  rxAudioQueue = xQueueCreate(
      RX_QUEUE_LENGTH,
      sizeof(RxQueueItem)
  );

  if (rxAudioQueue == nullptr) {
    Serial.println("创建接收队列失败");

    while (true) {
      setLed(LED_ERROR);
      delay(500);
    }
  }

  const bool speakerOk = initializeSpeaker();
  const bool espNowOk = initializeEspNow();

  if (!speakerOk || !espNowOk) {
    Serial.println("初始化失败，请检查串口提示");
    setLed(LED_ERROR);

    while (true) {
      delay(1000);
    }
  }

  const BaseType_t taskResult = xTaskCreatePinnedToCore(
      audioPlaybackTask,
      "AudioPlayback",
      4096,
      nullptr,
      3,
      nullptr,
      1
  );

  if (taskResult != pdPASS) {
    Serial.println("创建播放任务失败");

    while (true) {
      setLed(LED_ERROR);
      delay(1000);
    }
  }

  setLed(LED_IDLE);

  Serial.println("系统准备完成");
  Serial.println("每按一次 GPIO0 按键，切换到下一个逻辑频道");
  Serial.println("频道顺序：1 -> 2 -> ... -> 10 -> 1");
}

void loop() {
  updateChannelButton();
  sendPresenceHeartbeat();
  delay(1);
}
