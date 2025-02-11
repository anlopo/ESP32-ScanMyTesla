/*
ESP32-ScanMyTesla 2.0.0 (2024)
Author: E.Burkowski
GENERAL PUBLIC LICENSE
*/

#include "driver/twai.h"
#include <BluetoothSerial.h>

#define BUFFER_LENGTH 16 // Play with this size to get better packets/second

#define RX_PIN 16
#define TX_PIN 17

//#define DEBUG  // Serial debug output

uint8_t messageCounter = 0; // should not be greater than BUFFER_LENGTH

bool ids[2048]; // save here ID that we care about.
uint16_t canDataBufferId[BUFFER_LENGTH];
uint8_t canDataBufferLength[BUFFER_LENGTH];
uint8_t canDataBufferData[BUFFER_LENGTH][8];

bool noFilter = true;
char btBufferCounter = 0;
char buffer[128];

BluetoothSerial SerialBT;

#ifdef DEBUG
void printFrame(twai_message_t &canMessage)
{
  Serial.printf("%03x: ", canMessage.identifier);
  for (uint8_t i = 0; i < canMessage.data_length_code; i++)
  {
    Serial.printf("%02x ", canMessage.data[i]);
  }
  Serial.println();
}

#define debug_println(...) Serial.println(__VA_ARGS__)
#define debug_print(...) Serial.print(__VA_ARGS__)
#define debug_printf(...) Serial.printf(__VA_ARGS__)
#else
#define printFrame(...)
#define debug_println(...)
#define debug_print(...)
#define debug_printf(...)
#endif

void processCanMessage(twai_message_t &canMessage)
{
  uint8_t length = canMessage.data_length_code;

  // If the ID is not in the list, exit immediately.
  if (!ids[canMessage.identifier])
    return;

  // Verifying the correct length
  if (length == 0 || length > 8)
  {
    debug_println("Message dropped, wrong length");
    return;
  }

  // Using modulo for optimized buffer indexing
  uint8_t lineNumber = canMessage.identifier % BUFFER_LENGTH;

  // Check for duplicates on the same index
  if (canDataBufferId[lineNumber] == canMessage.identifier && canDataBufferData[lineNumber][0] == canMessage.data[0])
  {
    debug_print("ID ");
    debug_print(canMessage.identifier);
    debug_println(" already in buffer");
    return;
  }

  // Saving data to the buffer
  canDataBufferId[lineNumber] = canMessage.identifier;
  canDataBufferLength[lineNumber] = length;
  memcpy(canDataBufferData[lineNumber], canMessage.data, length);

  // Message index shift
  messageCounter = (messageCounter + 1) % BUFFER_LENGTH;
}

bool canInit()
{
  // Initialize configuration structures using macro initializers
  // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
  twai_general_config_t g_config = {.mode = TWAI_MODE_LISTEN_ONLY,
                                    .tx_io = (gpio_num_t)TX_PIN,
                                    .rx_io = (gpio_num_t)RX_PIN,
                                    .clkout_io = TWAI_IO_UNUSED,
                                    .bus_off_io = TWAI_IO_UNUSED,
                                    .tx_queue_len = 1,
                                    .rx_queue_len = 127,
                                    .alerts_enabled = TWAI_ALERT_ALL,
                                    .clkout_divider = 0};
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    debug_println("Driver installed");
  }
  else
  {
    debug_println("Failed to install driver");
    return false;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK)
  {
    debug_println("Driver started");
  }
  else
  {
    debug_println("Failed to start driver");
    return false;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
  {
    debug_println("CAN Alerts reconfigured");
  }
  else
  {
    debug_println("Failed to reconfigure alerts");
    return false;
  }

  return true;
}

void setup()
{
#ifdef DEBUG
  delay(1000);
  Serial.begin(250000);
  debug_println("\nESP_SMT init");
#endif

  while (!canInit())
  {
    debug_println("Retrying in 1s");
    delay(1000);
  }

  SerialBT.begin("ESP-SMT");

  noFilter = false; // There are some filters
  // Set true for all IDs, because no filters applied yet
  memset(ids, true, sizeof(ids));
  memset(canDataBufferId, 0, sizeof(canDataBufferId) / sizeof(canDataBufferId[0]));
  memset(canDataBufferLength, 0, sizeof(canDataBufferLength) / sizeof(canDataBufferLength[0]));
  memset(canDataBufferData, 0, sizeof(canDataBufferData) / sizeof(canDataBufferData[0][0]));
}

void canLoop()
{
  uint32_t alerts_triggered;
  if (twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1)) != ESP_OK)
    return; // If there are no alerts, quit

  twai_status_info_t twaistatus;

  // Alert processing
  if (alerts_triggered)
  {
    twai_get_status_info(&twaistatus); // Load status only once if there are any alerts

    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
    {
      debug_println("Alert: TWAI controller is error passive.");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    {
      debug_printf("Alert: Bus error occurred. Count: %d\n", twaistatus.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL)
    {
      debug_printf("Alert: RX queue full! Buffered: %d, Missed: %d, Overrun: %d\n",
                   twaistatus.msgs_to_rx, twaistatus.rx_missed_count, twaistatus.rx_overrun_count);
    }
  }

  // Processing received CAN messages
  if (alerts_triggered & TWAI_ALERT_RX_DATA)
  {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK)
    {
      processCanMessage(message);
    }
  }
}

void processSmtCommands(char *smtCmd, char *returnToSmt)
{
  returnToSmt[0] = '\0'; // Empty buffer before use

  debug_print("smtCmd: ");
  debug_println(smtCmd);

  // Data polling
  if (!strncmp(smtCmd, "atma", 4) || !strncmp(smtCmd, "stm", 3))
  {
    for (uint8_t i = 0; i < BUFFER_LENGTH; i++)
    {
      if (canDataBufferId[i] == 0)
        continue;

      char tempBuffer[32];
      snprintf(tempBuffer, sizeof(tempBuffer), "%03X", canDataBufferId[i]);
      strcat(returnToSmt, tempBuffer);

      for (uint8_t l = 0; l < canDataBufferLength[i]; l++)
      {
        snprintf(tempBuffer, sizeof(tempBuffer), "%02X", canDataBufferData[i][l]);
        strcat(returnToSmt, tempBuffer);
      }

      canDataBufferId[i] = 0; // Setting ID to zero to ignore on next cycle
      strcat(returnToSmt, "\n");
    }
  }
  // Nastavenie filtrov
  else if (!strncmp(smtCmd, "stfap ", 6))
  {
    uint16_t filter = strtol(smtCmd + 6, NULL, 16); // Direct HEX parsing

    if (noFilter)
    {
      memset(ids, false, sizeof(ids)); // Remove filters and set only one
      noFilter = false;
    }

    debug_print("New filter from SMT: ");
    debug_println(filter);

    if (filter < 2048)
    {
      ids[filter] = true;
      strcat(returnToSmt, "OK\n");
    }
    else
    {
      strcat(returnToSmt, "ERR: Invalid ID\n");
    }
  }
  // Remove filters
  else if (!strncmp(smtCmd, "stfcp", 5))
  {
    memset(ids, true, sizeof(ids)); // Allow all IDs
    noFilter = true;

    debug_println("Clear all filters = allow all IDs!");
    strcat(returnToSmt, "OK\n");
  }
  // All others commands
  else
  {
    strcat(returnToSmt, "OK\n");
  }

  strcat(returnToSmt, ">\n"); // Indicate "waiting for command"
}

void processBtMessage()
{
  char responseToBt[332]; // Buffer for response
  processSmtCommands(buffer, responseToBt);

  debug_println("BT out message: ");
  debug_println(responseToBt);

  SerialBT.print(responseToBt);
}

void btLoop()
{
  while (SerialBT.available())
  {
    char tmp = SerialBT.read();

    // If "Carriage Return" or buffer is full, process message
    if (tmp == 13 || btBufferCounter >= sizeof(buffer) - 2)
    {
      buffer[btBufferCounter] = '\0'; // Null terminate string
      btBufferCounter = 0;            // Buffer reset
      processBtMessage();
      continue;
    }

    // Ignore "New Line" (10) and "Space" (32)
    if (tmp != 10 && tmp != 32)
    {
      buffer[btBufferCounter++] = tolower(tmp);
    }
  }
}

void loop()
{
  canLoop();
  btLoop();
}