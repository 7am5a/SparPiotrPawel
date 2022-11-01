#ifndef ESP_NOW_DRV
#define ESP_NOW_DRV

/**
 * @brief Receive MAC address from chip
 * 
 * @return char* Received and print MAC addres
 */
char *mac_to_str();

/**
 * @brief Callback function required to make possible to send data
 * 
 */
void on_sent();

/**
 * @brief Callback function required to make possible to receive data
 * 
 */
void on_receive();

/**
 * @brief Initialize and set up WiFi drivers and then  esp-now drivers
 * 
 */
void init_esp_now();

//optionaly send flag as params - value will be stay as it is 
/**
 * @brief Send data to peer
 * 
 * @param param1 - name of first parameter
 * @param value1 - value of first parameter
 * @param param2 - name of second parameter - if exist
 * @param value2 - value of second parameter - if exist
 */
void send_now_task(int16_t param1, int16_t value1, int16_t param2, int16_t value2);

/**
 * @brief Set the send now task object
 * 
 */
void set_send_now_task();

#endif