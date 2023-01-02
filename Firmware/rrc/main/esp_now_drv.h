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


void enc_send_now(float kp, float ki, float kd);

/**
 * @brief Send joystick data to peer
 * 
 * @param xVal X value in %
 * @param yVal Y value in %
 */
void joy_send_now(int xVal, int yVal);

/**
 * @brief 
 * 
 * @param kp 
 * @param ki 
 * @param kd 
 */
void reset_send_now(int kp, int ki, int kd);

struct __attribute__((__packed__))PID_data 
{
    float Kp;
    float Ki;
    float Kd;
};

#endif