#ifndef ESP_NOW_DRV
#define ESP_NOW_DRV

struct State_feedback_vals
{
    float K1 = 0.0f;
    float K2 = 20.2f;
    float K3 = 0.0f;
    float K4 = 1.7f;
};

struct PID_vals
{
    float Kp;
    float Ki;
    float Kd;
};

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

/**
 * @brief Send encoder data to peer
 * 
 * @param param Parameter prefix
 * @param paramVal Current value of encoder
 */
void enc_send_now(char param[2], int paramVal);

/**
 * @brief Send joystick data to peer
 * 
 * @param xVal X value in %
 * @param yVal Y value in %
 */
void joy_send_now(int xVal, int yVal);

#endif