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


void enc_state_based_send_now(float k1, float k2, float k3);

void enc_pid_send_now(float kp, float ki, float kd);

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
 * @param k1 
 * @param k2 
 * @param k3 
 * @param kp 
 * @param ki 
 * @param kd 
 */
void reset_send_now(float k1, float k2, float k3, float kp, float ki, float kd);

struct __attribute__((__packed__))State_Based_data 
{
    float K1;
    float K2;
    float K3;
};

struct __attribute__((__packed__))PID_data 
{
    float Kp;
    float Ki;
    float Kd;
};

#endif