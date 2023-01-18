#pragma once


class Encoder {
public:
	Encoder(gpio_num_t gpioPinA, gpio_num_t gpioPinB, pcnt_unit_t pcntUnit);
	void init();
	int getValue() const;
	void clear_pcnt();
protected:
	int32_t getAddition() const;
	gpio_num_t gpioPinA, gpioPinB;
	pcnt_unit_t pcntUnit;
	uint8_t startingOffset;
};

/**
 * @brief 
 * 
 * @param pvParameter 
 */
void encoder_task(void *pvParameter);

/**
 * @brief 
 * 
 * @return float 
 */
float encoder_get_speed(uint8_t motor);

// /**
//  * @brief 
//  * 
//  */
// float encoder_get_radian(uint8_t motor);