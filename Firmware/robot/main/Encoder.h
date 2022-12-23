#pragma once


class Encoder {
public:
	Encoder(gpio_num_t gpioPinA, gpio_num_t gpioPinB, pcnt_unit_t pcntUnit = PCNT_UNIT_0);
	void init();
	int getValue() const;
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