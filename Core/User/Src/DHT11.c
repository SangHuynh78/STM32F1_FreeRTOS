#include "DHT11.h"
#include "i2c-lcd.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim2;
void HAL_DelayMicroseconds(uint16_t us);

void DHT11_Start(void)
{
    // Set pin as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

    // Pull pin low for 18ms
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);

    // Pull pin high for 20-40µs
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    HAL_DelayMicroseconds(20);
}

uint8_t DHT11_Check_Response(void)
{
    uint8_t response = 0;

    // Set pin as input
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

    // Wait for DHT11 response (low for 80us, then high for 80us)
    if (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
    {
        HAL_DelayMicroseconds(80);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
            response = 1;
        HAL_DelayMicroseconds(80);
    }

    return response;
}

uint8_t DHT11_Read_Byte(void)
{
    uint8_t i, byte = 0;
    for (i = 0; i < 8; i++)
    {
        // Wait for pin to go high
        while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));

        HAL_DelayMicroseconds(30); // Wait 30µs to determine the bit value

        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
            byte |= (1 << (7 - i)); // If pin is high after 30µs, bit is 1

        // Wait for pin to go low
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
    }
    return byte;
}

void DHT11_Read_Data(float *temperature, float *humidity)
{
    uint8_t rh_int, rh_dec, temp_int, temp_dec, checksum;

    DHT11_Start();
    if (DHT11_Check_Response())
    {
        rh_int = DHT11_Read_Byte();
        rh_dec = DHT11_Read_Byte();
        temp_int = DHT11_Read_Byte();
        temp_dec = DHT11_Read_Byte();
        checksum = DHT11_Read_Byte();

        // Verify checksum
        if (checksum == (rh_int + rh_dec + temp_int + temp_dec))
        {
            *humidity = (float)rh_int;
            *temperature = (float)temp_int;
        }
    }
}

void Display_Temp (float Temp)
{

    char str[20] = "TE:";
    int index = 3;
    int integerPart = (int)Temp;

    lcd_put_cur(1, 2);

    // Add integer part
    if (integerPart == 0) {
        str[index++] = '0';
    } else {
        if (integerPart >= 10) {
            str[index++] = '0' + (integerPart / 10);
        }
        str[index++] = '0' + (integerPart % 10);
    }

    str[index] = '\0';
    lcd_send_string(str);
    lcd_send_data('C');
}

void Display_Rh(float Rh)
{
    char str[20] = "HU:";
    int index = 3;
    int integerPart = (int)Rh;

    lcd_put_cur(1, 9);
    // Add integer part
    if (integerPart == 0) {
        str[index++] = '0';
    } else {
        if (integerPart >= 10) {
            str[index++] = '0' + (integerPart / 10);
        }
        str[index++] = '0' + (integerPart % 10);
    }
    // Null-terminate the string
    str[index] = '\0';
    lcd_send_string(str);
    lcd_send_data('%');
}

void HAL_DelayMicroseconds(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset counter
    while (__HAL_TIM_GET_COUNTER(&htim2) < us); // Đợi cho đến khi đạt đến giá trị cần
}
