#include "lcd.h"

#include <inttypes.h>
#include "stdlib.h"

#define LCD_DELAY_MS 5

#define BACKLIGHT 0x08
#define NOBACKLIGHT 0x00

#define EN 0x04
#define RS 0x01

#define CLEAR_DISPLAY 0x01
#define GO_HOME 0x02
#define CONTROL 0x08

#define DISPLAY_ON 0x04
#define DISPLAY_OFF 0x00
#define CURSOR_ON 0x02
#define CURSOR_OFF 0x00
#define BLINK_ON 0x01
#define BLINK_OFF 0x00

#define DEFAULT_COLUMNS 16
#define DEFAULT_ROWS 2

uint8_t _addr;
uint8_t _backlight = BACKLIGHT;
uint8_t _columns = DEFAULT_COLUMNS;
uint8_t _row = DEFAULT_ROWS;
uint8_t _display = DISPLAY_ON;
uint8_t _cursor = CURSOR_OFF;
uint8_t _blink = BLINK_OFF;

I2C_HandleTypeDef *i2c;

volatile uint32_t wite_time = 0;
volatile uint32_t last_send_time = 0;

// Send data to the display
HAL_StatusTypeDef send(uint8_t data, uint8_t cmd)
{
	HAL_StatusTypeDef res;

	while (HAL_GetTick() - last_send_time < wite_time)
	{
		HAL_Delay(1);
	}

	for (;;)
	{
		res = HAL_I2C_IsDeviceReady(i2c, _addr, 1,
									HAL_MAX_DELAY);
		if (res == HAL_OK)
			break;
	}

	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	data_arr[0] = up | cmd | _backlight | EN;
	data_arr[1] = up | cmd | _backlight;
	data_arr[2] = lo | cmd | _backlight | EN;
	data_arr[3] = lo | cmd | _backlight;

	res = HAL_I2C_Master_Transmit(i2c, _addr, data_arr,
								  sizeof(data_arr), HAL_MAX_DELAY);
	last_send_time = HAL_GetTick();
	return res;
}

// Send command to the display
HAL_StatusTypeDef send_command(uint8_t cmd)
{
	HAL_StatusTypeDef res = send(cmd, 0);
	if (res == HAL_OK)
	{
		wite_time = 5;
	}
	return res;
}

// Send data to the display
HAL_StatusTypeDef send_data(uint8_t data)
{
	return send(data, RS);
}

// Initialize the display
HAL_StatusTypeDef LCD_init(I2C_HandleTypeDef *_i2c, uint8_t lcd_addr)
{
	i2c = _i2c;
	_addr = (lcd_addr << 1);

	while (HAL_GetTick() < 50)
	{
		HAL_Delay(1);
	}

	HAL_StatusTypeDef res = HAL_OK;
	res += send_command(0b00110000);
	res += send_command(0b00110000);
	res += send_command(0b00110000);

	res += send_command(GO_HOME);
	res += send_command(CONTROL | _display | _cursor | _blink);
	res += send_command(CLEAR_DISPLAY);

	return res;
}

// Initialize the display with custom size
HAL_StatusTypeDef LCD_init_custom(I2C_HandleTypeDef *_i2c, uint8_t lcd_addr, uint8_t column, uint8_t rows){
	_columns = column;
	_row = rows;
	return LCD_init(_i2c, lcd_addr);
}

// Clear the display
HAL_StatusTypeDef LCD_clear()
{
	return send_command(CLEAR_DISPLAY);
}

// Set cursor to the home position 0, 0
HAL_StatusTypeDef LCD_home()
{
	return send_command(GO_HOME);
}

// Set cursor to the specified position
HAL_StatusTypeDef LCD_set_pos(uint8_t column, uint8_t row)
{
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	if (row > _row)
	{
		row = _row - 1;
	}

	return send_command(0x80 | (column + row_offsets[row]));
}

// Turn the backlight on or off
HAL_StatusTypeDef LCD_backlight(uint8_t state)
{
	if (state == 0)
	{
		_backlight = NOBACKLIGHT;
	}
	else
	{
		_backlight = BACKLIGHT;
	}
	return send_command(0);
}

// Turn the cursor on or off
HAL_StatusTypeDef LCD_cursor(uint8_t cursor)
{
	if (cursor == 0)
	{
		_cursor = CURSOR_OFF;
	}
	else
	{
		_cursor = CURSOR_ON;
	}

	return send_command(CONTROL | _display | _cursor | _blink);
}

// Turn the cursor blink on or off
HAL_StatusTypeDef LCD_blink(uint8_t blink)
{
	if (blink == 0)
	{
		_blink = BLINK_OFF;
	}
	else
	{
		_blink = BLINK_ON;
	}

	return send_command(CONTROL | _display | _cursor | _blink);
}

// Turn the display off
HAL_StatusTypeDef LCD_off()
{
	_backlight = NOBACKLIGHT;
	return send_command(CONTROL);
}

// Turn the display on
HAL_StatusTypeDef LCD_on()
{
	_backlight = BACKLIGHT;
	return send_command(CONTROL | _display | _cursor | _blink);
}

// Write a string to the display
HAL_StatusTypeDef LCD_write_str(char *str)
{
	HAL_StatusTypeDef res = HAL_OK;
	while (*str)
	{
		res += send_data((uint8_t)(*str));
		str++;
	}
	return res;
}

// Write data to the display
HAL_StatusTypeDef LCD_write_data(uint8_t *data, size_t size)
{
	HAL_StatusTypeDef res = HAL_OK;
	for (size_t i = 0; i < size; i++)
	{
		res += send_data(data[i]);
	}
	return res;
}