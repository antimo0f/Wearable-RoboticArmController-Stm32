#include "display.h"
#include "lcd1602.h"
#include <stdio.h>
#include <string.h>

static void Display_PrintField(uint8_t row, uint8_t col, const char *text, uint8_t width)
{
	char field[4];
	size_t len = strlen(text);

	if (width > 3U)
	{
		width = 3U;
	}

	if (len > width)
	{
		len = width;
	}

	memset(field, ' ', width);
	memcpy(field, text, len);
	field[width] = '\0';

	HD44780_SetCursor(col, row);
	HD44780_PrintStr(field);
}

#define DISPLAY_FIELD_COUNT 5U

static const char *display_labels[DISPLAY_FIELD_COUNT] = {"1", "2", "3", "4", "5"};
static int display_values[DISPLAY_FIELD_COUNT] = {0, 0, 0, 0, 0};
static char display_mode_char = 'N';

void Display_Init(void)
{
	HD44780_Init(2);
	HD44780_Clear();
}

void Display_RenderAll(void)
{
	uint8_t i;

	for (i = 0U; i < DISPLAY_FIELD_COUNT; i++)
	{
		char name_field[4];
		char value_field[4];
		int value = display_values[i];
		uint8_t col = (uint8_t)(i * 3U);

		value = value < 0 ? -value : value;

		snprintf(name_field, sizeof(name_field), "%-2.2s", display_labels[i]);
		if (value == 0 && i != 4U) {
			snprintf(value_field, sizeof(value_field), "   ");
		} else {
			if (value > 999) value = 999;
			snprintf(value_field, sizeof(value_field), "%3d", value);
		}
		Display_PrintField(0, col, name_field, 3U);
		Display_PrintField(1, col, value_field, 3U);
	}

	char mode_field[2];
	mode_field[0] = 'M';
	mode_field[1] = '\0';
	Display_PrintField(0, 15, mode_field, 1U);
	mode_field[0] = display_mode_char;
	Display_PrintField(1, 15, mode_field, 1U);
}

void Display_RenderValues(const int *values)
{
	uint8_t i;
	const int *source = values;

	if (source == NULL)
	{
		source = display_values;
	}

	for (i = 0U; i < DISPLAY_FIELD_COUNT; i++)
	{
		char value_field[4];
		int value = source[i];
		uint8_t col = (uint8_t)(i * 3U);

		value = value < 0 ? -value : value;

		if (value == 0 && i != 4U) {
			snprintf(value_field, sizeof(value_field), "   ");
		} else {
			if (value > 999) value = 999;
			snprintf(value_field, sizeof(value_field), "%3d", value);
		}

		Display_PrintField(1, col, value_field, 3U);
	}
}

void Display_SetValue(uint8_t index, int value)
{
	if (index >= DISPLAY_FIELD_COUNT)
	{
		return;
	}

	display_values[index] = value;
}

void Display_IncrementValue(uint8_t index, int delta)
{
	int value;

	if (index >= DISPLAY_FIELD_COUNT)
	{
		return;
	}

	value = display_values[index] + delta;
	Display_SetValue(index, value);
}

void Display_SetMode(char mode_char)
{
	display_mode_char = mode_char;
	char mode_field[2];
	mode_field[0] = mode_char;
	mode_field[1] = '\0';
	Display_PrintField(1, 15, mode_field, 1U);
}
