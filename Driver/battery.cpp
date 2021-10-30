#include "battery.h"

Battery::Battery(float _threshold, float _volt_min, float _volt_max)
{
	threshold = _threshold;
	volt_min = _volt_min;
	volt_max = _volt_max;
}

void Battery::init()
{
		   pinMode(PA0, INPUT_ANALOG_DMA);
	    ADC_DMA_Init();

}

float Battery::get_volt()
{
	return ((analogRead_DMA(PA0) * 3.3 * 11) / 4095.0);
}

float Battery::get_battery_notifier()
{
	float volt = get_volt();
	if(volt > volt_max)
		volt = volt_max;

	return ((volt- volt_min) / (volt_max - volt_min) * 100);
}

bool Battery::get_battery_low()
{
	if(get_battery_notifier() > threshold)
		return false;
	else
		return true;
}
