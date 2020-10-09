#include <map>

#include <Arduino.h>
#include <APRS-Decoder.h>
#include <LoRa_APRS.h>

#include "pins.h"
#include "settings.h"
#include "display.h"

#if defined(ARDUINO_T_Beam) && !defined(ARDUINO_T_Beam_V0_7)
#include "power_management.h"
PowerManagement powerManagement;
#endif

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * timer = NULL;
volatile uint secondsSinceLastTX = 0;
volatile uint secondsSinceStartup = 0;

LoRa_APRS lora_aprs;

String create_lat_aprs(double lat);
String create_long_aprs(double lng);

void setup_lora();

std::map<uint, std::shared_ptr<APRSMessage>> lastMessages;

void IRAM_ATTR onTimer()
{
	portENTER_CRITICAL_ISR(&timerMux);
	secondsSinceLastTX++;
	secondsSinceStartup++;
	portEXIT_CRITICAL_ISR(&timerMux);
}


// cppcheck-suppress unusedFunction
void setup()
{
	Serial.begin(115200);

#if defined(ARDUINO_T_Beam) && !defined(ARDUINO_T_Beam_V0_7)
	Wire.begin(SDA, SCL);
	if (!powerManagement.begin(Wire))
	{
		Serial.println("LoRa-APRS / Init / AXP192 Begin PASS");
	} else {
		Serial.println("LoRa-APRS / Init / AXP192 Begin FAIL");
	}
	powerManagement.activateLoRa();
	powerManagement.activateOLED();
	powerManagement.deactivateGPS();
#endif

	setup_display();
	
	delay(500);
	Serial.println("[INFO] LoRa APRS Digi by OE5BPA (Peter Buchegger)");
	show_display("OE5BPA", "LoRa APRS Digi", "by Peter Buchegger", 2000);

	setup_lora();

	timer = timerBegin(0, 80, true);
	timerAlarmWrite(timer, 1000000, true);
	timerAttachInterrupt(timer, &onTimer, true);
	timerAlarmEnable(timer);
	
	delay(500);
}

// cppcheck-suppress unusedFunction
void loop()
{
	static bool send_update = true;

	if(secondsSinceLastTX >= (BEACON_TIMEOUT*60))
	{
		portENTER_CRITICAL(&timerMux);
		secondsSinceLastTX -= (BEACON_TIMEOUT*60);
		portEXIT_CRITICAL(&timerMux);
		send_update = true;
	}

	if(send_update)
	{
		send_update = false;

		std::shared_ptr<APRSMessage> msg = std::shared_ptr<APRSMessage>(new APRSMessage());
		msg->setSource(CALL);
		msg->setDestination("APLG0");
		String lat = create_lat_aprs(BEACON_LAT);
		String lng = create_long_aprs(BEACON_LNG);
		msg->getAPRSBody()->setData(String("=") + lat + "R" + lng + "#" + BEACON_MESSAGE);
		String data = msg->encode();
		Serial.print(data);
		show_display(CALL, "<< Beaconing myself >>", data);
		lora_aprs.sendMessage(msg);
		Serial.println("finished TXing...");
	}

	if(lora_aprs.hasMessage())
	{
		std::shared_ptr<APRSMessage> msg = lora_aprs.getMessage();

		if(msg->getSource().indexOf(CALL) != -1)
		{
			Serial.print("Message already received as repeater: '");
			Serial.print(msg->toString());
			Serial.print("' with RSSI ");
			Serial.print(lora_aprs.getMessageRssi());
			Serial.print(" and SNR ");
			Serial.println(lora_aprs.getMessageSnr());
			return;
		}

		// lets try not to flood the LoRa frequency in limiting the same messages:
		std::map<uint, std::shared_ptr<APRSMessage>>::iterator foundMsg = std::find_if(lastMessages.begin(), lastMessages.end(), [&](std::pair<const unsigned int, std::shared_ptr<APRSMessage> > & old_msg)
			{
				if(msg->getSource() == old_msg.second->getSource() &&
					msg->getDestination() == old_msg.second->getDestination() &&
					msg->getAPRSBody()->getData() == old_msg.second->getAPRSBody()->getData())
				{
					return true;
				}
				return false;
			});

		if(foundMsg == lastMessages.end())
		{
			show_display(CALL, "RSSI: " + String(lora_aprs.getMessageRssi()) + ", SNR: " + String(lora_aprs.getMessageSnr()), msg->toString(), 0);
			Serial.print("Received packet '");
			Serial.print(msg->toString());
			Serial.print("' with RSSI ");
			Serial.print(lora_aprs.getMessageRssi());
			Serial.print(" and SNR ");
			Serial.println(lora_aprs.getMessageSnr());
			msg->setPath(String(CALL) + "*");
			lora_aprs.sendMessage(msg);
			lastMessages.insert({secondsSinceStartup, msg});
		}
		else
		{
			Serial.print("Message already received (timeout): '");
			Serial.print(msg->toString());
			Serial.print("' with RSSI ");
			Serial.print(lora_aprs.getMessageRssi());
			Serial.print(" and SNR ");
			Serial.println(lora_aprs.getMessageSnr());
		}
		return;
	}

	for(std::map<uint, std::shared_ptr<APRSMessage>>::iterator iter = lastMessages.begin(); iter != lastMessages.end(); )
	{
		if(secondsSinceStartup >= iter->first + FORWARD_TIMEOUT*60)
		{
			iter = lastMessages.erase(iter);
		}
		else
		{
			iter++;
		}
	}

	static int _secondsSinceLastTX = 0;
	if(secondsSinceLastTX != _secondsSinceLastTX)
	{
		show_display(CALL, "Time to next beaconing: " + String((BEACON_TIMEOUT*60) - secondsSinceLastTX));
	}
}

void setup_lora()
{
	lora_aprs.tx_frequency = LORA_RX_FREQUENCY;
	//lora_aprs.rx_frequency = LORA_TX_FREQUENCY; // for debugging
	if (!lora_aprs.begin())
	{
		Serial.println("[ERROR] Starting LoRa failed!");
		show_display("ERROR", "Starting LoRa failed!");
		while (1);
	}
	Serial.println("[INFO] LoRa init done!");
	show_display("INFO", "LoRa init done!", 2000);
}

String create_lat_aprs(double lat)
{
	char str[20];
	char n_s = 'N';
	if(lat < 0)
	{
		n_s = 'S';
	}
	lat = std::abs(lat);
	sprintf(str, "%02d%05.2f%c", (int)lat, (lat - (double)((int)lat)) * 60.0, n_s);
	String lat_str(str);
	return lat_str;
}

String create_long_aprs(double lng)
{
	char str[20];
	char e_w = 'E';
	if(lng < 0)
	{
		e_w = 'W';
	}
	lng = std::abs(lng);
	sprintf(str, "%03d%05.2f%c", (int)lng, (lng - (double)((int)lng)) * 60.0, e_w);
	String lng_str(str);
	return lng_str;
}
