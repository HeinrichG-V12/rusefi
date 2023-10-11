//DO NOT EDIT MANUALLY, let automation work hard.

// auto-generated by PinoutLogic.java based on config/boards/hellen/small-can-board/connectors/small.yaml
#include "pch.h"

// see comments at declaration in pin_repository.h
const char * getBoardSpecificPinName(brain_pin_e brainPin) {
	switch(brainPin) {
		case Gpio::A8: return "C8 Low Side";
		case Gpio::B7: return "A7 Low Side";
		case Gpio::C8: return "A8 Low Side";
		case Gpio::C9: return "B8 Low Side";
		case Gpio::E12: return "C3 Digital Input SENT";
		case Gpio::E14: return "C2 Digital Input";
		case Gpio::F11: return "C4 - Digital Input 3";
		default: return nullptr;
	}
	return nullptr;
}
