#pragma once
#include <map>
#include "cereal/gen/cpp/log.capnp.h"

typedef cereal::CarControl::HUDControl::AudibleAlert AudibleAlert;

static std::map<AudibleAlert, std::pair<const char *, int>> sound_map {
  // AudibleAlert, (file path, loop count)
  {AudibleAlert::CHIME_DISENGAGE, {"../assets/sounds/disengaged.wav", 0}},
  {AudibleAlert::CHIME_ENGAGE, {"../assets/sounds/engaged.wav", 0}},
  {AudibleAlert::CHIME_WARNING1, {"../assets/sounds/warning_1.wav", 0}},
  {AudibleAlert::CHIME_WARNING2, {"../assets/sounds/warning_2.wav", 0}},
  {AudibleAlert::CHIME_WARNING2_REPEAT, {"../assets/sounds/warning_2.wav", 3}},
  {AudibleAlert::CHIME_WARNING_REPEAT, {"../assets/sounds/warning_repeat.wav", 1}},
  {AudibleAlert::CHIME_ERROR, {"../assets/sounds/error.wav", 0}},
  {AudibleAlert::CHIME_PROMPT, {"../assets/sounds/error.wav", 0}},
  {AudibleAlert::CHIME_READY, {"../assets/sounds/ready.wav", 0}},
  {AudibleAlert::CHIME_DOOR_OPEN, {"../assets/sounds/dooropen.wav", 0}},
  {AudibleAlert::CHIME_GEAR_DRIVE, {"../assets/sounds/geardrive.wav", 0}},
  {AudibleAlert::CHIME_LANE_CHANGE, {"../assets/sounds/lanechange.wav", 0}},
  {AudibleAlert::CHIME_LANE_DEPARTURE, {"../assets/sounds/lanedeparture.wav", 0}},
  {AudibleAlert::CHIME_ROAD_WARNING, {"../assets/sounds/roadwarning.wav", 0}},
  {AudibleAlert::CHIME_SEAT_BELT, {"../assets/sounds/seatbelt.wav", 0}},
  {AudibleAlert::CHIME_VIEW_UNCERTAIN, {"../assets/sounds/viewuncertain.wav", 0}},
  {AudibleAlert::CHIME_MODE_OPENPILOT, {"../assets/sounds/modeopenpilot.wav", 0}},
  {AudibleAlert::CHIME_MODE_DISTCURV, {"../assets/sounds/modedistcurv.wav", 0}},
  {AudibleAlert::CHIME_MODE_DISTANCE, {"../assets/sounds/modedistance.wav", 0}},
  {AudibleAlert::CHIME_MODE_ONEWAY, {"../assets/sounds/modeoneway.wav", 0}}
};

class Sound {
public:
  virtual bool play(AudibleAlert alert) = 0;
  virtual void stop() = 0;
  virtual void setVolume(int volume) = 0;
};
