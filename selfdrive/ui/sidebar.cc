#include <stdio.h>
#include <string.h>
#include <math.h>
#include <map>

#include "paint.hpp"
#include "sidebar.hpp"

static void ui_draw_sidebar_background(UIState *s) {
  int sbr_x = !s->scene.uilayout_sidebarcollapsed ? 0 : -(sbr_w) + bdr_s * 2;
  ui_draw_rect(s->vg, sbr_x, 0, sbr_w, s->fb_h, COLOR_BLACK_ALPHA(85));
}

static void ui_draw_sidebar_settings_button(UIState *s) {
  const float alpha = s->active_app == cereal::UiLayoutState::App::SETTINGS ? 1.0f : 0.65f;
  ui_draw_image(s->vg, settings_btn.x, settings_btn.y, settings_btn.w, settings_btn.h, s->img_button_settings, alpha);
}

static void ui_draw_sidebar_home_button(UIState *s) {
  const float alpha = s->active_app == cereal::UiLayoutState::App::HOME ? 1.0f : 0.65f;;
  ui_draw_image(s->vg, home_btn.x, home_btn.y, home_btn.w, home_btn.h, s->img_button_home, alpha);
}

static void ui_draw_sidebar_network_strength(UIState *s) {
  static std::map<cereal::ThermalData::NetworkStrength, int> network_strength_map = {
      {cereal::ThermalData::NetworkStrength::UNKNOWN, 1},
      {cereal::ThermalData::NetworkStrength::POOR, 2},
      {cereal::ThermalData::NetworkStrength::MODERATE, 3},
      {cereal::ThermalData::NetworkStrength::GOOD, 4},
      {cereal::ThermalData::NetworkStrength::GREAT, 5}};
  const int network_img_h = 27;
  const int network_img_w = 176;
  const int network_img_x = 58;
  const int network_img_y = 196;
  const int img_idx = s->scene.thermal.getNetworkType() == cereal::ThermalData::NetworkType::NONE ? 0 : network_strength_map[s->scene.thermal.getNetworkStrength()];
  ui_draw_image(s->vg, network_img_x, network_img_y, network_img_w, network_img_h, s->img_network[img_idx], 1.0f);
}

static void ui_draw_sidebar_ip_addr(UIState *s) {
  const int network_ip_w = 220;
  const int network_ip_x = !s->scene.uilayout_sidebarcollapsed ? 38 : -(sbr_w); 
  const int network_ip_y = 255;

  char network_ip_str[20];
  snprintf(network_ip_str, sizeof(network_ip_str), "%s", s->scene.ipAddr);
  nvgFillColor(s->vg, COLOR_YELLOW);
  nvgFontSize(s->vg, 28);
  nvgFontFaceId(s->vg, s->font_sans_bold);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  nvgTextBox(s->vg, network_ip_x, network_ip_y, network_ip_w, network_ip_str, NULL);
}

static void ui_draw_sidebar_battery_text(UIState *s) {
  const int battery_img_w = 96;
  const int battery_img_x = !s->scene.uilayout_sidebarcollapsed ? 150 : -(sbr_w);
  const int battery_img_y = 303;

  char battery_str[7];
  snprintf(battery_str, sizeof(battery_str), "%d%%%s", s->scene.thermal.getBatteryPercent(), s->scene.thermal.getBatteryStatus() == "Charging" ? "+" : "-");  
  nvgFillColor(s->vg, COLOR_WHITE);
  nvgFontSize(s->vg, 44*0.8);
  nvgFontFaceId(s->vg, s->font_sans_regular);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  nvgTextBox(s->vg, battery_img_x, battery_img_y, battery_img_w, battery_str, NULL);
}

static void ui_draw_sidebar_network_type(UIState *s) {
  static std::map<cereal::ThermalData::NetworkType, const char *> network_type_map = {
      {cereal::ThermalData::NetworkType::NONE, "--"},
      {cereal::ThermalData::NetworkType::WIFI, "WiFi"},
      {cereal::ThermalData::NetworkType::CELL2_G, "2G"},
      {cereal::ThermalData::NetworkType::CELL3_G, "3G"},
      {cereal::ThermalData::NetworkType::CELL4_G, "4G"},
      {cereal::ThermalData::NetworkType::CELL5_G, "5G"}};
  const int network_x = 50;
  const int network_y = 303;
  const int network_w = 100;
  const char *network_type = network_type_map[s->scene.thermal.getNetworkType()];
  nvgFillColor(s->vg, COLOR_WHITE);
  nvgFontSize(s->vg, 48*0.8);
  nvgFontFaceId(s->vg, s->font_sans_regular);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  nvgTextBox(s->vg, network_x, network_y, network_w, network_type ? network_type : "--", NULL);
}

static void ui_draw_sidebar_metric(UIState *s, const char* label_str, const char* value_str, const int severity, const int y_offset, const char* message_str) {
  const int metric_x = 30;
  const int metric_y = 338 + y_offset;
  const int metric_w = 240;
  const int metric_h = message_str ? strchr(message_str, '\n') ? 124 : 100 : 148;

  NVGcolor status_color;

  if (severity == 0) {
    status_color = COLOR_WHITE;
  } else if (severity == 1) {
    status_color = COLOR_YELLOW;
  } else if (severity > 1) {
    status_color = COLOR_RED;
  }

  ui_draw_rect(s->vg, metric_x, metric_y, metric_w, metric_h,
               severity > 0 ? COLOR_WHITE : COLOR_WHITE_ALPHA(85), 20, 2);

  nvgBeginPath(s->vg);
  nvgRoundedRectVarying(s->vg, metric_x + 6, metric_y + 6, 18, metric_h - 12, 25, 0, 0, 25);
  nvgFillColor(s->vg, status_color);
  nvgFill(s->vg);

  if (!message_str) {
    nvgFillColor(s->vg, COLOR_WHITE);
    nvgFontSize(s->vg, 78*0.8);
    nvgFontFaceId(s->vg, s->font_sans_bold);
    nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, metric_x + 50, metric_y + 50, metric_w - 60, value_str, NULL);

    nvgFillColor(s->vg, COLOR_WHITE);
    nvgFontSize(s->vg, 48*0.8);
    nvgFontFaceId(s->vg, s->font_sans_regular);
    nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, metric_x + 50, metric_y + 50 + 66, metric_w - 60, label_str, NULL);
  } else {
    nvgFillColor(s->vg, COLOR_WHITE);
    nvgFontSize(s->vg, 48*0.8);
    nvgFontFaceId(s->vg, s->font_sans_bold);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, metric_x + 35, metric_y + (strchr(message_str, '\n') ? 40 : 50), metric_w - 50, message_str, NULL);
  }
}

static void ui_draw_sidebar_temp_metric(UIState *s) {
  static std::map<cereal::ThermalData::ThermalStatus, const int> temp_severity_map = {
      {cereal::ThermalData::ThermalStatus::GREEN, 0},
      {cereal::ThermalData::ThermalStatus::YELLOW, 1},
      {cereal::ThermalData::ThermalStatus::RED, 2},
      {cereal::ThermalData::ThermalStatus::DANGER, 3}};
  std::string temp_val = std::to_string((int)s->scene.thermal.getAmbient()) + "°C";
  ui_draw_sidebar_metric(s, "시스템온도", temp_val.c_str(), temp_severity_map[s->scene.thermal.getThermalStatus()], 0, NULL);
}

static void ui_draw_sidebar_panda_metric(UIState *s) {
  const int panda_y_offset = 32 + 148;

  int panda_severity = 0;
  std::string panda_message = "차량\n연결됨";
  if (s->scene.hwType == cereal::HealthData::HwType::UNKNOWN) {
    panda_severity = 2;
    panda_message = "차량\n연결안됨";
  } else if (s->started) {
  	if (s->scene.satelliteCount <= 0) {
  	  panda_severity = 0;
  	  panda_message = "차량\n연결됨";
  	} else {
      panda_severity = 0;
      panda_message = "차량연결됨\nGPS : " + std::to_string((int)s->scene.satelliteCount);
    }
  }
  ui_draw_sidebar_metric(s, NULL, NULL, panda_severity, panda_y_offset, panda_message.c_str());
}

static void ui_draw_sidebar_connectivity(UIState *s) {
  static std::map<NetStatus, std::pair<const char *, int>> connectivity_map = {
    {NET_ERROR, {"인터넷\n접속오류", 2}},
    {NET_CONNECTED, {"인터넷\n온라인", 0}},
    {NET_DISCONNECTED, {"인터넷\n오프라인", 1}},
  };
  auto net_params = connectivity_map[s->scene.athenaStatus];
  ui_draw_sidebar_metric(s, NULL, NULL, net_params.second, 180+158, net_params.first);
}

void ui_draw_sidebar(UIState *s) {
  ui_draw_sidebar_background(s);
  if (s->scene.uilayout_sidebarcollapsed) {
    return;
  }
  ui_draw_sidebar_settings_button(s);
  ui_draw_sidebar_home_button(s);
  ui_draw_sidebar_network_strength(s);
  ui_draw_sidebar_ip_addr(s);
  ui_draw_sidebar_battery_text(s);
  ui_draw_sidebar_network_type(s);
  ui_draw_sidebar_temp_metric(s);
  ui_draw_sidebar_panda_metric(s);
  ui_draw_sidebar_connectivity(s);
}
