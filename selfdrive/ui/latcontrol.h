#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "common/params.h"
#include "ui.hpp"


bool control_button_clicked1(int touch_x, int touch_y) {
  if (touch_x >= 1585 && touch_x <= 1725) {
    if (touch_y >= 905 && touch_y <= 1045) {
      return true;
    }
  }
  return false;
}

bool control_button_clicked2(int touch_x, int touch_y) {
  if (touch_x >= 1425 && touch_x <= 1565) {
    if (touch_y >= 905 && touch_y <= 1045) {
      return true;
    }
  }
  return false;
}

static void draw_control_button1(UIState *s, int touch_x, int touch_y) {
  if (s->vision_connected){
    int btn_w = 140;
    int btn_h = 140;
    int btn_x1 = 1920 - btn_w - 195;
    int btn_y = 1080 - btn_h - 35;
    int btn_xc1 = btn_x1 + (btn_w/2);
    int btn_yc = btn_y + (btn_h/2);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    if (s->limit_set_speed_camera == 1) {
      nvgStrokeColor(s->vg, nvgRGBA(230,170,3,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,175,3,80));
      nvgFill(s->vg);
    } else {
      nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
    }
    nvgFontSize(s->vg, 45);
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgText(s->vg,btn_xc1,btn_yc,"CAM",NULL);
  }
}

static void draw_control_button2(UIState *s, int touch_x, int touch_y) {
  if (s->vision_connected){
    int btn_w = 140;
    int btn_h = 140;
    int btn_x2 = 1920 - btn_w - 355;
    int btn_y = 1080 - btn_h - 35;
    int btn_xc2 = btn_x2 + (btn_w/2);
    int btn_yc = btn_y + (btn_h/2);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x2, btn_y, btn_w, btn_h, 100);
    if (s->limit_set_speed == 1) {
      nvgStrokeColor(s->vg, nvgRGBA(55,184,104,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgFillColor(s->vg, nvgRGBA(0,255,0,80));
      nvgFill(s->vg);
    } else {
      nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
    }
    nvgFontSize(s->vg, 45);
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgText(s->vg,btn_xc2,btn_yc,"LIM",NULL);
  }
}

bool latcontrol( UIState *s, int touch_x, int touch_y ) {

  bool touched = false;

  draw_control_button1(s, touch_x, touch_y);
  draw_control_button2(s, touch_x, touch_y);

  if ((control_button_clicked1(touch_x,touch_y)) && (s->scene.uilayout_sidebarcollapsed == true)) {
    if (s->limit_set_speed_camera == false) {
      s->limit_set_speed_camera = true;
      Params().write_db_value("LimitSetSpeedCamera", "1", 1);
    } else if (s->limit_set_speed_camera == true) {
      s->limit_set_speed_camera = false;
      Params().write_db_value("LimitSetSpeedCamera", "0", 1);
    }
    touched = true;
  }

  if ((control_button_clicked2(touch_x,touch_y)) && (s->scene.uilayout_sidebarcollapsed == true)) {
    if (s->limit_set_speed == false) {
      s->limit_set_speed = true;
      Params().write_db_value("LimitSetSpeed", "1", 1);
    } else if (s->limit_set_speed == true) {
      s->limit_set_speed = false;
      Params().write_db_value("LimitSetSpeed", "0", 1);
    }
    touched = true;
  }

  return touched;
}