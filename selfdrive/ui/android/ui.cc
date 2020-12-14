#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/resource.h>

#include <algorithm>

#include "common/util.h"
#include "common/utilpp.h"
#include "common/params.h"
#include "common/touch.h"
#include "common/swaglog.h"

#include "ui.hpp"
#include "paint.hpp"
#include "android/sl_sound.hpp"
#include "dashcam.h"
#include "latcontrol.h"

volatile sig_atomic_t do_exit = 0;
static void set_do_exit(int sig) {
  do_exit = 1;
}

static void ui_set_brightness(UIState *s, int brightness) {
  static int last_brightness = -1;
  if (last_brightness != brightness && (s->awake || brightness == 0)) {
    if (set_brightness(brightness)) {
      last_brightness = brightness;
    }
  }
}

int event_processing_enabled = -1;
static void enable_event_processing(bool yes) {
  if (event_processing_enabled != 1 && yes) {
    system("service call window 18 i32 1");  // enable event processing
    event_processing_enabled = 1;
  } else if (event_processing_enabled != 0 && !yes) {
    system("service call window 18 i32 0");  // disable event processing
    event_processing_enabled = 0;
  }
}

static void set_awake(UIState *s, bool awake) {
  if (awake) {
    // 30 second timeout
    s->awake_timeout = (s->nOpkrAutoScreenOff && s->started)? s->nOpkrAutoScreenOff*60*UI_FREQ : 30*UI_FREQ;
  }
  if (s->awake != awake) {
    s->awake = awake;

    // TODO: replace command_awake and command_sleep with direct calls to android
    if (awake) {
      LOGW("awake normal");
      framebuffer_set_power(s->fb, HWC_POWER_MODE_NORMAL);
      enable_event_processing(true);
    } else {
      LOGW("awake off");
      ui_set_brightness(s, 0);
      //framebuffer_set_power(s->fb, HWC_POWER_MODE_OFF);
      enable_event_processing(false);
    }
  }
}

static void handle_vision_touch(UIState *s, int touch_x, int touch_y) {
  if (s->started && (touch_x >= s->scene.viz_rect.x - bdr_s)
      && (s->active_app != cereal::UiLayoutState::App::SETTINGS)) {
    if (!s->scene.frontview) {
      s->scene.uilayout_sidebarcollapsed = !s->scene.uilayout_sidebarcollapsed;
    } else {
      Params().write_db_value("IsDriverViewEnabled", "0", 1);
    }
  }
}

static void handle_sidebar_touch(UIState *s, int touch_x, int touch_y) {
  if (!s->scene.uilayout_sidebarcollapsed && touch_x <= sbr_w) {
    if (settings_btn.ptInRect(touch_x, touch_y)) {
      s->active_app = cereal::UiLayoutState::App::SETTINGS;
    } else if (home_btn.ptInRect(touch_x, touch_y)) {
      if (s->started) {
        s->active_app = cereal::UiLayoutState::App::NONE;
        s->scene.uilayout_sidebarcollapsed = true;
      } else {
        s->active_app = cereal::UiLayoutState::App::HOME;
      }
    }
  }
}

static void update_offroad_layout_state(UIState *s, PubMaster *pm) {
  static int timeout = 0;
  static bool prev_collapsed = false;
  static cereal::UiLayoutState::App prev_app = cereal::UiLayoutState::App::NONE;
  if (timeout > 0) {
    timeout--;
  }
  if (prev_collapsed != s->scene.uilayout_sidebarcollapsed || prev_app != s->active_app || timeout == 0) {
    MessageBuilder msg;
    auto layout = msg.initEvent().initUiLayoutState();
    layout.setActiveApp(s->active_app);
    layout.setSidebarCollapsed(s->scene.uilayout_sidebarcollapsed);
    pm->send("offroadLayout", msg);
    LOGD("setting active app to %d with sidebar %d", (int)s->active_app, s->scene.uilayout_sidebarcollapsed);
    prev_collapsed = s->scene.uilayout_sidebarcollapsed;
    prev_app = s->active_app;
    timeout = 2 * UI_FREQ;
  }
}

int main(int argc, char* argv[]) {
  setpriority(PRIO_PROCESS, 0, -14);
  signal(SIGINT, (sighandler_t)set_do_exit);
  SLSound sound;

  UIState uistate = {};
  UIState *s = &uistate;
  ui_init(s);
  s->sound = &sound;

  set_awake(s, true);
  enable_event_processing(true);

  PubMaster *pm = new PubMaster({"offroadLayout"});

  TouchState touch = {0};
  touch_init(&touch);

  // light sensor scaling and volume params
  const bool LEON = util::read_file("/proc/cmdline").find("letv") != std::string::npos;

  float brightness_b = 0, brightness_m = 0;
  int result = read_param(&brightness_b, "BRIGHTNESS_B", true);
  result += read_param(&brightness_m, "BRIGHTNESS_M", true);
  if(result != 0) {
    brightness_b = LEON ? 10.0 : 5.0;
    brightness_m = LEON ? 2.6 : 1.3;
    write_param_float(brightness_b, "BRIGHTNESS_B", true);
    write_param_float(brightness_m, "BRIGHTNESS_M", true);
  }
  float smooth_brightness = brightness_b;

  const int MIN_VOLUME = LEON ? 12 : 9;
  const int MAX_VOLUME = LEON ? 15 : 12;
  s->sound->setVolume(MIN_VOLUME);

  if (s->nOpkrAutoScreenOff && !s->awake) {
    set_awake(s, true);
  }

  while (!do_exit) {
    if (!s->started || !s->vision_connected) {
      // Delay a while to avoid 9% cpu usage while car is not started and user is keeping touching on the screen.
      usleep(30 * 1000);
    }
    double u1 = millis_since_boot();

    ui_update(s);

    // manage wakefulness
    if (s->started || s->ignition) {
      if (s->nOpkrAutoScreenOff) {
        // turn on screen when alert is here.
        if (s->awake_timeout == 0 && (s->status == STATUS_DISENGAGED || s->status == STATUS_ALERT || s->status == STATUS_WARNING || (s->scene.alert_text1 != ""))) {
          set_awake(s, true);
        }
      } else {
        set_awake(s, true);
      }
    }

    if (s->awake_timeout > 0) {
      s->awake_timeout--;
    } else {
      set_awake(s, false);
    }

    // poll for touch events
    int touch_x = -1, touch_y = -1;
    int touched = touch_poll(&touch, &touch_x, &touch_y, 0);

    if ((s->awake) && (dashcam(s, touch_x, touch_y))) {
      touched = 0;
    }

    if ((s->awake) && (latcontrol(s, touch_x, touch_y))) {
      touched = 0;
    }

    if (touched == 1) {
      if (s->nOpkrAutoScreenOff && s->awake_timeout == 0) {
        set_awake(s, true);
      } else {
        set_awake(s, true);
        handle_sidebar_touch(s, touch_x, touch_y);
        handle_vision_touch(s, touch_x, touch_y);
      }
    }

    // Don't waste resources on drawing in case screen is off
    if (!s->awake) {
      continue;
    }

    // up one notch every 5 m/s
    float min = MIN_VOLUME + s->scene.controls_state.getVEgo() / 5;
    if (s->nOpkrUIVolumeBoost > 0 || s->nOpkrUIVolumeBoost < 0) {
      min = min * (1 + s->nOpkrUIVolumeBoost * 0.01);
    }
    s->sound->setVolume(fmin(MAX_VOLUME, min)); // up one notch every 5 m/s

    // set brightness
    if (s->nOpkrUIBrightness == 0) {
    float clipped_brightness = (s->light_sensor*brightness_m) + brightness_b;
    if (clipped_brightness > 512) clipped_brightness = 512;
    smooth_brightness = clipped_brightness * 0.01 + smooth_brightness * 0.99;
    if (smooth_brightness > 255) smooth_brightness = 255;
    ui_set_brightness(s, (int)smooth_brightness);
    } else {
      ui_set_brightness(s, (int)(255*s->nOpkrUIBrightness*0.01));
    }

    update_offroad_layout_state(s, pm);

    ui_draw(s);
    double u2 = millis_since_boot();
    if (!s->scene.frontview && (u2-u1 > 66)) {
      // warn on sub 15fps
      LOGW("slow frame(%llu) time: %.2f", (s->sm)->frame, u2-u1);
    }
    framebuffer_swap(s->fb);
  }

  set_awake(s, true);
  delete s->sm;
  delete pm;
  return 0;
}
