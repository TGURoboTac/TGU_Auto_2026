//
// Created by double_J on 2026/4/23.
//

#include "def.h"
#include "bsp/buzzer.h"
#include "bsp/time.h"
#include "motor/dji.h"
#include "utils/os.h"
#include "utils/vofa.h"

#define MUSIC_BUZZER_DUTY   0.3f

// 音符
#define REST 0

#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494

#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988

typedef struct {
    uint16_t freq;
    uint16_t ms;
} buzzer_note_t;

static volatile uint8_t g_music_stop = 0;

static const buzzer_note_t *g_song = nullptr;
static size_t g_song_len = 0;
static size_t g_song_idx = 0;
static uint8_t g_song_phase = 0;
static uint8_t g_song_active = 0;
static uint32_t g_next_time = 0;
static uint32_t g_gap_ms = 15;

void music_init() {
    bsp_buzzer_init();
    bsp_buzzer_quiet();
}

void music_stop() {
    g_music_stop = 1;
    g_song_active = 0;
    bsp_buzzer_quiet();
}

static void music_start_song(const buzzer_note_t *song, size_t len, uint32_t gap_ms) {
    g_song = song;
    g_song_len = len;
    g_song_idx = 0;
    g_song_phase = 0;
    g_song_active = 1;
    g_next_time = 0;
    g_gap_ms = gap_ms;
    g_music_stop = 0;
    bsp_buzzer_quiet();
}

void music_tick() {
    if (!g_song_active || g_music_stop || g_song == nullptr || g_song_len == 0)
        return;

    uint32_t now = bsp_time_get_ms();

    if (g_next_time != 0 && static_cast<int32_t>(now - g_next_time) < 0)
        return;

    if (g_song_idx >= g_song_len) {
        g_song_active = 0;
        bsp_buzzer_quiet();
        return;
    }

    const buzzer_note_t *note = &g_song[g_song_idx];

    if (g_song_phase == 0) {
        if (note->freq == REST) {
            bsp_buzzer_quiet();
            g_next_time = now + note->ms;
            g_song_idx++;
        } else {
            bsp_buzzer_alarm((float)note->freq, MUSIC_BUZZER_DUTY);
            g_next_time = now + note->ms;
            g_song_phase = 1;
        }
    } else {
        bsp_buzzer_quiet();
        g_next_time = now + g_gap_ms;
        g_song_phase = 0;
        g_song_idx++;
    }
}

/* ================= 抒情旋律B版（高辨识顺耳版） ================= */

static const buzzer_note_t song[] = {
    // 前奏
    {NOTE_A4,260},{NOTE_C5,260},{NOTE_D5,260},{NOTE_C5,260},
    {NOTE_A4,260},{NOTE_G4,260},{NOTE_A4,500},{REST,120},

    {NOTE_A4,240},{NOTE_C5,240},{NOTE_D5,240},{NOTE_E5,240},
    {NOTE_D5,240},{NOTE_C5,240},{NOTE_A4,500},{REST,140},

    // 主段1
    {NOTE_G4,240},{NOTE_A4,240},{NOTE_C5,240},{NOTE_A4,240},
    {NOTE_G4,240},{NOTE_E4,240},{NOTE_G4,480},{REST,120},

    {NOTE_A4,240},{NOTE_C5,240},{NOTE_D5,240},{NOTE_C5,240},
    {NOTE_A4,520},{REST,150},

    {NOTE_A4,240},{NOTE_C5,240},{NOTE_D5,240},{NOTE_E5,240},
    {NOTE_D5,240},{NOTE_C5,240},{NOTE_A4,500},{REST,140},

    {NOTE_G4,240},{NOTE_A4,240},{NOTE_C5,240},{NOTE_A4,240},
    {NOTE_G4,240},{NOTE_E4,240},{NOTE_G4,600},{REST,180},

    // 过渡
    {NOTE_C5,220},{NOTE_D5,220},{NOTE_E5,220},{NOTE_D5,220},
    {NOTE_C5,220},{NOTE_A4,220},{NOTE_C5,520},{REST,120},

    {NOTE_A4,220},{NOTE_C5,220},{NOTE_D5,220},{NOTE_C5,220},
    {NOTE_A4,220},{NOTE_G4,220},{NOTE_A4,520},{REST,120},

    // 高潮
    {NOTE_D5,220},{NOTE_E5,220},{NOTE_F5,220},{NOTE_E5,220},
    {NOTE_D5,220},{NOTE_C5,220},{NOTE_D5,540},{REST,140},

    {NOTE_E5,220},{NOTE_F5,220},{NOTE_G5,220},{NOTE_F5,220},
    {NOTE_E5,220},{NOTE_D5,220},{NOTE_E5,540},{REST,140},

    {NOTE_D5,220},{NOTE_E5,220},{NOTE_F5,220},{NOTE_E5,220},
    {NOTE_D5,220},{NOTE_C5,220},{NOTE_A4,540},{REST,140},

    {NOTE_C5,240},{NOTE_D5,240},{NOTE_E5,240},{NOTE_D5,240},
    {NOTE_C5,240},{NOTE_A4,240},{NOTE_G4,720},{REST,220},

    // 尾声
    {NOTE_A4,260},{NOTE_C5,260},{NOTE_D5,260},{NOTE_C5,260},
    {NOTE_A4,260},{NOTE_G4,260},{NOTE_A4,800},{REST,300}
};

/* ================= 接口 ================= */

void music_start() {
    music_start_song(song, sizeof(song) / sizeof(song[0]), 15);
}

[[noreturn]] void music_task(void *args) {
    // music_init();
    // music_start();
    uint32_t cur_time = 0;
    while (true) {
        // music_tick();
        if (cross_beep_req and cur_time < 400) {
            cross_beep_req = true, cur_time ++;
            bsp_buzzer_alarm(3000,0.2f);
        } else {
            cross_beep_req = false, cur_time = 0;
            bsp_buzzer_quiet();
        }
        os::task::sleep(1);
    }
}