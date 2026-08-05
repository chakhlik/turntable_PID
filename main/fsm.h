#pragma once

#include <stdbool.h>

typedef enum {
    FSM_STATE_IDLE_33,
    FSM_STATE_IDLE_45,
    FSM_STATE_ON_33,
    FSM_STATE_ON_45,
    FSM_STATE_ROTATION_33,
    FSM_STATE_ROTATION_45
} fsm_state_t;

void fsm_init(void);
void fsm_task(void *pvParameters);
fsm_state_t fsm_get_state(void);

// События
void fsm_event_play(void);
void fsm_event_stop(void);
void fsm_event_speed_33(void);
void fsm_event_speed_45(void);
void fsm_event_tonearm_outer(bool closed);
void fsm_event_tonearm_inner(bool closed);