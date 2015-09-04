/******************************************************************************
 *                       Copyright (c) 2008, Jabil Circuit
 *
 * This source code and any compilation or derivative thereof is the sole
 * property of Jabil Circuit and is provided pursuant to a Software License
 * Agreement.  This code is the proprietary information of Jabil Circuit and
 * is confidential in nature.  Its use and dissemination by any party other
 * than Jabil Circuit is strictly limited by the confidential information
 * provisions of the Software License Agreement referenced above.
 *
 ******************************************************************************
 *
 * Project      : WaterOptimizer Irrigation System (WOIS)
 * Organization : WaterOptimizer, LLC
 * Module       : drvKeypad.c
 * Description  : This file implements the keypad driver, including support for
 *                both the pushbuttons and rotary encoder (navigation dial).
 *
 *****************************************************************************/

/* MODULE drvKeypad */

#include "global.h"
#include "hwBusData.h"
#include "hwBusKeypad.h"
#include "hwCpu.h"
//#include "hwLed.h"
#include "hwNav.h"

#include "drvKeypad.h"


/* non-tuneable parameters for keypad driver (based on hardware) */
#define DRV_KEYPAD_N_FUNCKEYS       16      /* number of function keys */
#define DRV_KEYPAD_N_SOFTKEYS       6       /* number of soft keys */
#define DRV_KEYPAD_N_ALLKEYS    (DRV_KEYPAD_N_FUNCKEYS + DRV_KEYPAD_N_SOFTKEYS)

/* tuneable parameters for keypad driver */
#define DRV_KEYPAD_QUEUE_SIZE       48U
#define DRV_KEYPAD_ALT_FUNC_DELAY   100U    /* 100 ticks = 2000ms */
#define DRV_KEYPAD_SOFT_RPT_DELAY_S 30U     /* 30 ticks = 600ms */
#define DRV_KEYPAD_SOFT_RPT_DELAY_F 230U    /* 230 ticks = 4600ms */
#define DRV_KEYPAD_SOFT_RPT_RATE    5U      /* 5 ticks = 100ms = 10/sec */


/* Defines for encoder states and manipulation */
#define DRV_KEYPAD_NAV_TURN_INC     10
#define DRV_KEYPAD_NAV_MAX_TURNS    10

enum drvKeypadNavStates
{
    IDLE = 0,
    ERROR,
    CW_START,
    CW_CENTER,
    CW_END,
    CCW_START,
    CCW_CENTER,
    CCW_END,
    NSTATES
};

/*
 * Nav-dial FSM inputs.
 * Note that the values depend on the values read from the GPIO pins, and
 * are modified by the FSM input-conditioning logic based on rotation history.
 */
enum drvKeypadNavInputs
{
    NO_BIAS_SW_AB    = 0,       /* both contacts closed, no recent rotation */
    NO_BIAS_SW_B     = 1,       /* B closed (and A open), no recent rotation */
    NO_BIAS_SW_A     = 2,       /* A closed (and B open), no recent rotation */
    NO_BIAS_SW_NONE  = 3,       /* both contacts open, no recent rotation */
    CW_BIAS_SW_AB    = 4,       /* ditto, but with recent clockwise motion */
    CW_BIAS_SW_B     = 5,
    CW_BIAS_SW_A     = 6,
    CW_BIAS_SW_NONE  = 7,
    CCW_BIAS_SW_AB   = 8,       /* ditto, but with counter-clockwise motion */
    CCW_BIAS_SW_B    = 9,
    CCW_BIAS_SW_A    = 10,
    CCW_BIAS_SW_NONE = 11,
    NINPUTS
};

enum drvKeypadNavActions
{
    NONE = 0,
    ENQ_CW,
    ENQ_CCW,
    BIAS_CW,
    BIAS_CCW,
    NACTIONS
};

#define EDGE(next_state,action) (((next_state) << 4) | (action))

/*
 * Nav-dial FSM transitions - indexed by current-state and input
 */
static const uint8_t drvKeypadNavFsmEdges[NSTATES][NINPUTS] =
{
    /* State = IDLE */
    {
        /* NO_BIAS_SW_AB    */ EDGE(ERROR,      NONE),
        /* NO_BIAS_SW_B     */ EDGE(CCW_START,  NONE),    /* CCW normal */
        /* NO_BIAS_SW_A     */ EDGE(CW_START,   NONE),    /* CW normal */
        /* NO_BIAS_SW_NONE  */ EDGE(IDLE,       NONE),    /* same state */
        /* CW_BIAS_SW_AB    */ EDGE(CW_CENTER,  NONE),    /* CW predictive */
        /* CW_BIAS_SW_B     */ EDGE(ERROR,      NONE),
        /* CW_BIAS_SW_A     */ EDGE(CW_START,   NONE),    /* CW normal */
        /* CW_BIAS_SW_NONE  */ EDGE(IDLE,       NONE),    /* same state */
        /* CCW_BIAS_SW_AB   */ EDGE(CCW_CENTER, NONE),    /* CCW predictive */
        /* CCW_BIAS_SW_B    */ EDGE(CCW_START,  NONE),    /* CCW normal */
        /* CCW_BIAS_SW_A    */ EDGE(ERROR,      NONE),
        /* CCW_BIAS_SW_NONE */ EDGE(IDLE,       NONE),    /* same state */
    },
    /* State = ERROR */
    {
        /* NO_BIAS_SW_AB    */ EDGE(ERROR,      NONE),
        /* NO_BIAS_SW_B     */ EDGE(ERROR,      NONE),
        /* NO_BIAS_SW_A     */ EDGE(ERROR,      NONE),
        /* NO_BIAS_SW_NONE  */ EDGE(IDLE,       NONE),    /* resynchronized */
        /* CW_BIAS_SW_AB    */ EDGE(ERROR,      NONE),
        /* CW_BIAS_SW_B     */ EDGE(ERROR,      NONE),
        /* CW_BIAS_SW_A     */ EDGE(ERROR,      NONE),
        /* CW_BIAS_SW_NONE  */ EDGE(IDLE,       NONE),    /* resynchronized */
        /* CCW_BIAS_SW_AB   */ EDGE(ERROR,      NONE),
        /* CCW_BIAS_SW_B    */ EDGE(ERROR,      NONE),
        /* CCW_BIAS_SW_A    */ EDGE(ERROR,      NONE),
        /* CCW_BIAS_SW_NONE */ EDGE(IDLE,       NONE),    /* resynchronized */
    },
    /* State = CW_START */
    {
        /* NO_BIAS_SW_AB    */ EDGE(CW_CENTER,  NONE),    /* CW normal */
        /* NO_BIAS_SW_B     */ EDGE(CW_END,     NONE),    /* CW guess */
        /* NO_BIAS_SW_A     */ EDGE(CW_START,   NONE),    /* same state */
        /* NO_BIAS_SW_NONE  */ EDGE(IDLE,       NONE),    /* CW debounce */
        /* CW_BIAS_SW_AB    */ EDGE(CW_CENTER,  NONE),    /* CW normal */
        /* CW_BIAS_SW_B     */ EDGE(CW_END,     NONE),    /* CW predictive */
        /* CW_BIAS_SW_A     */ EDGE(CW_START,   NONE),    /* same state */
        /* CW_BIAS_SW_NONE  */ EDGE(IDLE,       NONE),    /* CW debounce */
        /* CCW_BIAS_SW_AB   */ EDGE(CW_CENTER,  NONE),    /* CW normal */
        /* CCW_BIAS_SW_B    */ EDGE(ERROR,      NONE),
        /* CCW_BIAS_SW_A    */ EDGE(CW_START,   NONE),    /* same state */
        /* CCW_BIAS_SW_NONE */ EDGE(IDLE,       NONE),    /* CW debounce */
    },
    /* State = CW_CENTER */
    {
        /* NO_BIAS_SW_AB    */ EDGE(CW_CENTER,  NONE),    /* same state */
        /* NO_BIAS_SW_B     */ EDGE(CW_END,     NONE),    /* CW normal */
        /* NO_BIAS_SW_A     */ EDGE(CW_START,   NONE),    /* CW debounce */
        /* NO_BIAS_SW_NONE  */ EDGE(IDLE,       ENQ_CW),  /* CW guess */
        /* CW_BIAS_SW_AB    */ EDGE(CW_CENTER,  NONE),    /* same state */
        /* CW_BIAS_SW_B     */ EDGE(CW_END,     NONE),    /* CW normal */
        /* CW_BIAS_SW_A     */ EDGE(CW_START,   NONE),    /* CW debounce */
        /* CW_BIAS_SW_NONE  */ EDGE(IDLE,       ENQ_CW),  /* CW predictive */
        /* CCW_BIAS_SW_AB   */ EDGE(CW_CENTER,  NONE),    /* same state */
        /* CCW_BIAS_SW_B    */ EDGE(CW_END,     NONE),    /* CW normal */
        /* CCW_BIAS_SW_A    */ EDGE(CW_START,   NONE),    /* CW debounce */
        /* CCW_BIAS_SW_NONE */ EDGE(ERROR,      NONE),
    },
    /* State = CW_END */
    {
        /* NO_BIAS_SW_AB    */ EDGE(CW_CENTER,  NONE),    /* CW debounce */
        /* NO_BIAS_SW_B     */ EDGE(CW_END,     NONE),    /* same state */
        /* NO_BIAS_SW_A     */ EDGE(CW_START,   ENQ_CW),  /* CW guess */
        /* NO_BIAS_SW_NONE  */ EDGE(IDLE,       ENQ_CW),  /* CW normal */
        /* CW_BIAS_SW_AB    */ EDGE(CW_CENTER,  NONE),    /* CW debounce */
        /* CW_BIAS_SW_B     */ EDGE(CW_END,     NONE),    /* same state */
        /* CW_BIAS_SW_A     */ EDGE(CW_START,   ENQ_CW),  /* CW predictive */
        /* CW_BIAS_SW_NONE  */ EDGE(IDLE,       ENQ_CW),  /* CW normal */
        /* CCW_BIAS_SW_AB   */ EDGE(CW_CENTER,  NONE),    /* CW debounce */
        /* CCW_BIAS_SW_B    */ EDGE(CW_END,     NONE),    /* same state */
        /* CCW_BIAS_SW_A    */ EDGE(ERROR,      NONE),
        /* CCW_BIAS_SW_NONE */ EDGE(IDLE,       ENQ_CW),  /* CW normal */
    },
    /* State = CCW_START */
    {
        /* NO_BIAS_SW_AB    */ EDGE(CCW_CENTER, NONE),    /* CCW normal */
        /* NO_BIAS_SW_B     */ EDGE(CCW_START,  NONE),    /* same state */
        /* NO_BIAS_SW_A     */ EDGE(CCW_END,    NONE),    /* CCW guess */
        /* NO_BIAS_SW_NONE  */ EDGE(IDLE,       NONE),    /* CCW debounce */
        /* CW_BIAS_SW_AB    */ EDGE(CCW_CENTER, NONE),    /* CCW normal */
        /* CW_BIAS_SW_B     */ EDGE(CCW_START,  NONE),    /* same state */
        /* CW_BIAS_SW_A     */ EDGE(ERROR,      NONE),
        /* CW_BIAS_SW_NONE  */ EDGE(IDLE,       NONE),    /* CCW debounce */
        /* CCW_BIAS_SW_AB   */ EDGE(CCW_CENTER, NONE),    /* CCW normal */
        /* CCW_BIAS_SW_B    */ EDGE(CCW_START,  NONE),    /* same state */
        /* CCW_BIAS_SW_A    */ EDGE(CCW_END,    NONE),    /* CCW predictive */
        /* CCW_BIAS_SW_NONE */ EDGE(IDLE,       NONE),    /* CCW debounce */
    },
    /* State = CCW_CENTER */
    {
        /* NO_BIAS_SW_AB    */ EDGE(CCW_CENTER, NONE),    /* same state */
        /* NO_BIAS_SW_B     */ EDGE(CCW_START,  NONE),    /* CCW debounce */
        /* NO_BIAS_SW_A     */ EDGE(CCW_END,    NONE),    /* CCW normal */
        /* NO_BIAS_SW_NONE  */ EDGE(IDLE,       ENQ_CCW), /* CCW guess */
        /* CW_BIAS_SW_AB    */ EDGE(CCW_CENTER, NONE),    /* same state */
        /* CW_BIAS_SW_B     */ EDGE(CCW_START,  NONE),    /* CCW debounce */
        /* CW_BIAS_SW_A     */ EDGE(CCW_END,    NONE),    /* CCW normal */
        /* CW_BIAS_SW_NONE  */ EDGE(ERROR,      NONE),
        /* CCW_BIAS_SW_AB   */ EDGE(CCW_CENTER, NONE),    /* same state */
        /* CCW_BIAS_SW_B    */ EDGE(CCW_START,  NONE),    /* CCW debounce */
        /* CCW_BIAS_SW_A    */ EDGE(CCW_END,    NONE),    /* CCW normal */
        /* CCW_BIAS_SW_NONE */ EDGE(IDLE,       ENQ_CCW), /* CCW predictive */
    },
    /* State = CCW_END */
    {
        /* NO_BIAS_SW_AB    */ EDGE(CCW_CENTER, NONE),    /* CCW debounce */
        /* NO_BIAS_SW_B     */ EDGE(CCW_START,  ENQ_CCW), /* CCW guess */
        /* NO_BIAS_SW_A     */ EDGE(CCW_END,    NONE),    /* same state */
        /* NO_BIAS_SW_NONE  */ EDGE(IDLE,       ENQ_CCW), /* CCW normal */
        /* CW_BIAS_SW_AB    */ EDGE(CCW_CENTER, NONE),    /* CCW debounce */
        /* CW_BIAS_SW_B     */ EDGE(ERROR,      NONE),
        /* CW_BIAS_SW_A     */ EDGE(CCW_END,    NONE),    /* same state */
        /* CW_BIAS_SW_NONE  */ EDGE(IDLE,       ENQ_CCW), /* CCW normal */
        /* CCW_BIAS_SW_AB   */ EDGE(CCW_CENTER, NONE),    /* CCW debounce */
        /* CCW_BIAS_SW_B    */ EDGE(CCW_START,  ENQ_CCW), /* CCW predictive */
        /* CCW_BIAS_SW_A    */ EDGE(CCW_END,    NONE),    /* same state */
        /* CCW_BIAS_SW_NONE */ EDGE(IDLE,       ENQ_CCW), /* CCW normal */
    },
};

static uint8_t drvKeypadQueue[DRV_KEYPAD_QUEUE_SIZE];
static uint8_t drvKeypadQueueInsert = 0U;
static uint8_t drvKeypadQueueRemove = 0U;

static uint32_t drvKeypadKeyState = 0U;
static uint8_t  drvKeypadKeyHoldTime[DRV_KEYPAD_N_ALLKEYS] = {0};
static uint8_t  drvKeypadSoftKeyRptCnt[DRV_KEYPAD_N_SOFTKEYS];

static uint8_t drvKeypadNavState = 0U;
static uint8_t drvKeypadNavCcwTurns = 0U;
static uint8_t drvKeypadNavCwTurns = 0U;


/******************************************************************************
 *
 *  drvKeypadGet
 *
 *  DESCRIPTION:
 *      This driver API function polls the keypad event queue and returns the
 *      oldest event (if any) in the queue.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      pushbutton / nav-dial event, or DRV_KEYPAD_TYPE_NONE if none
 *
 *  NOTES:
 *      This can be invoked from any context but it is NOT reentrant.
 *
 *****************************************************************************/
uint8_t drvKeypadGet(void)
{
    uint8_t key = DRV_KEYPAD_TYPE_NONE;

    if (drvKeypadQueueRemove != drvKeypadQueueInsert)
    {
        key = drvKeypadQueue[drvKeypadQueueRemove];
        if (++drvKeypadQueueRemove >= DRV_KEYPAD_QUEUE_SIZE)
        {
            drvKeypadQueueRemove = 0;
        }
    }

    return key;
}


/******************************************************************************
 *
 *  drvKeypadPut
 *
 *  DESCRIPTION:
 *      This driver API function enqueues a keypad event in the event
 *      queue.  (Adding an event to a full queue clears the queue.)
 *
 *  PARAMETERS:
 *      key (in) - event code to add to the queue
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context (and it is reentrant).
 *      It is used by the switch and nav ISRs, which run at different levels,
 *      and by several parts of the irrigation logic.
 *
 *****************************************************************************/
void drvKeypadPut(uint8_t key)
{
    EnterCritical();                    /* save and disable interrupts */

    /* NOTE: adding to a full queue yields an empty queue */
    drvKeypadQueue[drvKeypadQueueInsert] = key;
    if (++drvKeypadQueueInsert >= DRV_KEYPAD_QUEUE_SIZE)
    {
        drvKeypadQueueInsert = 0;
    }

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvKeypadSwitchIsr
 *
 *  DESCRIPTION:
 *      This driver internal function is invoked by a 20ms timer ISR to perform
 *      periodic scans of the pushbutton switch matrix.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is called from a timer ISR.
 *
 *****************************************************************************/
void drvKeypadSwitchIsr(void)
{
    uint32_t newKeyState;
    uint32_t off2on;
    int i;
    static const uint8_t rowEnables[] =
    {
        (uint8_t)~0x04,                 /* row 2: SOFT 0-5 */
        (uint8_t)~0x02,                 /* row 1: FUNC 8-15 */
        (uint8_t)~0x01                  /* row 0: FUNC 0-7 */
    };

    EnterCritical();                    /* save and disable interrupts */

    /* scan hardware keypad matrix - 3 rows */
    hwBusData_SetInput();
    for (i = 0; i < sizeof(rowEnables) / sizeof(rowEnables[0]); i++)
    {
        /* drive appropriate row select low */
        hwBusKeypad_PutVal(rowEnables[i]);
        /* allow time for switch input values to settle */
        /* 2.5uS is enough time when using 10K pull-ups on the data bus */
        asm
        {
                move.l  #21,d0          /* 1 cycle = 40nS at 25MHz */
            @loop:
                subq.l  #1,d0           /* 1 cycle = 40nS at 25MHz */
                bne.b   @loop           /* 2 cycles = 80nS at 25MHz */
        }
        /* read keypad row */
        newKeyState = (newKeyState << 8) | (~hwBusData_GetVal() & 0xFF);
    }
    hwBusKeypad_PutVal(0xFF);           /* disable keypad row drivers */
    hwBusData_SetOutput();

    ExitCritical();                     /* restore interrupts */

    /* mask off bit positions for non-existant switches */
    newKeyState &= 0x003FFFFF;

    /* deal with OFF->ON (0->1) changes, if any */
    off2on = ~drvKeypadKeyState & newKeyState;
    if (off2on != 0)
    {
        for (i = 0; i < DRV_KEYPAD_N_ALLKEYS; i++)
        {
            if ((off2on & (1 << i)) != 0)
            {
                if (i < DRV_KEYPAD_N_FUNCKEYS)
                {
                    /* add function key keypress event to queue */
                    drvKeypadPut((uint8_t)(DRV_KEYPAD_KEY_FUNC(i)));
                }
                else
                {
                    uint8_t softkey = i - DRV_KEYPAD_N_FUNCKEYS;

                    /* add softkey keypress event to queue */
                    drvKeypadPut((uint8_t)(DRV_KEYPAD_KEY_SOFT(softkey)));
                    /* initialize softkey auto-repeat */
                    drvKeypadSoftKeyRptCnt[softkey] = 0;
                }
                /* initialize hold-time counter */
                drvKeypadKeyHoldTime[i] = 0;
            }
        }
    }

    /* deal with auto-repeat for keys that were ON, if any */
    if ((drvKeypadKeyState & 0x007FFFFF) != 0)
    {
        for (i = 0; i < DRV_KEYPAD_N_ALLKEYS; i++)
        {
            if ((drvKeypadKeyState & (1 << i)) != 0)
            {
                /* increment key hold time, up to uint8_t max of 255 */
                if (drvKeypadKeyHoldTime[i] < 255)
                {
                    drvKeypadKeyHoldTime[i]++;
                }
                if (i < DRV_KEYPAD_N_FUNCKEYS)
                {
                    /* handle alt-func press-and-hold detection */
                    if (drvKeypadKeyHoldTime[i] == DRV_KEYPAD_ALT_FUNC_DELAY)
                    {
                        /* add alt-func keypress event to queue */
                        drvKeypadPut((uint8_t)(DRV_KEYPAD_KEY_ALTFUNC(i)));
                    }
                }
                else
                {
                    uint8_t softkey = i - DRV_KEYPAD_N_FUNCKEYS;

                    /* handle soft-key slow/fast repeat */
                    if (drvKeypadKeyHoldTime[i] >= DRV_KEYPAD_SOFT_RPT_DELAY_F)
                    {
                        if (drvKeypadSoftKeyRptCnt[softkey] == 0)
                        {
                            /* add fast-repeat keypress event to queue */
                            drvKeypadPut((uint8_t)(DRV_KEYPAD_KEY_SOFT_FAST(softkey)));
                            /* reinitialize softkey auto-repeat */
                            drvKeypadSoftKeyRptCnt[softkey] = DRV_KEYPAD_SOFT_RPT_RATE - 1;
                        }
                        else
                        {
                            drvKeypadSoftKeyRptCnt[softkey]--;
                        }
                    }
                    else if (drvKeypadKeyHoldTime[i] >= DRV_KEYPAD_SOFT_RPT_DELAY_S)
                    {
                        if (drvKeypadSoftKeyRptCnt[softkey] == 0)
                        {
                            /* add slow-repeat keypress event to queue */
                            drvKeypadPut((uint8_t)(DRV_KEYPAD_KEY_SOFT_SLOW(softkey)));
                            /* reinitialize softkey auto-repeat */
                            drvKeypadSoftKeyRptCnt[softkey] = DRV_KEYPAD_SOFT_RPT_RATE - 1;
                        }
                        else
                        {
                            drvKeypadSoftKeyRptCnt[softkey]--;
                        }
                    }
                }
            }
        }
    }

    /* update saved key state */
    drvKeypadKeyState = newKeyState;
}


/******************************************************************************
 *
 *  drvKeypadNavIsr
 *
 *  DESCRIPTION:
 *      This driver internal function is invoked by a 2ms timer ISR to perform
 *      periodic sampling of the rotary encoder (nav dial).
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is called from a timer ISR.
 *
 *****************************************************************************/
void drvKeypadNavIsr(void)
{
    static uint8_t state = IDLE;
    uint8_t kbEncSet;

    /* read current encoder state */
    kbEncSet = hwNav_GetVal();
    if (drvKeypadNavState == kbEncSet)
    {
        /* no new encoder activity - decrement predictive turn variables */
        if (drvKeypadNavCwTurns > 0U)
        {
            drvKeypadNavCwTurns--;
        }
        if (drvKeypadNavCcwTurns > 0U)
        {
            drvKeypadNavCcwTurns--;
        }
    }

    else

    {
        /* encoder activity detected - process according to state machine */

        uint8_t input = kbEncSet;
        uint8_t edge;

        /* condition encoder switch state with rotation history */
        if (drvKeypadNavCwTurns > 0)
        {
            input += 4;                 /* CW_BIAS */
        }
        else if (drvKeypadNavCcwTurns > 0)
        {
            input += 8;                 /* CCW_BIAS */
        }

        /* get FSM transition based on current state and conditioned input */
        edge = drvKeypadNavFsmEdges[state][input];

        /* change to next state */
        state = (uint8_t)(edge >> 4);

        /* perform indicated action (if any) */
        switch (edge & 0x0F)
        {
            case ENQ_CW:
                /* add clockwise nav dial turn event to queue */
                drvKeypadPut(DRV_KEYPAD_KEY_NAV_CW);
                /* FALL THROUGH */
            case BIAS_CW:
                /* Increment the drvKeypadNavTurns variable IF the opposite
                   direction has not been detected recently */
                if (drvKeypadNavCcwTurns == 0)
                {
                    if ((drvKeypadNavCwTurns += DRV_KEYPAD_NAV_TURN_INC) > DRV_KEYPAD_NAV_MAX_TURNS)
                    {
                        drvKeypadNavCwTurns = DRV_KEYPAD_NAV_MAX_TURNS;
                    }
                }
                break;
            case ENQ_CCW:
                /* add counter-clockwise nav dial turn event to queue */
                drvKeypadPut(DRV_KEYPAD_KEY_NAV_CCW);
                /* FALL THROUGH */
            case BIAS_CCW:
                /* Increment the drvKeypadNavTurns variable IF the opposite
                   direction has not been detected recently */
                if (drvKeypadNavCwTurns == 0)
                {
                    if ((drvKeypadNavCcwTurns += DRV_KEYPAD_NAV_TURN_INC) > DRV_KEYPAD_NAV_MAX_TURNS)
                    {
                        drvKeypadNavCcwTurns = DRV_KEYPAD_NAV_MAX_TURNS;
                    }
                }
                break;
            case NONE:
            default:
                break;
        }

        //drvLedError(state == ERROR);

        drvKeypadNavState = kbEncSet;
    }

    //drvLedDebug(drvKeypadNavCwTurns != 0 ||
    //            drvKeypadNavCcwTurns != 0);
}


/* END drvKeypad */
