#include "gsm.h"
#include <string.h>
#include <stdio.h>
#include "queue.h"

// =====================================================
// INTERNAL TYPES
// =====================================================

//  FSM states 
//
//  Send path:
//    IDLE → SEND_AT → WAIT_AT_OK
//         → SET_TEXT → WAIT_TEXT_OK
//         → SEND_CMGS → WAIT_PROMPT
//         → SEND_BODY → WAIT_SEND_OK → IDLE
//
//  Receive path:
//    IDLE → READ_SMS → WAIT_CMGR → IDLE

typedef enum {

    ST_IDLE,

    // — outbound —
    ST_SEND_AT,
    ST_WAIT_AT_OK,
    ST_SET_TEXT,
    ST_WAIT_TEXT_OK,
    ST_SEND_CMGS,
    ST_WAIT_PROMPT,
    ST_SEND_BODY,
    ST_WAIT_SEND_OK,

    // — inbound —
    ST_READ_SMS,
    ST_WAIT_CMGR

} gsm_state_t;

// parser events 

typedef enum {
    EVT_NONE,
    EVT_OK,
    EVT_ERROR,
    EVT_PROMPT,
    EVT_CMTI,
    EVT_CMGR_HEADER,
    EVT_CMGR_BODY,
    EVT_CMGS_ACK
} gsm_evt_type_t;

typedef struct {
    gsm_evt_type_t type;
    int            index;
    char           number[20];
    char           message[GSM_LINE_MAX];
} gsm_evt_t;

//  TX request (caller to GSM_IO) 

typedef struct {
    char phone[20];
    char message[160];
} gsm_tx_req_t;

// ─── callback notification (GSM_IO → GSM_CB) ─────────

typedef enum {
    CB_SMS_RECEIVED,
    CB_SMS_SENT
} gsm_cb_type_t;    // Callback type

typedef struct {
    gsm_cb_type_t type;
    int           msg_id;
    char          number[20];
    char          message[GSM_LINE_MAX];
} gsm_cb_msg_t; //Callbakc message structure

// ─── driver context — owned only by GSM_IO ───────────

typedef struct {
    gsm_state_t state;
    char        tx_phone[20];
    char        tx_message[160];
    int         retries;
    char        rx_number[20];
    bool        expecting_body;
    TickType_t  state_tick;
    int         pending_rx_index;
} gsm_ctx_t;    //GSM Context

// =====================================================
// MODULE GLOBALS
// =====================================================

static gsm_ctx_t     ctx;
static QueueHandle_t gsm_tx_queue;
/*
this the queue where the gsm module will store the sending of messages request,
it is in the format
*/
static QueueHandle_t gsm_cb_queue;
static QueueHandle_t gsm_rx_idx_queue;

static char     line_buf[GSM_LINE_MAX];
static uint16_t line_len;
static bool     line_overflow;

// =====================================================
// HELPERS
// =====================================================

static inline void enter_wait(gsm_state_t s) {
    ctx.state      = s;
    ctx.state_tick = xTaskGetTickCount();
}

static inline bool timed_out(void) {
    return (xTaskGetTickCount() - ctx.state_tick)
            > pdMS_TO_TICKS(GSM_TIMEOUT_MS);
}

static bool load_next_tx(void) {
    gsm_tx_req_t req;
    if (xQueueReceive(gsm_tx_queue, &req, 0) == pdTRUE) {
        strncpy(ctx.tx_phone,   req.phone,   sizeof(ctx.tx_phone)   - 1);
        strncpy(ctx.tx_message, req.message, sizeof(ctx.tx_message) - 1);
        ctx.tx_phone  [sizeof(ctx.tx_phone)   - 1] = '\0';
        ctx.tx_message[sizeof(ctx.tx_message) - 1] = '\0';
        ctx.retries = 0;
        return true;
    }
    return false;
}

static bool load_next_rx(void) {
    int idx;
    if (xQueueReceive(gsm_rx_idx_queue, &idx, 0) == pdTRUE) {
        ctx.pending_rx_index = idx;
        return true;
    }
    return false;
}

static void post_cb_received(const char *number,
                             const char *message) {    
    gsm_cb_msg_t cb;
    memset(&cb, 0, sizeof(cb)); //  creating a cb variable and clear up any garbage
    
    cb.type = CB_SMS_RECEIVED;
    strncpy(cb.number,  number,  sizeof(cb.number)  - 1);
    strncpy(cb.message, message, sizeof(cb.message) - 1);
    on_sms_received(cb.number, cb.message);
}

static void post_cb_sent(int msg_id) {
    gsm_cb_msg_t cb;
    memset(&cb, 0, sizeof(cb));
    cb.type   = CB_SMS_SENT;
    cb.msg_id = msg_id;
    on_sms_sent(cb.msg_id);
}

// =====================================================
// LINE PARSER
// =====================================================

static gsm_evt_t parse_char(uint8_t c) {

    /*
    First you create the variable for evt, in the gsm_evt_t struct.
    and then you explicitly set the variable content in the memory to 0s to avoid the garbage value error
    
    First you set initially the evt.type to EVT_NONE to depict nothing happening first,
    Next you verify multiple cases you encounter after you get response from the AT

    */

    gsm_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = EVT_NONE;

    if (c == '>') {
        evt.type = EVT_PROMPT;
        return evt;
    }

    if (c == '\r')
        return evt;

    if (c == '\n') {

        line_buf[line_len] = '\0';

        if (line_overflow) {
            line_len      = 0;
            line_overflow = false;
            return evt;
        }

        if (line_len == 0)
            return evt;

        if (strcmp(line_buf, "OK") == 0) {
            evt.type = EVT_OK;
        }
        else if (strcmp(line_buf, "ERROR") == 0 ||
                 strncmp(line_buf, "+CMS ERROR:", 11) == 0 ||
                 strncmp(line_buf, "+CME ERROR:", 11) == 0) {
            evt.type = EVT_ERROR;
            // You can send a flag to the variable where you can speicfy the error that has happened
        }
        else if (strncmp(line_buf, "+CMTI:", 6) == 0) {
            evt.type = EVT_CMTI;
            sscanf(line_buf, "+CMTI: \"%*[^\"]\",%d", &evt.index);
        }
        else if (strncmp(line_buf, "+CMGR:", 6) == 0) {
            evt.type = EVT_CMGR_HEADER;
            sscanf(line_buf,
                   "+CMGR: \"%*[^\"]\",\"%19[^\"]\"",
                   evt.number);
        }
        else if (strncmp(line_buf, "+CMGS:", 6) == 0) {
            evt.type = EVT_CMGS_ACK;
            sscanf(line_buf, "+CMGS: %d", &evt.index);
        }
        else if (ctx.expecting_body) {
            evt.type = EVT_CMGR_BODY;
            strncpy(evt.message, line_buf, sizeof(evt.message) - 1);
            ctx.expecting_body = false;
        }

        line_len = 0;
        return evt;
    }

    if (!line_overflow) {
        if (line_len < GSM_LINE_MAX - 1) {
            line_buf[line_len++] = (char)c;
        } else {
            line_len      = 0;
            line_overflow = true;
        }
    }

    return evt;
}

// =====================================================
// FSM — EVENT HANDLER
// =====================================================

static void fsm_handle(const gsm_evt_t *evt) {

    // Buffer incoming SMS index regardless of TX state
    if (evt->type == EVT_CMTI) {
        int idx = evt->index;
        xQueueSend(gsm_rx_idx_queue, &idx, 0);
    }

    switch (ctx.state) {

    case ST_WAIT_AT_OK:
        if (evt->type == EVT_OK) {
            GSM_SERIAL.println("AT+CMGF=1");
            enter_wait(ST_WAIT_TEXT_OK);
        } else if (evt->type == EVT_ERROR || timed_out()) {
            ctx.state = ST_SEND_AT; // retry
        }
        break;

    case ST_WAIT_TEXT_OK:
        if (evt->type == EVT_OK) {
            GSM_SERIAL.print("AT+CMGS=\"");
            GSM_SERIAL.print(ctx.tx_phone);
            GSM_SERIAL.println("\"");
            enter_wait(ST_WAIT_PROMPT);
        } else if (evt->type == EVT_ERROR || timed_out()) {
            ctx.state = ST_SEND_AT;
        }
        break;

    case ST_WAIT_PROMPT:
        if (evt->type == EVT_PROMPT) {
            GSM_SERIAL.print(ctx.tx_message);
            GSM_SERIAL.write(0x1A); // Ctrl-Z
            enter_wait(ST_WAIT_SEND_OK);
        } else if (evt->type == EVT_ERROR || timed_out()) {
            ctx.state = ST_SEND_AT;
        }
        break;

    case ST_WAIT_SEND_OK:
        if (evt->type == EVT_CMGS_ACK) {
            post_cb_sent(evt->index);
        } else if (evt->type == EVT_OK) {
            ctx.state = ST_IDLE;
        } else if (evt->type == EVT_ERROR || timed_out()) {
            ctx.state = ST_SEND_AT;
        }
        break;

    case ST_WAIT_CMGR:
        if (evt->type == EVT_CMGR_HEADER) {
            strncpy(ctx.rx_number, evt->number,
                    sizeof(ctx.rx_number) - 1);
            ctx.expecting_body = true;
        } else if (evt->type == EVT_CMGR_BODY) {
            post_cb_received(ctx.rx_number, evt->message);
            ctx.state = ST_IDLE;
        } else if (evt->type == EVT_OK) {
            ctx.state = ST_IDLE; // empty/deleted message
        } else if (evt->type == EVT_ERROR || timed_out()) {
            ctx.state = ST_IDLE;
        }
        break;

    default:
        break;
    }
}

// =====================================================
// FSM — TICK (action states + timeout watchdog)
// =====================================================

static void fsm_tick(void) {

    switch (ctx.state) {

    case ST_SEND_AT:
        if (ctx.retries >= GSM_MAX_RETRIES) {
            ctx.retries = 0;
            ctx.state   = ST_IDLE;
            break;
        }
        ctx.retries++;
        GSM_SERIAL.println("AT");
        enter_wait(ST_WAIT_AT_OK);
        break;

    case ST_WAIT_AT_OK:
    case ST_WAIT_TEXT_OK:
    case ST_WAIT_PROMPT:
    case ST_WAIT_SEND_OK:
    case ST_WAIT_CMGR:
        if (timed_out()) {
            gsm_evt_t fake;
            memset(&fake, 0, sizeof(fake));
            fake.type = EVT_ERROR;
            fsm_handle(&fake);
        }
        break;

    case ST_IDLE:
        // RX has priority over TX
        if (load_next_rx()) {
            ctx.state = ST_READ_SMS;
        } else if (load_next_tx()) {
            ctx.state = ST_SEND_AT;
        }
        break;

    case ST_READ_SMS:
        GSM_SERIAL.print("AT+CMGR=");
        GSM_SERIAL.println(ctx.pending_rx_index);
        ctx.expecting_body = false;
        memset(ctx.rx_number, 0, sizeof(ctx.rx_number));
        enter_wait(ST_WAIT_CMGR);
        break;

    default:
        break;
    }
}

// =====================================================
// GSM_IO TASK  (priority 2)
// Sole owner of the UART, parser, and FSM.
// =====================================================

static void gsm_io_task(void *pv) {

    (void)pv;

    // Let the modem finish its power-on banner
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        bool got_bytes = false;
        while (GSM_SERIAL.available()) {
            uint8_t c   = (uint8_t)GSM_SERIAL.read();
            gsm_evt_t e = parse_char(c);
            if (e.type != EVT_NONE)
                fsm_handle(&e);
            got_bytes = true;
        }
        fsm_tick();
        if (got_bytes) {
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

// =====================================================
// PUBLIC API
// =====================================================

void gsm_init(void) {

    memset(&ctx,     0, sizeof(ctx));   // Set the memory for ctx variable to 0
    memset(line_buf, 0, sizeof(line_buf));  // Set the memory for line_bf
    line_len      = 0;
    line_overflow = false;
    ctx.state     = ST_IDLE;

    gsm_tx_queue     = xQueueCreate(GSM_TX_QUEUE_DEPTH,
                                    sizeof(gsm_tx_req_t));
    gsm_rx_idx_queue = xQueueCreate(GSM_RX_QUEUE_DEPTH,
                                    sizeof(int));

    xTaskCreate(gsm_io_task, "GSM_IO", 4096, NULL, 2, NULL);
}

gsm_err_t gsm_send_sms(const char *number,
                        const char *message) {

    if (!number || !message)
        return GSM_ERR_BAD_PARAM;

    gsm_tx_req_t req;
    memset(&req, 0, sizeof(req));
    strncpy(req.phone,   number,  sizeof(req.phone)   - 1);
    strncpy(req.message, message, sizeof(req.message) - 1);

    return (xQueueSend(gsm_tx_queue, &req, 0) == pdTRUE)
           ? GSM_OK
           : GSM_ERR_QUEUE_FULL;
}
