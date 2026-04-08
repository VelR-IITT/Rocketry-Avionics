#include "GSM.h"
#include <Arduino.h>
#include <string.h>

// Set up the Core Supporting functions
// PARSER - Only reads the and interpret the data recieved
// DISPATCHER - Only decides what to do next, changes the states depending on the event
// FSM (Free State Machine) - Only executes the state assigned to the GSM


// setup of objects/struct defined
// **To be attached in the .h file of the same GSM**

//Ring Buffer Structure
#define GSM_RB_SIZE 512 //Size of ring buffer given is 512 bytes

typedef struct {
    volatile uint8_t buf[GSM_RB_SIZE];
    volatile uint16_t head, tail;
} gsm_rb_t;

gsm_rb_t gsm_rb; // ring buffer variable declaration

// Ring Buffer Functions
// Using the inline method, though it occupies space, this will make faster execution

//Pushing the content to the ring buffer defined
inline void gsm_rb_push(uint8_t c) {
    uint16_t next = (gsm_rb.head + 1) & (GSM_RB_SIZE - 1);
    if (next != gsm_rb.tail) {
        gsm_rb.buf[gsm_rb.head] = c;
        gsm_rb.head = next;
    }
}

//Reading the content from the ring buffer defined
inline int gsm_rb_read(uint8_t *c) {
    if (gsm_rb.head == gsm_rb.tail) return 0;
    *c = gsm_rb.buf[gsm_rb.tail];
    gsm_rb.tail = (gsm_rb.tail + 1) & (GSM_RB_SIZE - 1);
    return 1;
}

//Defining the GSM events through enum constants
typedef enum {
  GSM_EVT_NONE, //Event where there is no valid event (eg: noise, empty line)
  GSM_EVT_OK, //Event when the response received is OK
  GSM_EVT_ERROR,  //Event when the response received is ERROR
  GSM_EVT_PROMPT, //Event when the response received is > (which means in sending the sms, I am ready to recieve SMS text now)
  GSM_EVT_CMTI, //Event when new SMS notification has been recieved
  GSM_EVT_CMGR_HEADER,  //Event when CMGR header comes (+CMGR: "REC UNREAD","+919876543210") which is just before accepting the message
  GSM_EVT_CMGR_BODY,  //Event after the CMGR Header, actual sms content.
  GSM_EVT_CMGS_ACK  //Event where the SMS is successfully sent
} gsm_event_def;

/*
=============================================
SENDING SMS

AT+CMGS="+91xxx"
↓
>              → GSM_EVT_PROMPT
↓
message + Ctrl+Z
↓
+CMGS: 45      → GSM_EVT_CMGS_ACK
↓
OK             → GSM_EVT_OK
=============================================
RECIEVING SMS

+CMTI: "SM",3         → GSM_EVT_CMTI
↓
AT+CMGR=3
↓
+CMGR header          → GSM_EVT_CMGR_HEADER
↓
message body          → GSM_EVT_CMGR_BODY
↓
OK                    → GSM_EVT_OK
*/

typedef struct {
    gsm_event_def type;
    int index;
    char number[16]; // 13 (+91 included) + 1 (Null terminator) + 2 (Extra margins to avoid risks)
    char message[256];
} gsm_msg_t;

// GSM State Definition and declaration 
typedef enum {
    GSM_IDLE, //GSM Idle

    // SEND SMS
    GSM_SEND_INIT, //Initialize sending of sms
    GSM_WAIT_OK,  // GSM waiting for response of OK after AT
    GSM_SET_TEXT, // Setting the SMS into Text mode
    GSM_WAIT_TEXT_OK, //Waiting for confirmation of text mode
    GSM_SEND_CMGS,  // GSM prepares to receive message (which means AT+CMGS="+91xxxx" action)
    GSM_WAIT_PROMPT,  //Waiting for > , process of inserting the text in the serial
    GSM_SEND_BODY,  //action of Ctrl+Z, sending the message
    GSM_WAIT_SEND_OK, //Waiting for confirmation of sending the sms

    // RECEIVE SMS
    GSM_READ_SMS, //GSM initiates the reading the incoming text recieved
    GSM_WAIT_CMGR, //GSM waiting for CMGR to give full text

    GSM_DONE, //Operation done successfully sets the gsm to idle and busy to 0
    GSM_ERROR //Something went wrong during the operation, ERROR
} gsm_state_t;

typedef struct {
    gsm_state_t state;
    int sms_index; //Location within the SIM memory where the message is stored

    char phone[20]; // 13 (+91 included) + 1 (Null terminator) + 2 (Extra margins to avoid risks) //Flag this
    char message[256]; // Message list is hardcoded into 256 bytes of storage. Could be changed later //Flag this

    //Actual elements to make this program non-blocking (to be flagged when written) CLEAR REPLACEMENT FOR DELAY FUNCTION WHICH IS BLOCKING PARAMETER
    bool expecting_body; // 1 - when it is header, 0 - not a header //Flag to say when in the process of recieving the sms_text (the first line is header, second line is the actual SMS_text body) this will say that the current line is header
    uint32_t timeout; //Maximum "wait_time" you are willing to wait for receiving a response, without this the gsm_system will freeze in this program
    uint32_t start_time; //Actual start time (used function millis()) for any current operation running
    // Start timer → wait for event → if not received in time → fail safely

    bool busy; // 1 - GSM is doing something (operation in progress), 0 - GSM is idle and ready for another operation
} gsm_t;

//Main function: UART reading through ISR
//PROBLEM FOUND IN INITIALIZING THE SERIAL