#include <stm32f0xx.h>
#include "clock_.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// =========================================================================

#define LOG( msg... ) printf( msg );

#define BAUDRATE 115200

#define FIELD_SIZE 100
#define ROWS 10
#define COLS 10
#define CELL (x * 10 + y)

// =========================================================================

int _write(int handle, char* data, int size) {
    int count = size;

    while (count--) {
        while (!(USART2->ISR & USART_ISR_TXE)) {
        }

        USART2->TDR = *data++;
    }
    
    return size;
}

// =========================================================================

#define BUFFER_SIZE 64
#define FIFO_ERROR -1

typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} Fifo_t;

void fifo_init(Fifo_t* fifo) {
    fifo->head = 0;
    fifo->tail = 0;
}

uint8_t fifo_is_empty(Fifo_t* fifo) {
    return (fifo->head == fifo->tail);
}

uint8_t fifo_is_full(Fifo_t* fifo) {
    return ((fifo->head + 1) % BUFFER_SIZE) == fifo->tail;
}

int fifo_put(Fifo_t* fifo, uint8_t data) {
    if (fifo_is_full(fifo)) {
        return FIFO_ERROR;
    }

    fifo->buffer[fifo->head] = data;
    fifo->head = (fifo->head + 1) % BUFFER_SIZE;
    return 0;
}

int fifo_get(Fifo_t* fifo, uint8_t* data) {
    if (fifo_is_empty(fifo)) {
        return FIFO_ERROR;
    }

    *data = fifo->buffer[fifo->tail];
    fifo->tail = (fifo->tail + 1) % BUFFER_SIZE;
    return 0;
}

// =========================================================================

void state_init(void);
void state_play(void);
void state_end(void);

void init_new_game(void);

typedef enum {STATE_INIT, STATE_PLAY, STATE_END} State_Type;
static void (*state_table[])(void)={state_init, state_play, state_end};
static State_Type curr_state; /* The "current state" */

typedef enum {
    HIT,
    MISS,
    LOST,
    ATTACK_ERROR
} AttackedCellType;

// =========================================================================
// global variables

volatile Fifo_t usart_rx_fifo;

volatile bool new_msg_ready = false;    // Parser-flag, if a new msg is ready to be processed

char msg_buffer[BUFFER_SIZE];           // global Array for the msg

uint8_t my_field[100] = {
    2,2,0,0,0,0,0,2,2,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,4,4,4,4,0,2,0,
    3,0,0,0,0,0,0,0,2,0,
    3,0,3,3,3,0,0,0,0,0,
    3,0,0,0,0,0,0,0,0,0,
    0,0,5,5,5,5,5,0,3,0,
    0,0,0,0,0,0,0,0,3,0,
    0,0,0,0,0,2,2,0,3,0,
    4,4,4,4,0,0,0,0,0,0
};
uint8_t enemy_field[100];
uint8_t my_checksum[ROWS] = {0};
uint8_t enemy_checksum[ROWS] = {0};

uint8_t enemy_hits = 0;
int last_shot_x = 0;
int last_shot_y = 0;

char enemy_shots[FIELD_SIZE];   // enemy fire
char my_shots[FIELD_SIZE];      // our fire

const uint8_t USART2_TX_PIN = 2; // PA2 is used as USART2_TX
const uint8_t USART2_RX_PIN = 3; // PA3 is used as USART2_RX
// =========================================================================

void fifo_parser(void);
void calculate_checksum(void);
AttackedCellType check_game_status(void);
void attacking_opponent(void);

// =========================================================================

int main(void) {

    // clock configuration
    SystemClock_Config();   // Configure the system clock to 48 MHz
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;     // Enable GPIOA (PA2, PA3) clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // Enable USART2 clock

    // USART TX pin configuration
    GPIOA->MODER  |= 0b10 << (USART2_TX_PIN * 2);   // Alternate function mode
    GPIOA->AFR[0] |= 0b0001 << (USART2_TX_PIN * 4); // AF1 for USART2_TX (Production Manual Page 41)

    // USART RX pin configuration
    GPIOA->MODER  |= 0b10 << (USART2_RX_PIN * 2);   // Alternate function mode
    GPIOA->AFR[0] |= 0b0001 << (USART2_RX_PIN * 4); // AF1 for USART2_RX (Production Manual Page 41)

    // USART configuration
    USART2-> BRR = (APB_FREQ / BAUDRATE);   // Set baud rate (Oversampling by 16); USART_BRR = 416 (int) -> Baudrate = APB_FREQ / USART_BRR = 115384.6154 Hz
    USART2->CR1 |= 0b1 << 2; // Enable receiver (RE)
    USART2->CR1 |= 0b1 << 3; // Enable transmitter (TE)
    USART2->CR1 |= 0b1 << 0; // Enable USART2 (UE)
    USART2->CR1 |= 0b1 << 5; // Enable RXNE interrupt (RXNEIE)
    /* settings for 8N1 (8 data bits, no parity, 1 stop bit) UART communication -> already set by default
    USART2->CR1 &= ~(0b1 << 15);    // Oversampling by 16 (OVER8)
    USART2->CR2 &= ~(0b11 << 12);   // Set stop bits to 1 (STOP[1:0] = 00)
    USART2->CR1 &= ~(1 << 28);      // M1 = 0
    USART2->CR1 &= ~(1 << 12);      // M0 = 0
    USART2->CR1 &= ~(0b1 << 10);    // Disable parity (PCE)
    */

    NVIC_SetPriorityGrouping(0);                               // Use 4 bits for priority, 0 bits for subpriority
    uint32_t uart_pri_encoding = NVIC_EncodePriority(0, 1, 0); // Encode priority: group 1, subpriority 0
    NVIC_SetPriority(USART2_IRQn, uart_pri_encoding);          // Set USART2 interrupt priority
    NVIC_EnableIRQ(USART2_IRQn);                               // Enable USART2 interrupt

    fifo_init((Fifo_t *)&usart_rx_fifo);                       // Init the FIFO

    init_new_game();

    while (1) 
    {
        fifo_parser();

        state_table[curr_state]();
    }

    return 0;
}
// =========================================================================
// Interrupt handler

void USART2_IRQHandler(void)
{
  static int ret; // You can do some error checking
  if (USART2->ISR & USART_ISR_RXNE)
  {                                              // Check if RXNE flag is set (data received)
    uint8_t c = USART2->RDR;                     // Read received byte from RDR (this automatically clears the RXNE flag)
    ret = fifo_put((Fifo_t *)&usart_rx_fifo, c); // Put incoming Data into the FIFO Buffer for later handling
  }
}
// =========================================================================
// functions

void state_init (void){
    // ----- Start-Messages -----
    // Input:  HD_START
    // Output: DH_START_MAX
    if(new_msg_ready && strcmp(msg_buffer, "HD_START") == 0) {
        LOG("DH_START_MAX\r\n");
        //create_my_field();
        calculate_checksum();
        new_msg_ready = false;

        curr_state = STATE_INIT; // kein State-Wechsel
    }
    // ----- Exchange Checksum -----
    // Input:  HD_CS_{xxxxxxxxxx}
    // Output: DH_CS_{xxxxxxxxxx}
    else if(new_msg_ready && strncmp(msg_buffer, "HD_CS_", 6) == 0){
        // safe enemy checksum
        for (uint8_t i = 0; i < ROWS; i++) {
            enemy_checksum[i] = msg_buffer[i + 6] - '0';
        }
        
        LOG("DH_CS_");
        for (int i = 0; i < 10; i++) {
            LOG("%d", my_checksum[i]);
        }
        LOG("\r\n");
        new_msg_ready = false;

        curr_state = STATE_PLAY;
    }
}

void state_play(void){
    // ----- Enemy lost the Game ----
    // Input:  HD_SF_{row}D{xxxxxxxxxx} (10 times)
    // Output: DH_SF_{row}D{xxxxxxxxxx} (10 times)
    if(new_msg_ready && strncmp(msg_buffer, "HD_SF", 5) == 0 && msg_buffer[5] - '0' < ROWS){
        // safe enemy field
        uint8_t row = msg_buffer[5] - '0';
        
        // safe_enemy_field();  -> in Funktion auslagern
        for (uint8_t col = 0; col < COLS; col++) {
            enemy_field[row * 10 + col] = msg_buffer[7 + col];
        }

        new_msg_ready = false;

        if (row < 9) {
            curr_state = STATE_PLAY;
        } else {
            // calculate_cs(enemy_field);   -> CS des gegnerischen Spielfelds bilden
            // return_field(my_field);      -> Ausgabe meines Spielfelds per UART
            curr_state = STATE_END;
        }
    }
    // ----- Enemy attack -----
    // INPUT:  HD_BOOM_{x}_{y}
    // OUTPUT: DH_BOOM_H/M and DH_BOOM_{x}_{y} OR DH_SF_{row}D{xxxxxxxxxx} (10 times), if we lost the game 
    else if(new_msg_ready && strncmp(msg_buffer, "HD_BOOM_", 8) == 0 && strlen(msg_buffer) == 11){
        // check if we were HIT
        AttackedCellType status = check_game_status();
        
        switch (status)
        {
        case MISS:
            LOG("DH_BOOM_M\r\n");
            attacking_opponent();
            curr_state = STATE_PLAY;
            break;
        
        case HIT:
            LOG("DH_BOOM_H\r\n");
            attacking_opponent();
            curr_state = STATE_PLAY;
            break;

        case LOST:
            // return_field(my_field);
            curr_state = STATE_END;
            break;
        
        case ATTACK_ERROR:
            // Vielleicht späteres Error-Handling implementieren
            break;
        
        default:
            break;
        }

        new_msg_ready = false;
    }
    // ----- Enemy Hit or Miss -----
    // IN:  HD_BOOM_H or HD_BOOM_M
    if(new_msg_ready && strncmp(msg_buffer, "HD_BOOM_", 8) == 0 && strlen(msg_buffer) == 9) {
        // H oder M in my_shots schreiben
        if(msg_buffer[8] == 'H') {
            my_shots[last_shot_x * 10 + last_shot_y] = 'H';
        } else {
            my_shots[last_shot_x * 10 + last_shot_y] = 'M';
        }
        new_msg_ready = false;

        curr_state = STATE_PLAY;
    }
}

void state_end(void){
    return;
    // curr_state = STATE_INIT;
}


void init_new_game(void)
{
    curr_state = STATE_INIT;
    // reset, gamefield, etc. ...
}

void fifo_parser(void)
{
    static char temp_msg_buffer[BUFFER_SIZE];   // static -> variable retains its value between multiple function calls
    static uint8_t index = 0;
    uint8_t byte;

    while (!fifo_is_empty(&usart_rx_fifo)) {
        if (fifo_get(&usart_rx_fifo, &byte) == 0) {
            if (byte == '\r') continue;
            if (byte == '\n') {
                temp_msg_buffer[index] = '\0';
                strcpy(msg_buffer, temp_msg_buffer);
                new_msg_ready = true;
                memset(temp_msg_buffer, 0, BUFFER_SIZE);
                index = 0;
                return;
            }
            if (index < BUFFER_SIZE - 1) {
                temp_msg_buffer[index++] = byte;
            } else {
                index = 0; // Overflow
            }
        }
    }
}

void calculate_checksum(void)
{
    for (uint8_t row = 0; row < ROWS; row++) {
        uint8_t cs = 0;
        for (uint8_t col = 0; col < COLS; col++) {
            uint8_t val = my_field[row * COLS + col];
            if (val != 0) {
                cs++;
            }
        }
        my_checksum[row] = cs;
    }
}

AttackedCellType check_game_status(void){
    int x = msg_buffer[8] - '0';    // row
    int y = msg_buffer[10] - '0';   // column

    if (msg_buffer[8] < '0' || msg_buffer[8] > '9' || msg_buffer[10] < '0' || msg_buffer[10] > '9') {
        return ATTACK_ERROR;
    }

    if (my_field[CELL] == 0) {
        if(enemy_shots[CELL] != 'M') {
            enemy_shots[CELL] = 'M';
        }

        return MISS;
    } else {
        if(enemy_shots[CELL] != 'H') {
            enemy_shots[CELL] = 'H';
            enemy_hits++;
        }
        
        if (enemy_hits == 30) {
            return LOST;
        } else {
            return HIT;
        }
    }
}

void attacking_opponent(void) {
    int x;
    int y;
    do {
        x = rand() % 10;           // Zeile 0–9
        y = rand() % 10;           // Spalte 0–9
    } while (my_shots[CELL] == 'H' || my_shots[CELL] == 'M');
    LOG("DH_BOOM_%d_%d\r\n", x, y);
    last_shot_x = x;
    last_shot_y = y;
}