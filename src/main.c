// =========================================================================
// SECTION: Includes and Basic Type Definitions
// =========================================================================

#include <stm32f0xx.h>
#include "clock_.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* instead of #include <stdbool.h> */
typedef enum {
    false,
    true
} bool;

// =========================================================================
// SECTION: Global Defines, Constrants & Macros
// =========================================================================

#define LOG( msg... ) printf( msg );

#define BUFFER_SIZE 64
#define FIFO_ERROR -1

#define BAUDRATE 115200

#define FIELD_SIZE 100
#define ROWS 10
#define COLS 10
#define IDX(x, y) ((x) * 10 + (y))

// =========================================================================
// SECTION: UART Output Redirection (for printf or LOG)
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
// SECTION: FIFO Setup
// =========================================================================

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
// SECTION: Game State Structure, Message Types & UART Buffer
// =========================================================================

volatile Fifo_t usart_rx_fifo;

const uint8_t USART2_TX_PIN = 2;
const uint8_t USART2_RX_PIN = 3;

typedef struct {
    char buffer[BUFFER_SIZE];
    bool ready;
} MessageBuffer;

typedef enum {
    /* MSG_NONE, */
    MSG_HD_START,
    MSG_HD_CS,
    MSG_HD_BOOM_XY,
    MSG_HD_BOOM_RESULT,
    MSG_HD_SF_ROW,
    /* MSG_INVALID */
} MessageType;

typedef struct {
    char my_field[FIELD_SIZE];
    char enemy_field[FIELD_SIZE];
    char my_shots[FIELD_SIZE];
    char enemy_shots[FIELD_SIZE];
    uint8_t my_checksum[ROWS];
    uint8_t enemy_checksum[ROWS];
    uint8_t enemy_hits;
    uint8_t last_shot_x;
    uint8_t last_shot_y;
    uint8_t total_games_played;
    uint8_t games_won;
    uint8_t games_lost;
    uint8_t parser_x;
    uint8_t parser_y;
    uint8_t parser_row;
    bool i_lost;
} GameState;

// =========================================================================
// SECTION: State Machine Setup
// =========================================================================

void state_init(MessageBuffer* msg, GameState* game);
void state_play(MessageBuffer* msg, GameState* game);
void state_end(MessageBuffer* msg, GameState* game);

void init_new_game(GameState* game);

typedef enum {STATE_INIT, STATE_PLAY, STATE_END} State_Type;
typedef void (*StateFunction)(MessageBuffer*, GameState*);
static StateFunction state_table[] = {
    state_init,
    state_play,
    state_end
};
static State_Type curr_state;

// =========================================================================
// SECTION: Function Prototypes
// =========================================================================

/* Parser */
void fifo_parser(MessageBuffer* msg);
MessageType message_parser(MessageBuffer* msg, GameState* game);

/* Message Handlers */
void handle_hd_start(GameState* game);
void handle_hd_cs(MessageBuffer* msg, GameState* game);
void handle_hd_boom_xy(GameState* game);
void handle_hd_boom_result(MessageBuffer* msg, GameState* game);
void handle_hd_sf_row(MessageBuffer* msg, GameState* game);

/* Game Logic */
void create_my_field(GameState* game);
void attacking_opponent(GameState* game);
bool validate_enemy_cs(GameState* game);

// =========================================================================
// SECTION: Main()
// =========================================================================

int main(void) {

    SystemClock_Config();

    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER  |= 0b10 << (USART2_TX_PIN * 2);
    GPIOA->AFR[0] |= 0b0001 << (USART2_TX_PIN * 4);

    GPIOA->MODER  |= 0b10 << (USART2_RX_PIN * 2);
    GPIOA->AFR[0] |= 0b0001 << (USART2_RX_PIN * 4);

    USART2-> BRR = (APB_FREQ / BAUDRATE);
    USART2->CR1 |= 0b1 << 2;
    USART2->CR1 |= 0b1 << 3;
    USART2->CR1 |= 0b1 << 0;
    USART2->CR1 |= 0b1 << 5;

    NVIC_SetPriorityGrouping(0);
    uint32_t uart_pri_encoding = NVIC_EncodePriority(0, 1, 0);
    NVIC_SetPriority(USART2_IRQn, uart_pri_encoding);
    NVIC_EnableIRQ(USART2_IRQn);

    fifo_init((Fifo_t *)&usart_rx_fifo);
    MessageBuffer usart_msg;

    GameState game;
    
    init_new_game(&game);

    while (1) {
        fifo_parser(&usart_msg);
        state_table[curr_state](&usart_msg, &game);
    }

    return 0;
}

// =========================================================================
// SECTION: Interrupt Handler
// =========================================================================

void USART2_IRQHandler(void) {
    if (USART2->ISR & USART_ISR_RXNE) {
        uint8_t c = USART2->RDR;
        fifo_put((Fifo_t *)&usart_rx_fifo, c);
    }
}

// =========================================================================
// SECTION: Parser
// =========================================================================

void fifo_parser(MessageBuffer* msg) 
{    
    static char temp_msg_buffer[BUFFER_SIZE];
    static uint8_t index = 0;
    uint8_t byte;

    while (!fifo_is_empty(&usart_rx_fifo)) {
        if (fifo_get(&usart_rx_fifo, &byte) == 0) {
            if (byte == '\r') continue;
            if (byte == '\n') {
                temp_msg_buffer[index] = '\0';
                strcpy(msg->buffer, temp_msg_buffer);
                msg->ready = true;
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

MessageType message_parser(MessageBuffer* msg, GameState* game) 
{
    /* HD_START */
    if (strcmp(msg->buffer, "HD_START") == 0) {
        return MSG_HD_START;
    }

    /* HD_CS_{xxxxxxxxxx} */
    if (strncmp(msg->buffer, "HD_CS_", 6) == 0 && strlen(msg->buffer) == 16) {
        /* saving enemy checksum */
        for (uint8_t i = 0; i < ROWS; i++) {
            game->enemy_checksum[i] = msg->buffer[i + 6] - '0';
        }
        return MSG_HD_CS;
    }

    /* HD_BOOM_{x}_{y} */
    if (strncmp(msg->buffer, "HD_BOOM_", 8) == 0 && strlen(msg->buffer) == 11 &&
        msg->buffer[8] >= '0' && msg->buffer[8] <= '9' &&
        msg->buffer[10] >= '0' && msg->buffer[10] <= '9') {

        game->parser_x = msg->buffer[8] - '0';
        game->parser_y = msg->buffer[10] - '0';
        return MSG_HD_BOOM_XY;
    }

    /* HD_BOOM_{H/M} */
    if (strncmp(msg->buffer, "HD_BOOM_", 8) == 0 && strlen(msg->buffer) == 9 &&
        (msg->buffer[8] == 'H' || msg->buffer[8] == 'M')) {
        return MSG_HD_BOOM_RESULT;
    }

    if (strncmp(msg->buffer, "HD_SF_", 6) == 0 &&
        msg->buffer[6] >= '0' && msg->buffer[6] <= '9' &&
        msg->buffer[7] == 'D' && strlen(msg->buffer) == 18) {

        game->parser_row = msg->buffer[6] - '0';
        return MSG_HD_SF_ROW;
    }

    /* return MSG_INVALID; */
}

// =========================================================================
// SECTION: FSM States
// =========================================================================

void state_init (MessageBuffer* msg, GameState* game) {
    if (!msg->ready) return;
    
    MessageType type = message_parser(msg, game);

    switch (type) {
        case MSG_HD_START:
            handle_hd_start(game);   
            break;

        case MSG_HD_CS:
            handle_hd_cs(msg, game);
            break;

        default:
            break;
    }
    
    msg->ready = false;
}

void state_play(MessageBuffer* msg, GameState* game) {
    if (!msg->ready) return;

    MessageType type = message_parser(msg, game);

    switch (type) {
        case MSG_HD_BOOM_XY:
            handle_hd_boom_xy(game);
            break;

        case MSG_HD_BOOM_RESULT:
            handle_hd_boom_result(game, msg);
            break;

        case MSG_HD_SF_ROW:
            handle_hd_sf_row(game, msg);
            break;

        default:
            break;
    }

    msg->ready = false;
}

void state_end(MessageBuffer* msg, GameState* game) {
    if (game->i_lost) {
        if (!msg->ready) return;

        MessageType type = message_parser(msg, game);

        if (type == MSG_HD_SF_ROW) {
            handle_hd_sf_row(game, msg);

            if (game->parser_row == 9) {
                bool valid = validate_enemy_cs(game);
                game->total_games_played++;
                init_new_game(game);
                curr_state = STATE_INIT;
            }
        }

        msg->ready = false;
    } else {
        for (uint8_t row = 0; row < 10; row++) {
            LOG("DH_SF%dD", row);
            for (uint8_t col = 0; col < 10; col++) {
                LOG("%c", game->my_field[IDX(row, col)]);
            }
            LOG("\r\n");
        }

        game->games_won++;
        game->total_games_played++;

        init_new_game(game);
        curr_state = STATE_INIT;
    }
}

void init_new_game(GameState* game) {
    memset(game->my_field, '0', FIELD_SIZE);
    memset(game->enemy_field, '0', FIELD_SIZE);
    memset(game->my_shots, '0', FIELD_SIZE);
    memset(game->enemy_shots, '0', FIELD_SIZE);

    memset(game->my_checksum, 0, ROWS);
    memset(game->enemy_checksum, 0, ROWS);

    game->enemy_hits = 0;
    game->last_shot_x = 0;
    game->last_shot_y = 0;
    game->i_lost = false;
}

// =========================================================================
// SECTION: Message Handlers
// =========================================================================

void handle_hd_start(GameState* game) {
    LOG("DH_START_MAX\r\n");
    create_my_field(game);
    curr_state = STATE_INIT;
}

void handle_hd_cs(MessageBuffer* msg, GameState* game) {
    for (uint8_t i = 0; i < ROWS; i++) {
        game->enemy_checksum[i] = msg->buffer[i + 6] - '0';
    }
    
    LOG("DH_CS_");
    for (int i = 0; i < 10; i++) {
        LOG("%d", game->my_checksum[i]);
    }
    LOG("\r\n");
    curr_state = STATE_PLAY;
}

void handle_hd_boom_xy(GameState* game) {
    uint8_t x = game->parser_x;
    uint8_t y = game->parser_y;
    uint8_t index = IDX(x, y);

    if (game->my_field[index] == '0') {
        LOG("DH_BOOM_M\r\n");
        game->enemy_shots[index] = 'M';
    } else {
        game->enemy_shots[index] = 'H';
        game->enemy_hits++;
        
        if (game->enemy_hits == 30) {
            game->i_lost = true;
            for (uint8_t row = 0; row < 10; row++) {
                LOG("DH_SF%dD", row);
                for (uint8_t col = 0; col < 10; col++) {
                    LOG("%c", game->my_field[IDX(row, col)]);
                }
                LOG("\r\n");
            }

            curr_state = STATE_END;
            return;
        }

        LOG("DH_BOOM_H\r\n");
    }

    attacking_opponent(game);
    curr_state = STATE_PLAY;
}

void handle_hd_boom_result(MessageBuffer* msg, GameState* game) {
    uint8_t x = game->last_shot_x;
    uint8_t y = game->last_shot_y;
    uint8_t index = IDX(x, y);

    if(msg->buffer[8] == 'H') {
        game->my_shots[index] = 'H';
    } else if (msg->buffer[8] == 'M') {
        game->my_shots[index] = 'M';
    }

    curr_state = STATE_PLAY;
}

void handle_hd_sf_row(MessageBuffer* msg, GameState* game) {
    uint8_t row = game->parser_row;

    for (uint8_t col = 0; col < COLS; col++) {
        game->enemy_field[IDX(row, col)] = msg->buffer[7 + col];
    }

    if (row == 9) { 
        game->i_lost = false;
        curr_state = STATE_END;
    } else {
        curr_state = STATE_PLAY;
    }
}

// =========================================================================
// SECTION: Game Logic
// =========================================================================

void create_my_field(GameState* game) {
    /* game field */
    const char my_field_init[FIELD_SIZE] = {
    '2','2','0','0','0','0','0','2','2','0',
    '0','0','0','0','0','0','0','0','0','0',
    '0','0','0','4','4','4','4','0','2','0',
    '3','0','0','0','0','0','0','0','2','0',
    '3','0','3','3','3','0','0','0','0','0',
    '3','0','0','0','0','0','0','0','0','0',
    '0','0','5','5','5','5','5','0','3','0',
    '0','0','0','0','0','0','0','0','3','0',
    '0','0','0','0','0','2','2','0','3','0',
    '4','4','4','4','0','0','0','0','0','0'
    };
    memcpy(game->my_field, my_field_init, FIELD_SIZE);

    /* checksum */
    for (uint8_t row = 0; row < ROWS; row++) {
        uint8_t cs = 0;
        for (uint8_t col = 0; col < COLS; col++) {
            uint8_t val = game->my_field[row * COLS + col] - '0';
            if (val != 0) {
                cs++;
            }
        }
        game->my_checksum[row] = cs;
    }
}

void attacking_opponent(GameState *game) {
    uint8_t x;
    uint8_t y;
    uint8_t index;

    do {
        x = rand() % 10;
        y = rand() % 10;
        index = IDX(x, y);
    } while (game->my_shots[index] == 'H' || game->my_shots[index] == 'M');

    LOG("DH_BOOM_%d_%d\r\n", x, y);
    
    game->last_shot_x = x;
    game->last_shot_y = y;
}

bool validate_enemy_cs(GameState* game) {
    uint8_t temp_enemy_cs[ROWS];

    for (uint8_t row = 0; row < ROWS; row++) {
        uint8_t cs = 0;
        for (uint8_t col = 0; col < COLS; col++) {
            uint8_t val = game->enemy_field[row * COLS + col] - '0';
            if (val != 0) {
                cs++;
            }
        }
        temp_enemy_cs[row] = cs;
    }
    
    for (uint8_t i = 0; i < ROWS; i++) {
        if (temp_enemy_cs[i] != game->enemy_checksum[i]) {
            return false;
        }
    }

    return true;
}