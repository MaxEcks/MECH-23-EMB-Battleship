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

volatile Fifo_t usart_rx_fifo;

const uint8_t USART2_TX_PIN = 2;
const uint8_t USART2_RX_PIN = 3;

typedef struct {
    char buffer[BUFFER_SIZE];
    bool ready;
} MessageBuffer;

typedef struct {
    char my_field[FIELD_SIZE];
    char enemy_field[FIELD_SIZE];
    char my_shots[FIELD_SIZE];
    char enemy_shots[FIELD_SIZE];
    uint8_t my_checksum[ROWS];
    uint8_t enemy_checksum[ROWS];
    uint8_t enemy_hits;
    int last_shot_x;
    int last_shot_y;
    int total_games_played;
    int games_won;
    int games_lost;
} GameState;

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

void fifo_parser(MessageBuffer* msg);
int calculate_checksum(GameState* game);
int attacking_opponent(GameState* game);

typedef enum {HIT, MISS, LOST, ATTACK_ERROR} AttackedCellType;

AttackedCellType check_game_status(MessageBuffer* msg, GameState* game);
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

    GameState game;
    MessageBuffer usart_msg;

    init_new_game(&game);

    // Initialize my_field with the desired values
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
    memcpy(game.my_field, my_field_init, FIELD_SIZE);

    while (1) {
        fifo_parser(&usart_msg);
        state_table[curr_state](&usart_msg, &game);
    }

    return 0;
}

// =========================================================================

void USART2_IRQHandler(void) {
    static int ret;
    if (USART2->ISR & USART_ISR_RXNE) {
        uint8_t c = USART2->RDR;
        ret = fifo_put((Fifo_t *)&usart_rx_fifo, c);
    }
}
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

void state_init (MessageBuffer* msg, GameState* game){
    // ----- Start-Messages -----
    // Input:  HD_START
    // Output: DH_START_MAX
    if(msg->ready && strcmp(msg->buffer, "HD_START") == 0) {
        LOG("DH_START_MAX\r\n");
        //create_my_field();
        calculate_checksum(game);
        msg->ready = false;

        curr_state = STATE_INIT; // kein State-Wechsel
    }
    // ----- Exchange Checksum -----
    // Input:  HD_CS_{xxxxxxxxxx}
    // Output: DH_CS_{xxxxxxxxxx}
    else if(msg->ready && strncmp(msg->buffer, "HD_CS_", 6) == 0){
        // safe enemy checksum
        for (uint8_t i = 0; i < ROWS; i++) {
            game->enemy_checksum[i] = msg->buffer[i + 6] - '0';
        }
        
        LOG("DH_CS_");
        for (int i = 0; i < 10; i++) {
            LOG("%d", game->my_checksum[i]);
        }
        LOG("\r\n");
        msg->ready = false;

        curr_state = STATE_PLAY;
    }
}

void state_play(MessageBuffer* msg, GameState* game){
    // ----- Enemy lost the Game ----
    // Input:  HD_SF_{row}D{xxxxxxxxxx} (10 times)
    // Output: DH_SF_{row}D{xxxxxxxxxx} (10 times)
    if(msg->ready && strncmp(msg->buffer, "HD_SF", 5) == 0 && msg->buffer[5] - '0' < ROWS){
        // safe enemy field
        uint8_t row = msg->buffer[5] - '0';
        
        // safe_enemy_field();  -> in Funktion auslagern
        for (uint8_t col = 0; col < COLS; col++) {
            game->enemy_field[row * 10 + col] = msg->buffer[7 + col];
        }

        msg->ready = false;

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
    else if(msg->ready && strncmp(msg->buffer, "HD_BOOM_", 8) == 0 && strlen(msg->buffer) == 11){
        // check if we were HIT
        AttackedCellType status = check_game_status(msg, game);
        
        switch (status)
        {
        case MISS:
            LOG("DH_BOOM_M\r\n");
            attacking_opponent(game);
            curr_state = STATE_PLAY;
            break;
        
        case HIT:
            LOG("DH_BOOM_H\r\n");
            attacking_opponent(game);
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

        msg->ready = false;
    }
    // ----- Enemy Hit or Miss -----
    // IN:  HD_BOOM_H or HD_BOOM_M
    if(msg->ready && strncmp(msg->buffer, "HD_BOOM_", 8) == 0 && strlen(msg->buffer) == 9) {
        // H oder M in my_shots schreiben
        if(msg->buffer[8] == 'H') {
            game->my_shots[game->last_shot_x * 10 + game->last_shot_y] = 'H';
        } else {
            game->my_shots[game->last_shot_x * 10 + game->last_shot_y] = 'M';
        }
        msg->ready = false;

        curr_state = STATE_PLAY;
    }
}

void state_end(MessageBuffer* msg, GameState* game){
    return;
    // curr_state = STATE_INIT;
}


void init_new_game(GameState* game)
{
    memset(game->my_field, '0', FIELD_SIZE);
    memset(game->enemy_field, '0', FIELD_SIZE);
    memset(game->my_shots, '0', FIELD_SIZE);
    memset(game->enemy_shots, '0', FIELD_SIZE);
    memset(game->my_checksum, 0, ROWS);
    memset(game->enemy_checksum, 0, ROWS);

    game->enemy_hits = 0;
    game->last_shot_x = 0;
    game->last_shot_y = 0;

    curr_state = STATE_INIT;
}

int calculate_checksum(GameState* game)
{
    for (uint8_t row = 0; row < ROWS; row++) {
        uint8_t cs = 0;
        for (uint8_t col = 0; col < COLS; col++) {
            uint8_t val = game->my_field[row * COLS + col]-'0';
            if (val != 0) {
                cs++;
            }
        }
        game->my_checksum[row] = cs;
    }
    return 0;
}

AttackedCellType check_game_status(MessageBuffer* msg, GameState* game){
    int x = msg->buffer[8] - '0';    // row
    int y = msg->buffer[10] - '0';   // column

    if (msg->buffer[8] < '0' || msg->buffer[8] > '9' || msg->buffer[10] < '0' || msg->buffer[10] > '9') {
        return ATTACK_ERROR;
    }

    if (game->my_field[CELL] == 0) {
        if(game->enemy_shots[CELL] != 'M') {
            game->enemy_shots[CELL] = 'M';
        }

        return MISS;
    } else {
        if(game->enemy_shots[CELL] != 'H') {
            game->enemy_shots[CELL] = 'H';
            game->enemy_hits++;
        }
        
        if (game->enemy_hits == 30) {
            return LOST;
        } else {
            return HIT;
        }
    }
}

int attacking_opponent(GameState *game) {
    int x;
    int y;
    do {
        x = rand() % 10;           // Zeile 0–9
        y = rand() % 10;           // Spalte 0–9
    } while (game->my_shots[CELL] == 'H' || game->my_shots[CELL] == 'M');
    LOG("DH_BOOM_%d_%d\r\n", x, y);
    game->last_shot_x = x;
    game->last_shot_y = y;

    return 0;
}