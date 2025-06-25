// =========================================================================
// SECTION: Includes and Basic Type Definitions
// =========================================================================

#include <stm32f0xx.h>
#include "clock_.h"
#include <stdio.h>      // for printf(), used via LOG() macro
#include <string.h>     // for strcmp(), strcpy(), memset(), memcpy()
#include <stdlib.h>     // for rand()
#include <stdbool.h>    // for bool type (true/false)

// =========================================================================
// SECTION: Global Defines, Constrants & Macros
// =========================================================================

#define LOG( msg... ) printf( msg );    // macro alias for printf(), used for UART logging

#define BUFFER_SIZE 64      // size of FIFO and message buffer
#define FIFO_ERROR -1       // return value for FIFO errors (not actively handled)

#define BAUDRATE 115200     // UART baud rate

#define FIELD_SIZE 100      // game field size (10 rows x 10 columns)
#define ROWS 10             // number of rows
#define COLS 10             // number of columns
#define IDX(x, y) ((x) * 10 + (y))  // macro to convert (x, y) to 1D array index

// =========================================================================
// SECTION: UART Output Redirection (for printf or LOG)
// =========================================================================

/**
 * @brief  Low-level write function used by printf()
 *
 * This function is called automatically by the C standard library
 * whenever printf() or other output functions are used.
 *
 * Parameters are internally determined by the library:
 *  - handle: file descriptor (ignored here, usually 1 = stdout)
 *  - data: pointer to the formatted output string
 *  - size: number of bytes to send
 *
 * We redirect all output to USART2 for UART communication.
 */
int _write(int handle, char* data, int size) {
    int count = size;

    while (count--) {
        // Wait until USART2 is ready to transmit (TXE = Transmit Data Register Empty)
        while (!(USART2->ISR & USART_ISR_TXE)) {
            // busy wait (blocking)
        }

        // Send one character over USART2
        USART2->TDR = *data++;  // send current char, then increment pointer
    }
    
    // Return total number of bytes "written" (as expected by printf())
    return size;
}

// =========================================================================
// SECTION: FIFO Setup
// =========================================================================

/**
 * @brief  FIFO (First-In First-Out) Buffer for UART Receive Handling
 *
 * This simple ring buffer implementation is used to store incoming UART data
 * (received in the interrupt handler). It allows decoupling the
 * receive process (IRQ) from the parsing logic in the main loop.
 */

 /* FIFO buffer structure */
typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    uint16_t head;  // index of next write position
    uint16_t tail;  // index of next read position
} Fifo_t;

/**
 * @brief  Initializes the FIFO buffer by resetting head and tail indices.
 */
void fifo_init(Fifo_t* fifo) {
    fifo->head = 0;
    fifo->tail = 0;
}

/**
 * @brief  Checks if the FIFO is empty.
 * @return 1 if empty, 0 if data is available.
 */
uint8_t fifo_is_empty(Fifo_t* fifo) {
    return (fifo->head == fifo->tail);
}

/**
 * @brief  Checks if the FIFO is full.
 * @return 1 if full, 0 if space is available.
 */
uint8_t fifo_is_full(Fifo_t* fifo) {
    return ((fifo->head + 1) % BUFFER_SIZE) == fifo->tail;
}

/**
 * @brief  Inserts a byte into the FIFO.
 * @param  fifo Pointer to the FIFO structure
 * @param  data Byte to insert
 * @return 0 on success, FIFO_ERROR if buffer is full
 */
int fifo_put(Fifo_t* fifo, uint8_t data) {
    if (fifo_is_full(fifo)) {
        return FIFO_ERROR;  // buffer overflow
    }

    fifo->buffer[fifo->head] = data;    // store byte
    fifo->head = (fifo->head + 1) % BUFFER_SIZE;    // increment head (circular/ring buffer logic)
    return 0;
}

/**
 * @brief  Retrieves one byte from the FIFO.
 * @param  fifo Pointer to the FIFO structure
 * @param  data Pointer to output byte variable
 * @return 0 on success, FIFO_ERROR if buffer is empty
 */
int fifo_get(Fifo_t* fifo, uint8_t* data) {
    if (fifo_is_empty(fifo)) {
        return FIFO_ERROR;  // no data available
    }

    *data = fifo->buffer[fifo->tail];   // read byte
    fifo->tail = (fifo->tail + 1) % BUFFER_SIZE;    // increment tail (circular/ring buffer logic)
    return 0;
}

// =========================================================================
// SECTION: Game State Structure, Message Types & UART Buffer
// =========================================================================

/**
 * @brief Global UART receive FIFO buffer.
 *
 * Declared as 'volatile' because it is accessed both in the main program flow
 * (e.g., in fifo_parser()) and asynchronously in the USART2 interrupt handler.
 *
 * 'volatile' prevents the compiler from optimizing out reads/writes,
 * ensuring correct behavior in concurrent access contexts (e.g., interrupts).
 */
volatile Fifo_t usart_rx_fifo;

const uint8_t USART2_TX_PIN = 2;
const uint8_t USART2_RX_PIN = 3;

/* Buffer structure used to store and flag complete UART messages */
typedef struct {
    char buffer[BUFFER_SIZE];   // stores the parsed message string
    bool ready;     // flag: true when message is complete and ready to process
} MessageBuffer;

/* Enum for supported message types received via UART */
typedef enum {
    MSG_HD_START,
    MSG_HD_CS,
    MSG_HD_BOOM_XY,
    MSG_HD_BOOM_RESULT,
    MSG_HD_SF_ROW,
    /* MSG_INVALID */   // could be added for error handling
} MessageType;

/* Enum for representing shot results */
typedef enum {
    HIT,
    MISS
} ShotType;

/* Main game state structure containing all field and game progress data */
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
    ShotType last_shot_result;

    bool hunter_mode;
    uint8_t hunter_x;
    uint8_t hunter_y;

    uint8_t parser_x;
    uint8_t parser_y;
    uint8_t parser_row;

    bool last_row;
    bool i_lost;
} GameState;

/* Counter to detect how often the opponent cheated */
int cheat_counter = 0;

// =========================================================================
// SECTION: State Machine Setup
// =========================================================================

/* FSM-State Functions (defined further below) */
void state_init(MessageBuffer*, GameState*);
void state_play(MessageBuffer*, GameState*);
void state_end(MessageBuffer*, GameState*);

/* Resets game state and prepares for a new match */
void init_new_game(MessageBuffer*, GameState*);

/* Enum for FSM states */
typedef enum {STATE_INIT, STATE_PLAY, STATE_END} State_Type;

/* State function pointer type */
typedef void (*StateFunction)(MessageBuffer*, GameState*);

/* Function table for dispatching FSM states */
static StateFunction state_table[] = {
    state_init,     // called, when curr_state == STATE_INIT
    state_play,     // called, when curr_state == STATE_PLAY
    state_end       // called, when curr_state == STATE_END
};

/* current FSM state */
static State_Type curr_state;

// =========================================================================
// SECTION: Function Prototypes
// =========================================================================

/* Parser and Decoder*/
void fifo_parser(MessageBuffer*);
MessageType message_decoder(MessageBuffer*, GameState*);

/* Message Handlers */
void handle_hd_start(GameState*);
void handle_hd_cs(GameState*);
void handle_hd_boom_xy(GameState*);
void handle_hd_boom_result(GameState*);
void handle_hd_sf_row(MessageBuffer*, GameState*);

/* Game Logic */
void print_my_field(GameState*);
void place_ship_and_blocked(GameState*, uint8_t, uint8_t, bool);
bool try_place_ship(GameState*, uint8_t);
void create_my_field(GameState*);
void attacking_opponent(GameState*);
bool validate_enemy_cs(GameState*);

// =========================================================================
// SECTION: Main()
// =========================================================================

int main(void) {
    /* Configure system clock (48 MHz) */
    SystemClock_Config();

    /* UART Setup */

    /* Enable GPIOA (for PA2/PA3) and USART2 peripheral clock */
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* Set PA2 (TX) to alternate function mode (AF1 for USART2) */
    GPIOA->MODER  |= 0b10 << (USART2_TX_PIN * 2);
    GPIOA->AFR[0] |= 0b0001 << (USART2_TX_PIN * 4);

    /* Set PA3 (RX) to alternate function mode (AF1 for USART2) */
    GPIOA->MODER  |= 0b10 << (USART2_RX_PIN * 2);
    GPIOA->AFR[0] |= 0b0001 << (USART2_RX_PIN * 4);

    /* Configure USART2: Baudrate, enable RX/TX, enable USART and RX interrupt */

    /* Set baud rate (Oversampling by 16); USART_BRR = 416 (int) -> Baudrate = APB_FREQ / USART_BRR = 115384.6154 Hz */
    USART2-> BRR = (APB_FREQ / BAUDRATE);
    USART2->CR1 |= 0b1 << 2;    // Enable receiver (RE)
    USART2->CR1 |= 0b1 << 3;    // Enable transmitter (TE)
    USART2->CR1 |= 0b1 << 0;    // Enable USART2 (UE)
    USART2->CR1 |= 0b1 << 5;    // Enable RXNE interrupt (RXNEIE)

    /* NVIC Configuration for USART2 IRQ */
    NVIC_SetPriorityGrouping(0);
    uint32_t uart_pri_encoding = NVIC_EncodePriority(0, 1, 0);
    NVIC_SetPriority(USART2_IRQn, uart_pri_encoding);
    NVIC_EnableIRQ(USART2_IRQn);

    /* Software Structures*/
    fifo_init((Fifo_t *)&usart_rx_fifo);
    MessageBuffer usart_msg;
    GameState game;

    init_new_game(&usart_msg, &game);

    /* Main Program Loop (Finite State Machine) */
    while (1) {
        fifo_parser(&usart_msg);    // parse complete UART message from FIFO
        state_table[curr_state](&usart_msg, &game); // call current FSM state handler
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

/**
 * @brief Parses bytes from UART receive FIFO into complete messages.
 *
 * Reads all available bytes from the FIFO and accumulates them into a temporary buffer
 * until a newline character ('\n') is received, which marks the end of a message.
 * Carriage returns ('\r') are ignored.
 *
 * Once a message is complete, it is copied into the provided MessageBuffer and marked as ready.
 */
void fifo_parser(MessageBuffer* msg) 
{    
    static char temp_msg_buffer[BUFFER_SIZE];   // temporary buffer for assembling message
    static uint8_t index = 0;
    uint8_t byte;

    while (!fifo_is_empty(&usart_rx_fifo)) {
        if (fifo_get(&usart_rx_fifo, &byte) == 0) {
            if (byte == '\r') continue; // ignore carriage return
            if (byte == '\n') {
                temp_msg_buffer[index] = '\0';              // terminate string
                strcpy(msg->buffer, temp_msg_buffer);       // copy message into buffer
                msg->ready = true;                          // mark message as ready
                memset(temp_msg_buffer, 0, BUFFER_SIZE);    // clear temp buffer
                index = 0;
                return;
            }
            if (index < BUFFER_SIZE - 1) {
                temp_msg_buffer[index++] = byte;
            } else {
                index = 0;  // overflow protection: discard message
            }
        }
    }
}

/**
 * @brief Decodes a complete UART message and extracts relevant data.
 *
 * This function identifies the message type based on known prefixes and formats:
 * - HD_START -> handshake
 * - HD_CS_XXXXXXXXXX -> opponent's checksums
 * - HD_BOOM_X_Y -> shot received
 * - HD_BOOM_H / HD_BOOM_M -> result of our shot
 * - HD_SF{row}D{xxxxxxxxxx} -> full field row from opponent
 *
 * The function extracts data (e.g., coordinates, checksums) and stores them in the GameState.
 * Debug messages (e.g., DD_GAMEFIELD) are also handled.
 *
 * @return Corresponding MessageType enum value
 */
MessageType message_decoder(MessageBuffer* msg, GameState* game) 
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
        if (msg->buffer[8] == "H") {
            game->last_shot_result = HIT;
        } else {
            game->last_shot_result = MISS;
        }
        return MSG_HD_BOOM_RESULT;
    }

    /* HD_SF{Row}D{xxxxxxxxxx} */
    if (strncmp(msg->buffer, "HD_SF", 5) == 0) {
        game->parser_row = msg->buffer[5] - '0';
        return MSG_HD_SF_ROW;
    }

    /* Debugging */
    if (strcmp(msg->buffer, "DD_GAMEFIELD") == 0) {
        create_my_field(game);
        print_my_field(game);
    }

    /* additional task */

    /* Count how often opponent's checksum was invalid */
    if (strcmp(msg->buffer, "DD_EVALUATE_CC") == 0) {
        LOG("HOST cheated %d times!\r\n", cheat_counter);
    }

    /* Reset cheat counter */
    if (strcmp(msg->buffer, "DD_RESET_CC") == 0) {
        cheat_counter = 0;
        LOG("Reset of Cheat-Counter was successfull!\r\n");
    }

    /* Unknown or unsupportd message */
    // return MSG_INVALID;
}

// =========================================================================
// SECTION: FSM States
// =========================================================================

/**
 * @brief Handles the initial handshake phase (STATE_INIT).
 *
 * Waits for "HD_START" from the host to begin communication,
 * and then for the checksum ("HD_CS_...") to transition into STATE_PLAY.
 *
 * This function prepares the device for the start of a new game.
 */
void state_init (MessageBuffer* msg, GameState* game) {
    if (!msg->ready) return;
    
    MessageType type = message_decoder(msg, game);

    if (type == MSG_HD_START) {
        handle_hd_start(game);          // respond with DH_START and generate field
        curr_state = STATE_INIT;        // wait for checksum next
    } else if (type == MSG_HD_CS) {
        handle_hd_cs(game);             // send own checksum and enter play phase
        curr_state = STATE_PLAY;
    }

    msg->ready = false;
}

/**
 * @brief Handles the main game loop state (STATE_PLAY).
 *
 * Depending on the received message, this function processes:
 * - shots fired by the opponent (HD_BOOM_XY)
 * - results of our own shots (HD_BOOM_H / M)
 * - incoming enemy field rows at game end (HD_SF{row}D{...})
 *
 * Transitions to STATE_END when the game is over (either side).
 */
void state_play(MessageBuffer* msg, GameState* game) {
    if (!msg->ready) return;

    MessageType type = message_decoder(msg, game);

    if (type == MSG_HD_BOOM_XY) {
        handle_hd_boom_xy(game);
        if (game->i_lost) {
            curr_state = STATE_END;
        } else {
            curr_state = STATE_PLAY;
        }
    } else if (type == MSG_HD_BOOM_RESULT) {
        handle_hd_boom_result(game);
        curr_state = STATE_PLAY;
    } else if (type == MSG_HD_SF_ROW) {
        handle_hd_sf_row(msg, game);
        if (game->last_row) {
            curr_state = STATE_END;
        } else {
            curr_state = STATE_PLAY;
        }
    }

    msg->ready = false;
}

/**
 * @brief Handles the end state (STATE_END) after a win or loss.
 *
 * If we lost, the function waits for the opponent's field rows (HD_SF),
 * then verifies their checksum. If cheating is detected, a counter is incremented.
 * If we won, our field is printed and a new game is initialized.
 */
void state_end(MessageBuffer* msg, GameState* game) {
    if (game->i_lost) {
        if (!msg->ready) return;

        MessageType type = message_decoder(msg, game);

        if (type == MSG_HD_SF_ROW) {
            handle_hd_sf_row(msg, game);

            if (game->last_row) {
                if (validate_enemy_cs(game)) {
                    /* handle Cheating here */
                    cheat_counter++;
                }
                init_new_game(msg, game);
                curr_state = STATE_INIT;
            }
        }

        msg->ready = false;
    } else {
        print_my_field(game);
        init_new_game(msg, game);
        curr_state = STATE_INIT;
    }
}

/**
 * @brief Resets the game state and message buffer for a new match.
 *
 * Clears all internal fields, resets checksums, shot history, and flags.
 * Called at the beginning and after each finished game.
 */
void init_new_game(MessageBuffer* msg, GameState* game) {
    /* Reset MessageBuffer */
    memset(msg->buffer, 0, BUFFER_SIZE);
    msg->ready = false;

    /* Reset GameState */
    memset(game->my_field, '0', FIELD_SIZE);
    memset(game->enemy_field, '0', FIELD_SIZE);
    memset(game->my_shots, '0', FIELD_SIZE);
    memset(game->enemy_shots, '0', FIELD_SIZE);

    memset(game->my_checksum, 0, ROWS);
    memset(game->enemy_checksum, 0, ROWS);

    game->enemy_hits = 0;

    game->last_shot_x = 0;
    game->last_shot_y = 0;

    game->hunter_mode = false;
    game->hunter_x = 0;
    game->hunter_y = 0;

    game->parser_x = 0;
    game->parser_y = 0;
    game->parser_row = 0;

    game->last_row = false;
    game->i_lost = false;
}

// =========================================================================
// SECTION: Message Handlers
// =========================================================================

/**
 * @brief Handles the initial start message from the host (HD_START).
 * Sends device name (DH_START_MAX) and generates a new game field.
 */
void handle_hd_start(GameState* game) {
    LOG("DH_START_MAX\r\n");
    create_my_field(game);
}

/**
 * @brief Responds with the checksum of the generated field (DH_CS_xxxxxxxxxx).
 * The actual checksum was already calculated during field creation.
 */
void handle_hd_cs(GameState* game) {
    // Checksum already saved in message_decoder    
    LOG("DH_CS_");
    for (int i = 0; i < 10; i++) {
        LOG("%d", game->my_checksum[i]);
    }
    LOG("\r\n");
}

/**
 * @brief Handles a shot fired by the opponent (HD_BOOM_x_y).
 * Sends hit/miss response and triggers own shot if not defeated.
 */
void handle_hd_boom_xy(GameState* game) {
    uint8_t x = game->parser_x;
    uint8_t y = game->parser_y;
    uint8_t index = IDX(x, y);

    if (game->my_field[index] == '0') {
        LOG("DH_BOOM_M\r\n");
        if (game->enemy_shots[index] != 'M') {
            game->enemy_shots[index] = 'M';
        }
    } else {
        if (game->enemy_shots[index] != 'H') {
            game->enemy_shots[index] = 'H';
            game->enemy_hits++;
        }
        
        if (game->enemy_hits == 30) {
            game->i_lost = true;
            print_my_field(game);

            return;
        }

        LOG("DH_BOOM_H\r\n");
    }

    attacking_opponent(game);
}

/**
 * @brief Handles the result of our last shot (HD_BOOM_H/M).
 * Updates our own shot tracking and enables hunter mode on hit.
 */
void handle_hd_boom_result(GameState* game) {
    uint8_t x = game->last_shot_x;
    uint8_t y = game->last_shot_y;
    uint8_t index = IDX(x, y);

    if(game->last_shot_result == HIT) {
        game->my_shots[index] = 'H';
        game->hunter_mode = true;
        game->hunter_x = x;
        game->hunter_y = y;
    } else if (game->last_shot_result == MISS) {
        game->my_shots[index] = 'M';
    }
}

/**
 * @brief Processes one row of the opponent's field (HD_SF{row}D{...}).
 * Copies the row data into the local enemy_field array.
 */
void handle_hd_sf_row(MessageBuffer* msg, GameState* game) {
    uint8_t row = game->parser_row;

    for (uint8_t col = 0; col < COLS; col++) {
        game->enemy_field[IDX(row, col)] = msg->buffer[7 + col];
    }

    if (row == 9) { 
        game->last_row = true;
    }
}

// =========================================================================
// SECTION: Game Logic
// =========================================================================

void print_my_field(GameState* game) {
    for (uint8_t row = 0; row < 10; row++) {
            LOG("DH_SF%dD", row);
            for (uint8_t col = 0; col < 10; col++) {
                LOG("%c", game->my_field[IDX(row, col)]);
            }
            LOG("\r\n");
    }
}

void place_ship_and_blocked(GameState* game, uint8_t index, uint8_t size, bool horizontal) {
    bool left = false;
    bool right = false;
    bool above = false;
    bool below = false;

    if (horizontal) {
        /* place ship horizontally */
        for (int i = 0; i < size; i++) {
            game->my_field[index + i] = size + '0';
        }

        /* place blocking left and right (if not at edge) */
        if (index % 10 != 0) {
            game->my_field[index - 1] = 'X';
            left = true;
        }
        if ((index + size - 1) % 10 != 9) {
            game->my_field[index + size] = 'X';
            right = true;
        }

        /* place blocking above */
        if ((index / 10) > 0) {
            for (int i = 0; i < size; i++) {
                game->my_field[index - 10 + i] = 'X';
            }
            above = true;
        }

        /* place blocking below */
        if ((index / 10) < 9) {
            for (int i = 0; i < size; i++) {
                game->my_field[index + 10 + i] = 'X';
            }
            below = true;
        }

        /* block corners around the ship */
        if (left && above) game->my_field[index - 10 - 1] = 'X';
        if (left && below) game->my_field[index + 10 - 1] = 'X';
        if (right && above) game->my_field[index - 10 + size] = 'X';
        if (right && below) game->my_field[index + 10 + size] = 'X';

    } else {  // vertical
        for (int i = 0; i < size; i++) {
            game->my_field[index + (10 * i)] = size + '0';
        }

        /* block above and below */
        if (index / 10 != 0) {
            game->my_field[index - 10] = 'X';
            above = true;
        }
        if (index / 10 != 9) {
            game->my_field[index + size * 10] = 'X';
            below = true;
        }

        /* block left and right columns next to ship */
        if ((index % 10) > 0) {
            for (int i = 0; i < size; i++) {
                game->my_field[index - 1 + (10 * i)] = 'X';
            }
            left = true;
        }
        if ((index % 10) < 9) {
            for (int i = 0; i < size; i++) {
                game->my_field[index + 1 + (10 * i)] = 'X';
            }
            right = true;
        }

        /* block corners */
        if (above && left) game->my_field[index - 10 - 1] = 'X';
        if (above && right) game->my_field[index - 10 + 1] = 'X';
        if (below && left) game->my_field[index + (size * 10) - 1] = 'X';
        if (below && right) game->my_field[index + (size * 10) + 1] = 'X';
    }
}

bool try_place_ship(GameState* game, uint8_t size) {
    uint8_t indices[10] = {0,1,2,3,4,5,6,7,8,9};

    // shuffle row/column indices for randomness
    for (int i = 9; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t tmp = indices[i];
        indices[i] = indices[j];
        indices[j] = tmp;
    }

    bool horizontal = rand() % 2;

    // try each row/column in random order
    for (int k = 0; k < 10; k++) {
        uint8_t fixed = indices[k];

        for (int pass = 0; pass < 2; pass++) {
            horizontal = !horizontal;  // alternate direction

            uint8_t run_start = 0;
            uint8_t run_len = 0;
            uint8_t best_start = 0;
            uint8_t best_len = 0;

            // find largest block of free '0' fields
            for (uint8_t i = 0; i < 10; i++) {
                uint8_t idx = horizontal ? IDX(fixed, i) : IDX(i, fixed);
                if (game->my_field[idx] == '0') {
                    if (run_len == 0) run_start = i;
                    run_len++;
                    if (run_len > best_len) {
                        best_len = run_len;
                        best_start = run_start;
                    }
                } else {
                    run_len = 0;
                }
            }

            // if space is large enough, place ship randomly inside it
            if (best_len >= size) {
                uint8_t offset_range = best_len - size + 1;
                uint8_t offset = rand() % offset_range;
                uint8_t start = best_start + offset;

                uint8_t index = horizontal ? IDX(fixed, start) : IDX(start, fixed);
                place_ship_and_blocked(game, index, size, horizontal);
                return true;
            }
        }
    }

    return false;
}

void create_my_field(GameState* game) {
    memset(game->my_field, '0', FIELD_SIZE);  // clear field

    uint8_t n_battleship = 1;
    uint8_t n_cruiser = 2;
    uint8_t n_destroyer = 3;
    uint8_t n_submarine = 4;

    // try to place each ship type
    while (n_battleship--) try_place_ship(game, 5);
    while (n_cruiser--)    try_place_ship(game, 4);
    while (n_destroyer--)  try_place_ship(game, 3);
    while (n_submarine--)  try_place_ship(game, 2);

    // remove blocking markers
    for (uint8_t i = 0; i < FIELD_SIZE; i++) {
        if (game->my_field[i] == 'X') game->my_field[i] = '0';
    }

    // count ship parts (should be exactly 30)
    uint8_t count = 0;
    for (uint8_t i = 0; i < FIELD_SIZE; i++) {
        if (game->my_field[i] >= '2' && game->my_field[i] <= '5') count++;
    }

    if (count != 30) {
        // optional: error handling or regenerate field
    }

    // calculate checksum for each row
    for (uint8_t row = 0; row < ROWS; row++) {
        uint8_t cs = 0;
        for (uint8_t col = 0; col < COLS; col++) {
            uint8_t val = game->my_field[IDX(row, col)] - '0';
            if (val != 0) cs++;
        }
        game->my_checksum[row] = cs;
    }
}

void attacking_opponent(GameState *game) {
    if (game->hunter_mode) {
        uint8_t x = game->hunter_x;
        uint8_t y = game->hunter_y;

        // try right
        if (y + 1 < 10 && game->my_shots[IDX(x, y + 1)] == '0') {
            game->last_shot_x = x;
            game->last_shot_y = y + 1;
            LOG("DH_BOOM_%d_%d\r\n", x, y + 1);
            return;
        }

        // try left
        if (y > 0 && game->my_shots[IDX(x, y - 1)] == '0') {
            game->last_shot_x = x;
            game->last_shot_y = y - 1;
            LOG("DH_BOOM_%d_%d\r\n", x, y - 1);
            return;
        }

        // try down
        if (x + 1 < 10 && game->my_shots[IDX(x + 1, y)] == '0') {
            game->last_shot_x = x + 1;
            game->last_shot_y = y;
            LOG("DH_BOOM_%d_%d\r\n", x + 1, y);
            return;
        }

        // try up
        if (x > 0 && game->my_shots[IDX(x - 1, y)] == '0') {
            game->last_shot_x = x - 1;
            game->last_shot_y = y;
            LOG("DH_BOOM_%d_%d\r\n", x - 1, y);
            return;
        }

        // no adjacent untried fields → end hunt
        game->hunter_mode = false;
    }

    // find row with highest remaining checksum
    uint8_t best_row = 0;
    uint8_t max_cs = 0;

    for (uint8_t row = 0; row < ROWS; row++) {
        if (game->enemy_checksum[row] > max_cs) {
            max_cs = game->enemy_checksum[row];
            best_row = row;
        }
    }

    // fire in checkerboard pattern
    for (uint8_t col = 0; col < COLS; col++) {
        if ((best_row + col) % 2 != 0) continue;

        uint8_t idx = IDX(best_row, col);
        if (game->my_shots[idx] == '0') {
            game->last_shot_x = best_row;
            game->last_shot_y = col;
            LOG("DH_BOOM_%d_%d\r\n", best_row, col);
            return;
        }
    }

    // fallback: any untried field
    for (uint8_t i = 0; i < FIELD_SIZE; i++) {
        if (game->my_shots[i] == '0') {
            game->last_shot_x = i / 10;
            game->last_shot_y = i % 10;
            LOG("DH_BOOM_%d_%d\r\n", game->last_shot_x, game->last_shot_y);
            return;
        }
    }
}

bool validate_enemy_cs(GameState* game) {
    uint8_t temp_enemy_cs[ROWS];

    /* Calculate checksum (based on field he sent) */
    for (uint8_t row = 0; row < ROWS; row++) {
        uint8_t cs = 0;
        for (uint8_t col = 0; col < COLS; col++) {
            uint8_t val = game->enemy_field[row * 10 + col] - '0';
            if (val != 0) {
                cs++;
            }
        }
        temp_enemy_cs[row] = cs;
    }
    
    /* compare both checksums (start end end) */
    for (uint8_t i = 0; i < ROWS; i++) {
        if (temp_enemy_cs[i] != game->enemy_checksum[i]) {
            return false;
        }
    }

    return true;
}