/**
* @brief bla bla bla
*
* @details
* - bla bla bla
* - bla bla bla
*
*/

// =========================================================================
// Libraries

#include <stm32f0xx.h>
#include "clock_.h"
#include <stdio.h>
#include <stdbool.h>
// =========================================================================
// Defines

// LOG() macro instead of printf()
#define LOG( msg... ) printf( msg );

// Baudrate for USART communication
#define BAUDRATE 115200
// battlefield size
#define FIELD_SIZE 100
#define ROWS 10
#define COLS 10
// =========================================================================

// For supporting printf function we override the _write function to redirect the output to UART
int _write(int handle, char* data, int size) {
  // 'handle' is typically ignored in this context, as we're redirecting all output to USART2
  // 'data' is a pointer to the buffer containing the data to be sent
  // 'size' is the number of bytes to send

  int count = size;  // Make a copy of the size to use in the loop

  // Loop through each byte in the data buffer
  while (count--) {
      // Wait until the transmit data register (TDR) is empty,
      // indicating that USART2 is ready to send a new byte
      while (!(USART2->ISR & USART_ISR_TXE)) {
          // Wait here (busy wait) until TXE (Transmit Data Register Empty) flag is set
      }

      // Load the next byte of data into the transmit data register (TDR)
      // This sends the byte over UART
      USART2->TDR = *data++;

      // The pointer 'data' is incremented to point to the next byte to send
  }

  // Return the total number of bytes that were written
  return size;
}

// =========================================================================
// FIFO buffer implementation

#define BUFFER_SIZE 64
#define FIFO_ERROR -1

typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} Fifo_t;

void fifo_init(Fifo_t* fifo) {
    fifo->head = 0; // Initialize head pointer to 0
    fifo->tail = 0; // Initialize tail pointer to 0
}

uint8_t fifo_is_empty(Fifo_t* fifo) {
    return (fifo->head == fifo->tail);  // FIFO is empty if head and tail are equal
}

uint8_t fifo_is_full(Fifo_t* fifo) {
    return ((fifo->head + 1) % BUFFER_SIZE) == fifo->tail; // FIFO is full if incrementing head would equal tail
}

int fifo_put(Fifo_t* fifo, uint8_t data) {
    if (fifo_is_full(fifo)) {   // Check if FIFO is full before inserting
        return FIFO_ERROR;      // Insertion failed (buffer full)
    }

    fifo->buffer[fifo->head] = data;            // Store data at current head position
    fifo->head = (fifo->head + 1) % BUFFER_SIZE;  // Move head forward and wrap around if needed
    return 0;                                   // Insertion successful
}

int fifo_get(Fifo_t* fifo, uint8_t* data) {
    if (fifo_is_empty(fifo)) {  // Check if FIFO is empty before reading
        return FIFO_ERROR;      // Read failed (buffer empty)
    }

    *data = fifo->buffer[fifo->tail];           // Retrieve data at current tail position
    fifo->tail = (fifo->tail + 1) % BUFFER_SIZE;  // Move tail forward and wrap around if needed
    return 0;                                   // Read successful Return 0
}
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
uint8_t checksum[ROWS] = {0};

char enemy_shots[FIELD_SIZE];   // enemy fire
char my_shots[FIELD_SIZE];      // our fire

const uint8_t USART2_TX_PIN = 2; // PA2 is used as USART2_TX
const uint8_t USART2_RX_PIN = 3; // PA3 is used as USART2_RX
// =========================================================================

/**
 * @brief bla bla bla
 * @param bla bla
 * @return bla bla bla
 */
// =========================================================================
// main loop

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

    while (1) 
    {
        fifo_parser();
        if(new_msg_ready && strcmp(msg_buffer, "HD_START") == 0) {
            LOG("DH_START_MAX\r\n");
            // create my_field and checksum
            calculate_checksum();
            new_msg_ready = false;
        }
        if(new_msg_ready && strncmp(msg_buffer, "HD_CS_", 6) == 0) {
            LOG("DH_CS_");
            for (int i = 0; i < 10; i++) {
                LOG("%d", checksum[i]);
            }
            LOG("\r\n");
            new_msg_ready = false;
        }
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
// Parser function

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
        checksum[row] = cs;
    }
}