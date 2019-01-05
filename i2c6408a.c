/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp007.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>

/* Example/Board Header files */
#include "Board.h"

#define TASKSTACKSIZE       640

#define TCA6408a_ADDR       0x2120
#define TCA6408a_INPUT      0x0000
#define TCA6408a_OUTPUT     0x0001
#define TCA6408a_POLARITY   0x0002
#define TCA6408a_CONFIG     0x0003


#ifndef Board_TMP_ADDR
#define Board_TMP_ADDR       TCA6408a_ADDR
#endif

#define SYSTEM_FREQ_HZ      I2C_400kHz
#define BAUD_RATE           9600
#define DELAY_US_1_5_BIT    ((SYSTEM_FREQ_HZ/BAUD_RATE) * 1.5)    // approx 62.5us
#define DELAY_US_1_0_BIT    ((SYSTEM_FREQ_HZ/BAUD_RATE) * 1.0)
#define DELAY_US_0_5_BIT    ((SYSTEM_FREQ_HZ/BAUD_RATE) * 0.5)
#define DELAY_1_MS          ((SYSTEM_FREQ_HZ/BAUD_RATE) * 1.5 * 16)    // approx 1ms

static Display_Handle display;
uint8_t         txBuffer[1];
uint8_t         rxBuffer[1];
uint8_t         *txPtr;
uint8_t         *rxPtr;
uint8_t         uart_ready;

// dummy tx command for testing
uint8_t      txCmd[] = {
  0x55,
  0x56,
  0x57,
  0x58,
  0x59,
  0x60
};

void uart_delay(unsigned int delay) {
    unsigned int d;
    for (d = 0; d < delay; ++d) {
        asm("NOP");
    }
}

uint8_t uart_read(I2C_Handle i2c) {
    uint8_t         rxByte;
    I2C_Transaction i2cTransaction;
    uint8_t         status;

    /* read from INPUT */
    txBuffer[0] = TCA6408a_INPUT;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = NULL;
    i2cTransaction.writeCount = 0;
    i2cTransaction.readBuf = txBuffer;
    i2cTransaction.readCount = 1;
    status = I2C_transfer(i2c, &i2cTransaction);
    if (!status) {
        Display_printf(display, 0, 0, "I2C transfer fails!\n");
    }

    // RXD is at P5
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = NULL;
    i2cTransaction.writeCount = 0;
    i2cTransaction.readBuf = &rxByte;
    i2cTransaction.readCount = 1;
    status = I2C_transfer(i2c, &i2cTransaction);
    if (!status) {
        Display_printf(display, 0, 0, "I2C transfer fails!\n");
    }
    rxByte >>= 5;
    return rxByte;
}

void uart_write(I2C_Handle i2c, uint8_t txByte) {
    I2C_Transaction i2cTransaction;
    uint8_t         status;

    /* write to OUTPUT */
    txBuffer[0] = TCA6408a_OUTPUT;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    status = I2C_transfer(i2c, &i2cTransaction);
    if (!status) {
        Display_printf(display, 0, 0, "I2C transfer fails!\n");
    }

    // TXD is at P6
    txByte <<= 6;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = &txByte;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    status = I2C_transfer(i2c, &i2cTransaction);
    if (!status) {
        Display_printf(display, 0, 0, "I2C transfer fails!\n");
    }
}

void invert_polarity(I2C_Handle i2c) {
    I2C_Transaction i2cTransaction;
    uint8_t         status;

    /* write to POLARITY INVERSION register */
    txBuffer[0] = TCA6408a_POLARITY;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    status = I2C_transfer(i2c, &i2cTransaction);
    if (!status) {
        Display_printf(display, 0, 0, "I2C transfer fails!\n");
    }
    // write 1 to invert polarity
    // RTS is at P0
    txBuffer[0] = 0x1;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    status = I2C_transfer(i2c, &i2cTransaction);
    if (!status) {
        Display_printf(display, 0, 0, "I2C transfer fails!\n");
    }
}

int compare_response(uint8_t *data_receive) {
    //compare received data to expect data
    //TBD
    return 1;
}

void timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {
    // interrupt callback code goes here. Minimize processing in interrupt.
    uart_ready = 1;
}

/*
 *  ======== mainThread ========
 */
void *mainThread_6408a(void *arg0)
{
    uint8_t         i, data_val, err, count;
    uint8_t         data_length;
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;
    uint8_t         status;
    GPTimerCC26XX_Params timerParams;
    GPTimerCC26XX_Handle hTimer;

    /* Call driver init functions */
    Display_init();
    GPIO_init();
    I2C_init();

    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Open the HOST display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while (1);
    }

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    Display_printf(display, 0, 0, "Starting the i2ctmp007 example\n");

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }

    /* write to CONFIG register */
    txBuffer[0] = TCA6408a_CONFIG;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    status = I2C_transfer(i2c, &i2cTransaction);
    if (!status) {
        Display_printf(display, 0, 0, "I2C transfer fails!\n");
    }
    // config P5 as input(RXD), P6 as output,(TXD) P0 as output(rts), P1 as input(cts)
    // bit set to 1 is configured as input, to 0 is configured as output
    txBuffer[0] = 0x22;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    status = I2C_transfer(i2c, &i2cTransaction);
    if (!status) {
        Display_printf(display, 0, 0, "I2C transfer fails!\n");
    }

    // setup delay timer to let power stabilize before using the uart
    uart_ready = 0;
    GPTimerCC26XX_Params_init(&timerParams);
    timerParams.mode = GPT_MODE_PERIODIC_UP;
    timerParams.width = GPT_CONFIG_32BIT;
    hTimer = GPTimerCC26XX_open(CC2640R2_LAUNCHXL_GPTIMER0A, &timerParams);
    // one second period
    GPTimerCC26XX_setLoadValue(hTimer, 47999999);
    GPTimerCC26XX_registerInterrupt(hTimer, timerCallback, GPT_INT_TIMEOUT);
    GPTimerCC26XX_start(hTimer);

    while (uart_ready == 0);

    data_length = sizeof(txCmd)/sizeof(uint8_t);
    for (count = 0; count < data_length; count++) {
        //activate RTS
        invert_polarity(i2c);
        //delay for 125 ms
        uart_delay(DELAY_1_MS * 125);

        // txCmd contains data to be sent to satellite chip
        txPtr = &txCmd[i];

        // transmit
        // send start bit
        uart_write(i2c, 0);
        uart_delay(DELAY_US_1_0_BIT);
        for (i = 0; i < 8; i++) {
            //if (((*txPtr >> i) & 0x1) == 1) {
            if (*txPtr & i) {
                uart_write(i2c, 1);
            }
            else {
                uart_write(i2c, 0);
            }
            uart_delay(DELAY_US_1_0_BIT);
        }
        // send stop bit
        uart_write(i2c, 1);
        uart_delay(DELAY_US_1_0_BIT);

        //deactivate RTS
        invert_polarity(i2c);
        //delay for 25 ms
        uart_delay(DELAY_1_MS * 25);

        // receive
        /* wait for start bit */
        while (uart_read(i2c) == 1);

        /* 1.5 bit delay */
        uart_delay(DELAY_US_1_5_BIT);

        /* read byte */
        for (i = 0; i < 8; i++) {
            if (uart_read(i2c) == 1) {
                data_val |= (1 << i);
            }
            uart_delay(DELAY_US_1_0_BIT);
        }

        // check for stop bit
        if (uart_read(i2c) == 1)
            err = 0;
        else
            err = 1;

        uart_delay(DELAY_US_0_5_BIT);

        if (0 == err) {
            // compare the response to expected
            compare_response(&data_val);
        }
    }

    // Deinitialized I2C
    I2C_close(i2c);
    Display_printf(display, 0, 0, "I2C closed!\n");

    return (NULL);
}
