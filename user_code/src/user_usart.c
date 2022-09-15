
#include "user_usart.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static inline void _sum_rx_buf(uint8_t *buffer, uint8_t *_buf);
static inline void _spdc_uart_protocol(uint8_t *buffer, uint8_t *_buf);
static inline void _test_matlab_model(const uint8_t command,
                                      ExtU_control_flow_T *input);
static inline void _transmit_string(volatile UART_HandleTypeDef *USARTx,
                                    uint8_t *buffer, char *Str);

st_protocol st_ptcl;
st_buffer st_buf;

uint8_t txERR[] = "Something's wrong\r\n";
char CR[] = {'\r'};

void receive_uart_start_it(void)
{
    HAL_UART_Receive_IT(&huart1, &st_buf.u1_buf, 1);
    //HAL_UART_Receive_IT(&huart2, st_buf.lp_buf, LAT_BUF_SIZE);
}

void TxPrintf(char *Form, ...)
{
    TX_LED_ON;

    static char Buff[BUF_SIZE];
    va_list ArgPtr;
    va_start(ArgPtr, Form);
    vsprintf(Buff, Form, ArgPtr);
    va_end(ArgPtr);
    _transmit_string(&huart1, st_buf.usart1.tx, Buff);

    TX_LED_OFF;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    RX_LED_ON;

    if (huart->Instance == USART1)
    {
        _sum_rx_buf(st_buf.usart1.rx, &st_buf.u1_buf);
        _test_matlab_model((const uint8_t)st_buf.u1_buf, control_flow.inputs);
        HAL_UART_Receive_IT(&huart1, &st_buf.u1_buf, 1);
    }

    if (huart->Instance == USART2)
    {
        _spdc_uart_protocol(st_buf.latte_panda.rx, st_buf.lp_buf);
        HAL_UART_Receive_IT(&huart2, st_buf.latte_panda.rx, LAT_BUF_SIZE);
    }

    RX_LED_OFF;
}

static inline void _sum_rx_buf(uint8_t *buffer, uint8_t *_buf)
{
    if (*_buf == '\r' || *_buf == '\n')
        memset((void *)buffer, 0x00, sizeof(uint8_t) * BUF_SIZE);
    else
        strcat((char *)buffer, (char *)_buf);
}

static inline void _spdc_uart_protocol(uint8_t *buffer, uint8_t *_buf)
{
    if (*_buf == 0x55 && *(_buf + 4) == 0x77)
    {
        st_ptcl = *(st_protocol *)_buf;

        control_flow.inputs->Input1.input_angle_r32 = (real_T)st_ptcl.angle;
        control_flow.inputs->flag.steer = st_ptcl.flag.bit.blinker;
        control_flow.inputs->flag.turn = st_ptcl.flag.bit.turn_onoff;
    }

    memset((void *)_buf, 0x00, sizeof(uint8_t) * LAT_BUF_SIZE);
}

static inline void _test_matlab_model(const uint8_t command,
                                      ExtU_control_flow_T *input)
{
    FROM_LATTE *input_val = &(input->Input1);
    FROM_LATTE_FLAG *input_flag = &(input->flag);

    switch (command)
    {
    case 'q':
        input_val->input_velo_r32 += 50;
        break;
    case 'a':
        input_val->input_velo_r32 -= 50;
        break;
    case 'w':
        input_val->input_angle_r32 += (float)10.0;
        break;
    case 's':
        input_val->input_angle_r32 -= (float)10.0;
        break;
    case 'e':
        if (input_flag->turn == 0)
            input_flag->turn = 1;
        else
            input_flag->steer = !input_flag->steer;
        break;
    }
}

static inline void _transmit_string(volatile UART_HandleTypeDef *USARTx,
                                    uint8_t *buffer, char *Str)
{
    uint16_t strCnt = 0;
    while (*Str)
    {
        if (*Str == '\n')
        {
            strncat((char *)buffer, CR, 1);
            strCnt++;
        }
        strncat((char *)buffer, Str++, 1);
        strCnt++;
    }

    HAL_UART_Transmit((UART_HandleTypeDef *)USARTx, (uint8_t *)buffer, strCnt,
                      100);

    memset((void *)buffer, 0x00, sizeof(uint8_t) * BUF_SIZE);
}
