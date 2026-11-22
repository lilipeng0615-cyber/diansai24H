#include "headfile.h"

static RingQueue_t uart_rx_queue;
static FrameParse_t parse;

void uart0_init(void)
{
    // 先清空接收队列，让解析器从等待第一个帧头字节开始，再开启接收中断。
    ring_queue_init(&uart_rx_queue);
    memset(&parse, 0, sizeof(parse));

    DL_UART_Main_clearInterruptStatus(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void ring_queue_init(RingQueue_t *q)
{
    memset(q, 0, sizeof(RingQueue_t));
}

uint8_t ring_queue_is_empty(RingQueue_t *q)
{
    return (q->head == q->tail);
}

uint8_t ring_queue_is_full(RingQueue_t *q)
{
    uint16_t next_tail = (uint16_t)((q->tail + 1U) % RING_BUF_SIZE);
    return (next_tail == q->head);
}

uint8_t ring_queue_enqueue(RingQueue_t *q, uint8_t data)
{
    // 队列预留一个空位。写指针再前进一步就碰到读指针时，队列已满。
    uint16_t next_tail = (uint16_t)((q->tail + 1U) % RING_BUF_SIZE);
    if (next_tail == q->head) {
        return 1;
    }

    q->buffer[q->tail] = data;
    q->tail = next_tail;
    return 0;
}

uint8_t ring_queue_dequeue(RingQueue_t *q, uint8_t *data)
{
    if (q->head == q->tail) {
        return 1;
    }

    *data = q->buffer[q->head];
    q->head = (uint16_t)((q->head + 1U) % RING_BUF_SIZE);
    return 0;
}

uint16_t ring_queue_get_size(RingQueue_t *q)
{
    uint16_t head = q->head;
    uint16_t tail = q->tail;

    if (tail >= head) {
        return (uint16_t)(tail - head);
    }
    return (uint16_t)(RING_BUF_SIZE - head + tail);
}

/**
 * @brief 从接收队列中逐字节解析数据包，每次最多取出一帧。
 * 帧格式（按接收顺序）：0xAA 0x55 | LEN | DATA（LEN 字节）| SUM | 0xBB 0xCC
 * LEN 和 SUM 各占一个字节。SUM 为 DATA 各字节之和的低 8 位，空数据的 SUM 为 0。
 * @param out_data 接收有效数据的缓冲区，需能容纳 LEN 字节（最多 255 字节）。
 * @param out_len 成功时写入有效数据的字节数。
 * @retval 0 已取出一帧，队列中的后续字节留给下次调用。
 * @retval 1 队列已读空，尚未得到完整有效帧。若已收到半帧，下次调用会接着解析。
 */
uint8_t frame_parse(uint8_t *out_data, uint8_t *out_len)
{
    uint8_t ch;

    while (ring_queue_dequeue(&uart_rx_queue, &ch) == 0U) {
        switch (parse.state) {
            case PARSE_HEAD1:
                if (ch == FRAME_HEAD1) {
                    parse.state = PARSE_HEAD2;
                }
                break;

            case PARSE_HEAD2:
                // 若再次收到 0xAA，就把它当作新的帧头起点，继续等待 0x55。
                if (ch == FRAME_HEAD2) {
                    parse.state = PARSE_LEN;
                    parse.check_sum = 0;
                } else if (ch != FRAME_HEAD1) {
                    parse.state = PARSE_HEAD1;
                }
                break;

            case PARSE_LEN:
                // 长度只计算数据区。长度为 0 时，下一个字节就是校验和。
                parse.data_len = ch;
                parse.data_cnt = 0;
                if (parse.data_len == 0U) {
                    parse.state = PARSE_CHECK;
                } else {
                    parse.state = PARSE_DATA;
                }
                break;

            case PARSE_DATA:
                // 按 LEN 收满数据，即使数据中出现帧头或帧尾字节也按普通数据处理。
                parse.data_buf[parse.data_cnt++] = ch;
                parse.check_sum = (uint8_t)(parse.check_sum + ch);
                if (parse.data_cnt >= parse.data_len) {
                    parse.state = PARSE_CHECK;
                }
                break;

            case PARSE_CHECK:
                // 校验和相符后才检查两字节帧尾，任一检查失败就重新寻找帧头。
                if (ch == parse.check_sum) {
                    parse.state = PARSE_TAIL1;
                } else {
                    parse.state = PARSE_HEAD1;
                }
                break;

            case PARSE_TAIL1:
                if (ch == FRAME_TAIL1) {
                    parse.state = PARSE_TAIL2;
                } else {
                    parse.state = PARSE_HEAD1;
                }
                break;

            case PARSE_TAIL2:
                // 最后一个帧尾字节匹配后才交付数据，避免调用者读到无效帧。
                if (ch == FRAME_TAIL2) {
                    if (parse.data_len > 0U) {
                        memcpy(out_data, parse.data_buf, parse.data_len);
                    }
                    *out_len = parse.data_len;
                    memset(&parse, 0, sizeof(FrameParse_t));
                    return 0;
                }
                parse.state = PARSE_HEAD1;
                break;

            default:
                parse.state = PARSE_HEAD1;
                break;
        }
    }
    return 1;
}

void uart_send_frame(uint8_t *data, uint8_t len)
{
    uint8_t check_sum = 0;
    uint8_t i;

    uart_send_byte(FRAME_HEAD1);
    uart_send_byte(FRAME_HEAD2);
    uart_send_byte(len);

    for (i = 0; i < len; i++) {
        uart_send_byte(data[i]);
        check_sum = (uint8_t)(check_sum + data[i]);
    }

    uart_send_byte(check_sum);
    uart_send_byte(FRAME_TAIL1);
    uart_send_byte(FRAME_TAIL2);
}

void uart_send_byte(uint8_t ch)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            // 中断中读空硬件接收 FIFO，将字节暂存到环形队列，供 frame_parse 解析。
            // 队列已满时丢弃刚读出的字节，保留队列中尚未解析的数据。
            while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
                uint8_t byte = DL_UART_Main_receiveData(UART_0_INST);
                (void)ring_queue_enqueue(&uart_rx_queue, byte);
            }
            break;

        default:
            break;
    }
}
