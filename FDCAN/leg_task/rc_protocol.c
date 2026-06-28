//
// rc_protocol.c
// RC 串口通信协议模块
// 使用 USART6 DMA 接收 + 状态机解析
// 支持半双工锁、超时重发、防粘包
//

#include "rc_protocol.h"
#include "usart.h"
#include <string.h>

/* ======================== DMA 接收缓冲区 (双缓冲 ping-pong) ======================== */
static uint8_t rc_dma_buf[2][RC_RX_DMA_SIZE];
static uint8_t rx_buf_idx = 0;  /* 当前 DMA 正在写入的缓冲区索引 */

/* ======================== RX 状态机 ======================== */
typedef enum {
    RC_STATE_WAIT_R,        /* 等待 'R' */
    RC_STATE_WAIT_C,        /* 等待 'C' */
    RC_STATE_RECV_CMD,      /* 接收指令内容 */
    RC_STATE_RECV_E,        /* 检测到 'E' */
    RC_STATE_RECV_N,        /* 检测到 'N' */
    RC_STATE_RECV_D,        /* 检测到 'D', 帧完成 */
} rc_rx_state_t;

static rc_rx_state_t rx_state = RC_STATE_WAIT_R;
static char    cmd_buffer[RC_CMD_MAX_LEN];     /* 指令内容缓存 */
static uint8_t cmd_len = 0;                     /* 当前指令长度 */

/* ======================== 解析结果 ======================== */
rc_cmd_t latest_cmd;                     /* 最新解析到的指令 */
uint8_t  new_cmd_flag = 0;               /* 新指令标志 */

/* ======================== TX 发送控制 ======================== */
static uint8_t  wait_ack = 0;                   /* 半双工锁: 1=等待ACK */
static uint8_t  retry_count = 0;                /* 已重试次数 */
static uint32_t send_tick = 0;                  /* 发送时刻 (ms) */
static char     pending_cmd[RC_CMD_MAX_LEN];    /* 待确认的指令内容 */
static uint8_t  pending_ack = 0;                /* 延迟ACK标志: 1=需在主循环发送ACK */
static volatile uint8_t  dma_tx_busy = 0;       /* DMA发送忙标志 */
static uint8_t  dma_tx_buf[RC_CMD_MAX_LEN + 8]; /* DMA发送缓冲区 */

/* ======================== 雷达映射表 ======================== */
static const int8_t radar_map_table[13] =
{
    -1,     // 0
     0,     // 1 -> 0
     1,     // 2 -> 1
     2,     // 3 -> 2
     3,     // 4 -> 3
    -1,     // 5 无效
     4,     // 6 -> 4
     5,     // 7 -> 5
    -1,     // 8 无效
     6,     // 9 -> 6
     7,     // 10 -> 7
     8,     // 11 -> 8
     9      // 12 -> 9
};

/* ======================== 内部函数前向声明 ======================== */
static void     rc_rx_feed(uint8_t byte);
static void     rc_rx_process_frame(void);
static void     rc_parse_cmd(const char *buf, uint8_t len);
static uint8_t  parse_uint8(const char *buf, uint8_t len);
static int8_t   match_prefix(const char *buf, uint8_t len, const char *prefix);

/* ======================== 公共接口实现 ======================== */

/**
 * @brief 通信模块初始化
 */
void rc_protocol_init(void)
{
    /* 重置状态机 */
    rx_state = RC_STATE_WAIT_R;
    cmd_len = 0;
    new_cmd_flag = 0;
    memset(&latest_cmd, 0, sizeof(latest_cmd));

    /* 重置发送控制 */
    wait_ack = 0;
    retry_count = 0;
    send_tick = 0;
    pending_ack = 0;
    dma_tx_busy = 0;
    memset(pending_cmd, 0, sizeof(pending_cmd));

    /* 启动 DMA 接收 (buf[0]) */
    rx_buf_idx = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rc_dma_buf[0], RC_RX_DMA_SIZE);
    /* 禁用半传输中断 (HT), 避免过早触发回调 */
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
}

/**
 * @brief 获取最新接收到的指令
 */
rc_cmd_t* rc_get_cmd(void)
{
    return &latest_cmd;
}

/**
 * @brief 检查是否有新指令到达
 */
uint8_t rc_has_new_cmd(void)
{
    return new_cmd_flag;
}

/**
 * @brief 清除新指令标志
 */
void rc_clear_new_cmd(void)
{
    new_cmd_flag = 0;
}

/**
 * @brief 发送 ACK 应答帧 (RCOKEND)
 */
void rc_send_ack(void)
{
    const uint8_t ack[] = "RCOKEND";
    HAL_UART_Transmit(&huart6, (uint8_t *)ack, 7, 50);
}

/**
 * @brief 发送一帧数据 (RC + cmd + END)
 * @param cmd  ASCII 指令内容 (不含 RC 和 END)
 * @return 0=成功, -1=忙
 */
int8_t rc_send_frame(const char *cmd)
{
    /* 半双工锁: 等待 ACK 时禁止发送 */
    if (wait_ack) {
        return -1;
    }

    /* 拼接帧: RC + cmd + END */
    uint8_t frame[RC_CMD_MAX_LEN + 8]; /* RC(2) + cmd + END(3) + \0 */
    uint16_t cmd_len = (uint16_t)strlen(cmd);

    if (cmd_len + 5 >= sizeof(frame)) {
        return -1;  /* 长度溢出 */
    }

    frame[0] = 'R';
    frame[1] = 'C';
    memcpy(&frame[2], cmd, cmd_len);
    frame[2 + cmd_len]     = 'E';
    frame[2 + cmd_len + 1] = 'N';
    frame[2 + cmd_len + 2] = 'D';

    /* 保存待确认指令 (用于超时重发) */
    memcpy(pending_cmd, cmd, cmd_len);
    pending_cmd[cmd_len] = '\0';

    /* 发送 */
    HAL_UART_Transmit(&huart6, frame, 2 + cmd_len + 3, 100);

    /* 锁定发送, 记录时刻 */
    wait_ack = 1;
    retry_count = 0;
    send_tick = HAL_GetTick();

    return 0;
}

/**
 * @brief 发送超时轮询
 */
void rc_tx_poll(void)
{
    /* ---- 处理延迟ACK发送 (保持同步, 仅7字节≈0.6ms) ---- */
    if (pending_ack) {
        pending_ack = 0;
        const uint8_t ack[] = "RCOKEND";
        HAL_UART_Transmit(&huart6, (uint8_t *)ack, 7, 50);
    }

    if (!wait_ack) {
        return;  /* 无需等待 */
    }

    /* DMA 发送中, 跳过本次重试检查 (不阻塞主循环) */
    if (dma_tx_busy) {
        return;
    }

    uint32_t elapsed = HAL_GetTick() - send_tick;

    if (elapsed >= RC_ACK_TIMEOUT_MS) {
        if (retry_count < RC_ACK_MAX_RETRY) {
            /* 重发 (DMA非阻塞) */
            retry_count++;
            send_tick = HAL_GetTick();

            uint16_t len = (uint16_t)strlen(pending_cmd);
            dma_tx_buf[0] = 'R';
            dma_tx_buf[1] = 'C';
            memcpy(&dma_tx_buf[2], pending_cmd, len);
            dma_tx_buf[2 + len]     = 'E';
            dma_tx_buf[2 + len + 1] = 'N';
            dma_tx_buf[2 + len + 2] = 'D';

            dma_tx_busy = 1;
            HAL_UART_Transmit_DMA(&huart6, dma_tx_buf, 2 + len + 3);
        } else {
            /* 超过最大重试次数, 放弃 */
            wait_ack = 0;
            retry_count = 0;
        }
    }
}

/**
 * @brief 雷达输入映射
 */
int8_t rc_radar_map(uint8_t input)
{
    if (input >= 1 && input <= 12) {
        return radar_map_table[input];
    }
    return -1;
}

/* ======================== DMA 回调 ======================== */

/**
 * @brief UART 空闲中断/DMA 接收完成回调
 *        当 DMA 接收到 IDLE 线或缓冲区将满时触发
 * @note  此函数为 HAL weak 函数重写, 在 stm32h7xx_it.c 中自动调用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART6) {
        return;
    }

    /* 双缓冲 ping-pong: 保存当前缓冲区索引, 立即用另一缓冲区重新武装 DMA */
    uint8_t idx = rx_buf_idx;
    rx_buf_idx ^= 1;  /* 切换 0↔1 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rc_dma_buf[rx_buf_idx], RC_RX_DMA_SIZE);
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);

    /* 现在安全地处理已保存缓冲区的数据 (DMA 不会覆盖它) */
    for (uint16_t i = 0; i < Size; i++) {
        rc_rx_feed(rc_dma_buf[idx][i]);
    }
}

/**
 * @brief UART DMA 发送完成回调
 *        在 DMA 将数据全部发送到 USART 移位寄存器后触发
 * @note  此函数为 HAL weak 函数重写
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        dma_tx_busy = 0;
    }
}

/* ======================== 状态机内部实现 ======================== */

/**
 * @brief 逐字节喂入 RX 状态机
 * @param byte  接收到的单字节
 */
static void rc_rx_feed(uint8_t byte)
{
    switch (rx_state) {

    case RC_STATE_WAIT_R:
        if (byte == 'R') {
            rx_state = RC_STATE_WAIT_C;
        }
        break;

    case RC_STATE_WAIT_C:
        if (byte == 'C') {
            rx_state = RC_STATE_RECV_CMD;
            cmd_len = 0;
        } else if (byte == 'R') {
            /* 连续 'R', 保持在 WAIT_C */
            rx_state = RC_STATE_WAIT_C;
        } else {
            rx_state = RC_STATE_WAIT_R;
        }
        break;

    case RC_STATE_RECV_CMD:
        if (byte == 'E') {
            rx_state = RC_STATE_RECV_E;
        } else {
            /* 存入指令缓冲区 */
            if (cmd_len < RC_CMD_MAX_LEN - 1) {
                cmd_buffer[cmd_len++] = (char)byte;
            } else {
                /* 溢出, 重置 */
                rx_state = RC_STATE_WAIT_R;
                cmd_len = 0;
            }
        }
        break;

    case RC_STATE_RECV_E:
        if (byte == 'N') {
            rx_state = RC_STATE_RECV_N;
        } else if (byte == 'R') {
            /* 'E' 可能是数据的一部分, 或新的 RC 开始 */
            /* 回退: 将 'E' 存入缓冲区 */
            if (cmd_len < RC_CMD_MAX_LEN - 1) {
                cmd_buffer[cmd_len++] = 'E';
            }
            rx_state = RC_STATE_WAIT_C;
        } else {
            /* 'E' 是数据, 继续接收 */
            if (cmd_len < RC_CMD_MAX_LEN - 1) {
                cmd_buffer[cmd_len++] = 'E';
            }
            rx_state = RC_STATE_RECV_CMD;
        }
        break;

    case RC_STATE_RECV_N:
        if (byte == 'D') {
            /* 帧完整! */
            cmd_buffer[cmd_len] = '\0';
            rc_rx_process_frame();
            rx_state = RC_STATE_WAIT_R;
            cmd_len = 0;
        } else {
            /* END 不完整, 'N' 是数据 */
            if (cmd_len < RC_CMD_MAX_LEN - 2) {
                cmd_buffer[cmd_len++] = 'E';
                cmd_buffer[cmd_len++] = 'N';
            }
            rx_state = RC_STATE_RECV_CMD;
        }
        break;

    default:
        rx_state = RC_STATE_WAIT_R;
        cmd_len = 0;
        break;
    }
}

/**
 * @brief 帧接收完成, 检查是否为 ACK 或指令
 */
static void rc_rx_process_frame(void)
{
    /* 检查是否为 ACK: RCOKEND */
    if (cmd_len == 2 && cmd_buffer[0] == 'O' && cmd_buffer[1] == 'K') {
        /* 收到 ACK, 解除半双工锁 */
        wait_ack = 0;
        retry_count = 0;
        return;
    }

    /* 合法指令帧, 设置延迟ACK标志 (不在中断里阻塞发送) */
    pending_ack = 1;

    /* 解析指令 */
    rc_parse_cmd(cmd_buffer, cmd_len);
}

/**
 * @brief 解析指令内容, 写入 latest_cmd
 * @param buf  指令内容 (RC 和 END 之间的部分)
 * @param len  指令内容长度
 */
static void rc_parse_cmd(const char *buf, uint8_t len)
{
    /* 转大写，兼容上位机混合大小写 (如 ATake00, RAbsorb 等) */
    char upper[RC_CMD_MAX_LEN];
    uint8_t ulen = (len < RC_CMD_MAX_LEN) ? len : (RC_CMD_MAX_LEN - 1);
    for (uint8_t i = 0; i < ulen; i++) {
        upper[i] = (buf[i] >= 'a' && buf[i] <= 'z') ? (char)(buf[i] - 32) : buf[i];
    }
    upper[ulen] = '\0';
    buf = upper;
    len = ulen;

    rc_cmd_t cmd;
    cmd.type = RC_CMD_NONE;
    cmd.param1 = 0;
    cmd.param2 = 0;

    /* ---- 开场定位: KFSx,y ---- */
    if (match_prefix(buf, len, "KFS") >= 0) {
        /* 查找逗号 */
        uint8_t comma_pos = 0;
        for (uint8_t i = 3; i < len; i++) {
            if (buf[i] == ',') {
                comma_pos = i;
                break;
            }
        }
        if (comma_pos > 3 && comma_pos < len - 1) {
            cmd.type = RC_CMD_KFS;
            cmd.param1 = parse_uint8(buf + 3, comma_pos - 3);
            cmd.param2 = parse_uint8(buf + comma_pos + 1, len - comma_pos - 1);
        }
    }
    /* ---- 左臂 Take: ATAKE00 / ATAKE01 ---- */
    else if (match_prefix(buf, len, "ATAKE") >= 0) {
        cmd.type = RC_CMD_ATAKE;
        cmd.param1 = parse_uint8(buf + 5, len - 5);
    }
    /* ---- 右臂 Take: BTAKE00 / BTAKE01 ---- */
    else if (match_prefix(buf, len, "BTAKE") >= 0) {
        cmd.type = RC_CMD_BTAKE;
        cmd.param1 = parse_uint8(buf + 5, len - 5);
    }
    /* ---- 右臂高度: RKFS100 / 200 / 400 / 600 ---- */
    else if (match_prefix(buf, len, "RKFS") >= 0) {
        cmd.type = RC_CMD_RKFS;
        cmd.param1 = parse_uint8(buf + 4, len - 4);
    }
    /* ---- 左臂高度: LKFS100 / 200 / 400 / 600 ---- */
    else if (match_prefix(buf, len, "LKFS") >= 0) {
        cmd.type = RC_CMD_LKFS;
        cmd.param1 = parse_uint8(buf + 4, len - 4);
    }
    /* ---- 右臂吸收: RABSORB ---- */
    else if (match_prefix(buf, len, "RABSORB") >= 0) {
        cmd.type = RC_CMD_RABSORB;
    }
    /* ---- 左臂吸收: LABSORB ---- */
    else if (match_prefix(buf, len, "LABSORB") >= 0) {
        cmd.type = RC_CMD_LABSORB;
    }
    /* ---- 右臂切换: RSWITCH ---- */
    else if (match_prefix(buf, len, "RSWITCH") >= 0) {
        cmd.type = RC_CMD_RSWITCH;
    }
    /* ---- 左臂切换: LSWITCH ---- */
    else if (match_prefix(buf, len, "LSWITCH") >= 0) {
        cmd.type = RC_CMD_LSWITCH;
    }
    /* ---- 右臂上升: RRISING ---- */
    else if (match_prefix(buf, len, "RRISING") >= 0) {
        cmd.type = RC_CMD_RRISING;
    }
    /* ---- 左臂上升: LRISING ---- */
    else if (match_prefix(buf, len, "LRISING") >= 0) {
        cmd.type = RC_CMD_LRISING;
    }
    /* ---- 右臂下降: RGODOWN ---- */
    else if (match_prefix(buf, len, "RGODOWN") >= 0) {
        cmd.type = RC_CMD_RGODOWN;
    }
    /* ---- 左臂下降: LGODOWN ---- */
    else if (match_prefix(buf, len, "LGODOWN") >= 0) {
        cmd.type = RC_CMD_LGODOWN;
    }
    /* ---- 抬升切换: UPLIFT0 / UPLIFT1 ---- */
    else if (match_prefix(buf, len, "UPLIFT") >= 0) {
        cmd.type = RC_CMD_UPLIFT;
        cmd.param1 = parse_uint8(buf + 6, len - 6);
    }
    /* ---- 左臂收回: LRECALL ---- */
    else if (match_prefix(buf, len, "LRECALL") >= 0) {
        cmd.type = RC_CMD_LRECALL;
    }
    /* ---- 右臂收回: RRECALL ---- */
    else if (match_prefix(buf, len, "RRECALL") >= 0) {
        cmd.type = RC_CMD_RRECALL;
    }
    /* ---- 左臂放料准备: LOUTLAY ---- */
    else if (match_prefix(buf, len, "LOUTLAY") >= 0) {
        cmd.type = RC_CMD_LOUTLAY;
    }
    /* ---- 右臂放料准备: ROUTLAY ---- */
    else if (match_prefix(buf, len, "ROUTLAY") >= 0) {
        cmd.type = RC_CMD_ROUTLAY;
    }
    /* ---- R2预备姿态: R2READY ---- */
    else if (match_prefix(buf, len, "R2READY") >= 0) {
        cmd.type = RC_CMD_R2READY;
    }
    /* ---- R1进攻姿态: ATREADY ---- */
    else if (match_prefix(buf, len, "ATREADY") >= 0) {
        cmd.type = RC_CMD_ATREADY;
    }
    /* ---- 触发放料准备: TRIGGER ---- */
    else if (match_prefix(buf, len, "TRIGGER") >= 0) {
        cmd.type = RC_CMD_TRIGGER;
    }
    /* 更新最新指令 (仅合法指令) */
    if (cmd.type != RC_CMD_NONE) {
        latest_cmd = cmd;
        new_cmd_flag = 1;
    }
}

/* ======================== 工具函数 ======================== */

/**
 * @brief 从 ASCII 字符串解析 uint8 数值
 * @param buf  数字字符串起始指针
 * @param len  数字字符串长度
 * @return 解析结果
 */
static uint8_t parse_uint8(const char *buf, uint8_t len)
{
    uint8_t val = 0;
    for (uint8_t i = 0; i < len; i++) {
        if (buf[i] >= '0' && buf[i] <= '9') {
            val = val * 10 + (uint8_t)(buf[i] - '0');
        } else {
            break;  /* 遇到非数字字符停止 */
        }
    }
    return val;
}

/**
 * @brief 前缀匹配
 * @param buf      待匹配字符串
 * @param len      字符串长度
 * @param prefix   前缀字符串 (以 '\0' 结尾)
 * @return >=0 匹配成功 (前缀长度), -1 不匹配
 */
static int8_t match_prefix(const char *buf, uint8_t len, const char *prefix)
{
    uint8_t plen = (uint8_t)strlen(prefix);
    if (plen > len) {
        return -1;
    }
    for (uint8_t i = 0; i < plen; i++) {
        if (buf[i] != prefix[i]) {
            return -1;
        }
    }
    return (int8_t)plen;
}