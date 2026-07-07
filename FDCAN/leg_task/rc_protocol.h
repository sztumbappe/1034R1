#ifndef __RC_PROTOCOL_H
#define __RC_PROTOCOL_H

#include "main.h"
#include <stdint.h>

/* ======================== 配置参数 ======================== */
#define RC_RX_DMA_SIZE      256     /* DMA 接收缓冲区大小 */
#define RC_CMD_MAX_LEN      64      /* 单条指令最大长度 */
#define RC_ACK_TIMEOUT_MS   300     /* ACK 超时时间 (ms) */
#define RC_ACK_MAX_RETRY    3       /* 最大重试次数 */

/* ======================== 指令类型枚举 ======================== */
typedef enum {
    RC_CMD_NONE = 0,        /* 无指令 */
    RC_CMD_KFS,             /* 开场定位: RCKFSx,yEND */
    RC_CMD_ATAKE,           /* 左臂Take: RCATAKE00END / RCATAKE01END */
    RC_CMD_BTAKE,           /* 右臂Take: RCBTAKE00END / RCBTAKE01END */
    RC_CMD_RKFS,            /* 右臂高度: RCRKFS100END / 200 / 400 / 600 */
    RC_CMD_LKFS,            /* 左臂高度: RCLKFS100END / 200 / 400 / 600 */
    RC_CMD_RABSORB,         /* 右臂吸收: RCRABSORBEND */
    RC_CMD_LABSORB,         /* 左臂吸收: RCLABSORBEND */
    RC_CMD_RSWITCH,         /* 右臂切换: RCRSWITCHEND */
    RC_CMD_LSWITCH,         /* 左臂切换: RCLSWITCHEND */
    RC_CMD_RRISING,         /* 右臂上升: RCRRISINGEND */
    RC_CMD_LRISING,         /* 左臂上升: RCLRISINGEND */
    RC_CMD_RGODOWN,         /* 右臂下降: RCRGODOWNEND */
    RC_CMD_LGODOWN,         /* 左臂下降: RCLGODOWNEND */
    RC_CMD_UPLIFT,          /* 抬升切换: RCUPLIFT0END / RCUPLIFT1END */
    RC_CMD_LRECALL,         /* 左臂收回: RCLRECALLEND */
    RC_CMD_RRECALL,         /* 右臂收回: RCRRECALLEND */
    RC_CMD_LOUTLAY,         /* 左臂放料准备: RCLOUTLAYEND */
    RC_CMD_ROUTLAY,         /* 右臂放料准备: RCROUTLAYEND */
    RC_CMD_R2READY,         /* R2预备姿态: RCR2READYEND */
    RC_CMD_ATREADY,         /* R1进攻姿态: RCATREADYEND */
    RC_CMD_TRIGGER,         /* 触发1号臂放料准备: RCTRIGGEREND */
    RC_CMD_LAUTO,           /* 左Auto: RCLAUTO00END/01END */
    RC_CMD_RAUTO,           /* 右Auto: RCRAUTO00END/01END */
} rc_cmd_type_t;

/* ======================== 指令数据结构 ======================== */
typedef struct {
    rc_cmd_type_t type;     /* 指令类型 */
    uint16_t      param1;   /* 参数1: zone / take动作 / height / uplift值 */
    uint8_t       param2;   /* 参数2: level (仅 KFS 使用) */
} rc_cmd_t;

/* ======================== 外部变量 ======================== */
extern UART_HandleTypeDef huart6;   /* USART6 句柄 (usart.c 中定义) */

/* ======================== 接口函数 ======================== */

/**
 * @brief 通信模块初始化
 *        启动 DMA 接收，初始化状态机和发送控制
 * @note  必须在 MX_USART6_UART_Init() 之后调用
 */
void rc_protocol_init(void);

/**
 * @brief 获取最新接收到的指令
 * @return 指向指令数据的指针 (调用 rc_clear_new_cmd() 后可再次检查)
 */
rc_cmd_t* rc_get_cmd(void);

/**
 * @brief 检查是否有新指令到达
 * @return 1=有新指令, 0=无
 */
uint8_t rc_has_new_cmd(void);

/**
 * @brief 清除新指令标志
 */
void rc_clear_new_cmd(void);

/**
 * @brief 发送一帧数据 (RC + cmd + END)
 *        内部自动处理半双工锁: 若 wait_ack==1 则拒绝发送
 * @param cmd  ASCII 指令内容 (不含 RC 和 END)
 * @return 0=成功入队, -1=忙 (等待 ACK 中)
 */
int8_t rc_send_frame(const char *cmd);

/**
 * @brief 发送 ACK 应答帧 (RCOKEND)
 */
void rc_send_ack(void);

/**
 * @brief 发送超时轮询 (需周期调用, 建议 10ms 间隔)
 *        检查 ACK 超时并自动重发
 */
void rc_tx_poll(void);

/**
 * @brief 雷达输入映射
 * @param input  雷达原始值 1~12
 * @return 映射后输出 0~9, 超范围返回 -1
 */
int8_t rc_radar_map(uint8_t input);

/* ======================== 内部回调 (自动注册) ======================== */
/**
 * @brief DMA 接收事件回调 (重写 HAL weak 函数)
 *        在 stm32h7xx_it.c 中由 HAL_UARTEx_RxEventCallback 触发
 * @param huart UART 句柄
 * @param Size  本次接收字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief UART DMA 发送完成回调 (重写 HAL weak 函数)
 *        在 DMA 将数据全部发送到 USART 移位寄存器后触发
 * @param huart UART 句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* __RC_PROTOCOL_H */
