/*
 * udp_task.c
 *
 *  Created on: Jun 27, 2025
 *      Author: tashi
 */

#include "udp_task.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>  // memcpy

#define MC_PRINTF 1  // 0にするとprintf無効化

// LwIP ネットワークインターフェース構造体（MX_LWIP_Init で初期化済み）
extern struct netif gnetif;

// PWM 用 TIM ハンドル（CubeMXで設定済みを extern 宣言）
extern TIM_HandleTypeDef htim1;  // SERVO1 (PB1) 例
extern TIM_HandleTypeDef htim2;  // MD1P (PA0)
extern TIM_HandleTypeDef htim3;  // MD2P (PA3)
extern TIM_HandleTypeDef htim4;  // MD3P (PC7)
extern TIM_HandleTypeDef htim5;  // MD4P (PC6)
extern TIM_HandleTypeDef htim9;  // MD5P (PC8)
extern TIM_HandleTypeDef htim12; // MD6P (PC9)
extern TIM_HandleTypeDef htim8;  // SERVO2 (PB6)
extern TIM_HandleTypeDef htim15; // SERVO3 (PD13)
extern TIM_HandleTypeDef htim16; // SERVO4 (PB8)

// DIR用 GPIO ピン定義
#define MD1D_PORT  GPIOD
#define MD1D_PIN   GPIO_PIN_2
#define MD2D_PORT  GPIOG
#define MD2D_PIN   GPIO_PIN_2
#define MD3D_PORT  GPIOD
#define MD3D_PIN   GPIO_PIN_5
#define MD4D_PORT  GPIOD
#define MD4D_PIN   GPIO_PIN_6
#define MD5D_PORT  GPIOD
#define MD5D_PIN   GPIO_PIN_7
#define MD6D_PORT  GPIOC
#define MD6D_PIN   GPIO_PIN_10

// トランジスタ用 GPIO ピン定義
#define TR1_PORT  GPIOF
#define TR1_PIN   GPIO_PIN_0
#define TR2_PORT  GPIOF
#define TR2_PIN   GPIO_PIN_1
#define TR3_PORT  GPIOF
#define TR3_PIN   GPIO_PIN_15
#define TR4_PORT  GPIOC
#define TR4_PIN   GPIO_PIN_11
#define TR5_PORT  GPIOC
#define TR5_PIN   GPIO_PIN_12
#define TR6_PORT  GPIOF
#define TR6_PIN   GPIO_PIN_14
#define TR7_PORT  GPIOF
#define TR7_PIN   GPIO_PIN_12
#define TR8_PORT  GPIOF
#define TR8_PIN   GPIO_PIN_13

// グローバル変数：制御信号格納用
static double mdd[7]; // 方向指令（0 or 1）
static double mdp[7]; // PWM duty比 (0.0〜1.0)

static struct udp_pcb *udp_control_pcb;

// map関数：入力値を別レンジに線形変換
static int map(int value, int inMin, int inMax, int outMin, int outMax) {
	return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// UDP受信コールバック
static void udp_receive_callback(void *arg, struct udp_pcb *upcb,
		struct pbuf *p, const ip_addr_t *addr, u16_t port) {
	if (p == NULL)
		return;

	// 受信データをコピー（int16_t 19要素想定）
	if (p->len < 19 * sizeof(int16_t)) {
		pbuf_free(p);
		return;
	}

	int16_t data[19];
	memcpy(data, p->payload, sizeof(data));

#if MC_PRINTF
	printf("UDP recv data: ");
	for (int i = 0; i < 19; i++) {
		printf("%d ", data[i]);
	}
	printf("\r\n");
#endif

	// 方向・PWM信号の計算
	for (int i = 1; i <= 6; i++) {
		mdd[i] = (data[i] >= 0) ? 1 : 0;
		mdp[i] = fabsf((float) data[i] / 100.0f);
		if (mdp[i] > 1.0)
			mdp[i] = 1.0; // 範囲制限
	}

	// モータPWM出力設定
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
			(uint32_t)(mdp[1] * __HAL_TIM_GET_AUTORELOAD(&htim2)));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
			(uint32_t)(mdp[2] * __HAL_TIM_GET_AUTORELOAD(&htim3)));
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
			(uint32_t)(mdp[3] * __HAL_TIM_GET_AUTORELOAD(&htim4)));
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,
			(uint32_t)(mdp[4] * __HAL_TIM_GET_AUTORELOAD(&htim5)));
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1,
			(uint32_t)(mdp[5] * __HAL_TIM_GET_AUTORELOAD(&htim9)));
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1,
			(uint32_t)(mdp[6] * __HAL_TIM_GET_AUTORELOAD(&htim12)));

	// モータ方向ピン制御
	HAL_GPIO_WritePin(MD1D_PORT, MD1D_PIN,
			(mdd[1]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2D_PORT, MD2D_PIN,
			(mdd[2]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD3D_PORT, MD3D_PIN,
			(mdd[3]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD4D_PORT, MD4D_PIN,
			(mdd[4]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD5D_PORT, MD5D_PIN,
			(mdd[5]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD6D_PORT, MD6D_PIN,
			(mdd[6]) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// サーボ PWM (data[7]～[10] を角度0～270度から500～2500usに変換)
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
			map(data[7], 0, 270, 500, 2500));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,
			map(data[8], 0, 270, 500, 2500));
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1,
			map(data[9], 0, 270, 500, 2500));
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,
			map(data[10], 0, 270, 500, 2500));

	// トランジスタGPIO制御 (data[11]～[18])
	HAL_GPIO_WritePin(TR1_PORT, TR1_PIN,
			(data[11]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TR2_PORT, TR2_PIN,
			(data[12]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TR3_PORT, TR3_PIN,
			(data[13]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TR4_PORT, TR4_PIN,
			(data[14]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TR5_PORT, TR5_PIN,
			(data[15]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TR6_PORT, TR6_PIN,
			(data[16]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TR7_PORT, TR7_PIN,
			(data[17]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TR8_PORT, TR8_PIN,
			(data[18]) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// 受信バッファ解放
	pbuf_free(p);
}

void udpReceiveTask(void *argument) {
	// IPアドレス設定
	ip_addr_t ipaddr, netmask, gw;
	IP4_ADDR(&ipaddr, 192, 168, 0, 217);
	IP4_ADDR(&netmask, 255, 255, 255, 0);
	IP4_ADDR(&gw, 192, 168, 0, 1);

	netif_set_addr(&gnetif, &ipaddr, &netmask, &gw);
	netif_set_up(&gnetif);

	// UDP PCBの生成とバインド
	udp_control_pcb = udp_new();
	if (udp_control_pcb == NULL) {
		osThreadTerminate(NULL);
	}

	if (udp_bind(udp_control_pcb, IP_ADDR_ANY, 5000) != ERR_OK) {
		udp_remove(udp_control_pcb);
		osThreadTerminate(NULL);
	}

	// 受信コールバック設定
	udp_recv(udp_control_pcb, udp_receive_callback, NULL);

	// タスクループ（受信はコールバックで処理）
	while (1) {
		osDelay(1000);
	}
}
