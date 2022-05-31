// The Number OF Servo Motors To Be Used In The Project
#define ACT_NUM  6

const act_cfg_t act_cfg_param[ ACT_NUM ] = {
	[ 0 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.Pin = LL_GPIO_PIN_8,
				// .Mode = LL_GPIO_MODE_OUTPUT,
				// .Speed = LL_GPIO_SPEED_FREQ_LOW,
				// .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
				// .Pull = LL_GPIO_PULL_NO,
				// .Alternate = LL_GPIO_AF_0,
			},
		},
		.TIM_Inst = TIM1,
		.TIM_CCRx = &TIM1->CCR1,
		.TIM_CH = LL_TIM_CHANNEL_CH1,
		.TIM_CLK = 96000000,
	},
	[ 1 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.Pin = LL_GPIO_PIN_9,
			},
		},
		.TIM_Inst = TIM1,
		.TIM_CCRx = &TIM1->CCR2,
		.TIM_CH = LL_TIM_CHANNEL_CH2,
		.TIM_CLK = 96000000,
	},
	[ 2 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.Pin = LL_GPIO_PIN_10,
			},
		},
		.TIM_Inst = TIM1,
		.TIM_CCRx = &TIM1->CCR3,
		.TIM_CH = LL_TIM_CHANNEL_CH3,
		.TIM_CLK = 96000000,
	},
	[ 3 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.Pin = LL_GPIO_PIN_0,
			},
		},
		.TIM_Inst = TIM3,
		.TIM_CCRx = &TIM3->CCR3,
		.TIM_CH = LL_TIM_CHANNEL_CH3,
		.TIM_CLK = 96000000,
	},
	[ 4 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.Pin = LL_GPIO_PIN_4,
			},
		},
		.TIM_Inst = TIM3,
		.TIM_CCRx = &TIM3->CCR1,
		.TIM_CH = LL_TIM_CHANNEL_CH1,
		.TIM_CLK = 96000000,
	},
	[ 5 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.Pin = LL_GPIO_PIN_5,
			},
		},
		.TIM_Inst = TIM3,
		.TIM_CCRx = &TIM3->CCR2,
		.TIM_CH = LL_TIM_CHANNEL_CH2,
		.TIM_CLK = 96000000,
	},
};
