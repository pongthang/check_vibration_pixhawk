px4_add_module(
	MODULE examples__check_vibration
	MAIN check_vibration
	STACK_MAIN 4096
	SRCS
		check_vibration.c
	DEPENDS
)
