#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/test_motor.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>





__EXPORT int check_vibration_main(int argc , char* argv[]);


void motor_test(unsigned channel, float value, uint8_t driver_instance, int timeout_ms)
{
	struct test_motor_s test_motor;
	memset(&test_motor, 0, sizeof(test_motor));
	orb_advert_t test_motor_pub = orb_advertise(ORB_ID(test_motor), &test_motor);
	test_motor.timestamp = hrt_absolute_time();
	test_motor.motor_number = channel;
	test_motor.value = value;
	test_motor.action = value >= 0.f ? 1: 0;
	test_motor.driver_instance = driver_instance;
	test_motor.timeout_ms = timeout_ms;

	orb_publish(ORB_ID(test_motor),  test_motor_pub, &test_motor);

	if (test_motor.action == 0) {
		PX4_INFO("motors stop command sent");

	}
	// else {
	// 	/* Adjust for 1-based motor indexing */
	// 	PX4_INFO("motor %d set to %.2f", channel + 1, (double)value);
	// }
}
int motor_number=0;
int stop_program = 0;
void activate_channel(struct rc_channels_s rc_signal){

	if (0.75<(double)rc_signal.channels[0]&&(double)rc_signal.channels[0]<1.0){
		motor_test(0,-1,0,0);//stop all the running motors
		motor_number=1; // set the motor number according to the stick movement(1)
	}else if (-1.0<(double)rc_signal.channels[0]&&(double)rc_signal.channels[0]<-0.75){
		motor_test(0,-1,0,0);//stop all the running motors
		motor_number=2; // set the motor number according to the stick movement(2)
	}else if (0.75<(double)rc_signal.channels[1]&&(double)rc_signal.channels[1]<1.0){
		motor_test(0,-1,0,0);//stop all the running motors
		motor_number=3; // set the motor number according to the stick movement(3)
	}else if (-1.0<(double)rc_signal.channels[1]&&(double)rc_signal.channels[1]<-0.75){
		motor_test(0,-1,0,0);//stop all the running motors
		motor_number=4; // set the motor number according to the stick movement(4)
	}else if (-1.0<=(double)rc_signal.channels[4]&&(double)rc_signal.channels[4]<-0.75){
		motor_test(0,-1,0,0);//stop all the running motors
		motor_number=5; // set the motor number according to the stick movement(5)
	}else if (0.75<(double)rc_signal.channels[4]&&(double)rc_signal.channels[4]<=1.0){
		motor_test(0,-1,0,0);//stop all the running motors
		motor_number=6; // set the motor number according to the stick movement(6)
	}

	double motor_power=(double)rc_signal.channels[2];
	if (motor_power<0.15 &&(double)rc_signal.channels[3]<-0.75){
		motor_test(0,-1,0,0); // Stop the running motors
		motor_number=0;
		stop_program=1;
		PX4_INFO("Exiting the program");
	}else if (motor_power<0.15){
		motor_test(0,-1,0,0); // Stop the running motors
	}else{
		// PX4_INFO("Motor Testing of :\t%d\t with power %8.4f",
		// 			 motor_number,
		// 			 motor_power);
		motor_test(motor_number-1,motor_power*0.1,0,0);

	}


}


int check_vibration_main(int argc , char* argv[]){

	PX4_INFO("Hello Sky! ");

	// subscribe to the vehicle_acceleration topic

	int sensor_sub_fd=orb_subscribe(ORB_ID(vehicle_acceleration));
	// Subscribe rc signal
	int rc_signal_fd=orb_subscribe(ORB_ID(rc_channels));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 100);
	// limit the update rate to 10 Hz for rc_signal
	orb_set_interval(rc_signal_fd, 100);
	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = rc_signal_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter=0;

	for (int i=0; i<atoi(argv[1]);i++){
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 2, 1000);
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_acceleration_s accel;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_acceleration ), sensor_sub_fd, &accel);
				PX4_INFO("Motor - %d Raw Acceleration:\t%8.4f\t%8.4f\t%8.4f",
					 motor_number,
					 (double)accel.xyz[0],
					 (double)accel.xyz[1],
					 (double)accel.xyz[2]);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/


			}
			if (fds[1].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct rc_channels_s rc;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(rc_channels ), rc_signal_fd, &rc);
				// PX4_INFO("RC:\t%8.4f\t%8.4f\t%8.4f",
				// 	 (double)rc.channels[0],
				// 	 (double)rc.channels[1],
				// 	 (double)rc.channels[4]);

				activate_channel(rc);
				if (stop_program==1){
					PX4_INFO("exiting");
					stop_program=0;
					return OK;
				}
				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/


			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}
	PX4_INFO("exiting");
	return OK;
}
