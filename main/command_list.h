/*
 * command_list.h
 *
 *  Created on: 23 мая 2021 г.
 *      Author: Danila
 */

#ifndef MAIN_COMMAND_LIST_H_
#define MAIN_COMMAND_LIST_H_

enum command_list
{
	set_throttle_comm,
	set_orientation_comm,
	start_comm, // empty
	stop_comm, // empty


};

struct set_throttle
{
	float value;
}__attribute__((packed));

struct set_orientation
{
	float pitch;
	float roll;
	float yaw;
}__attribute__((packed));




#endif /* MAIN_COMMAND_LIST_H_ */
