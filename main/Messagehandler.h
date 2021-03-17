/*
 * Messagehandler.h
 *
 *  Created on: Mar 17, 2021
 *      Author: gilg
 */

#ifndef MAIN_MESSAGEHANDLER_H_
#define MAIN_MESSAGEHANDLER_H_

#include "stddef.h"

class Message_handler {
public:
	Message_handler();
	~Message_handler();
	void get_new_message(void* ptr, int len);
};

#endif /* MAIN_MESSAGEHANDLER_H_ */
