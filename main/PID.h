/*
 * PID.h
 *
 *  Created on: Mar 18, 2021
 *      Author: gilg
 */

#ifndef MAIN_PID_H_
#define MAIN_PID_H_

template <typename T, typename TK>
class PID
{
private:
	TK Kp;
	TK Ki;
	TK Kd;

	T error;
	T privious_error;
	T delta_error;
	T summ_error;

	T p_er;
	T d_er;
	T I_er;

	T integral_saturation;
	T differential_saturation;
	bool use_integral_saturation = false;
	bool use_differential_saturation = false;

	T output;
public:
	PID(TK Kp_, TK Ki_, TK Kd_, T integral_saturation_, T differential_saturation_);

	PID();
	~PID();
	PID(const PID &other);
	PID(PID &&other);
	PID& operator=(const PID &other);
	PID& operator=(PID &&other);

	void update(T& x0, T& x, TK& delta_t);
	void update (T& error);
	void update (T&& error);
};
#include "PID.tpp"

#endif /* MAIN_PID_H_ */
