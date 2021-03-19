/*
 * PID.cpp
 *
 *  Created on: Mar 18, 2021
 *      Author: gilg
 */

#include "PID.h"

template <typename T, typename TK>
PID <T, TK>::~PID() {
	// TODO Auto-generated destructor stub
}

template <typename T, typename TK>
PID <T, TK>::PID(const PID &other) {
	// TODO Auto-generated constructor stub

}

template <typename T, typename TK>
PID <T, TK> ::PID(PID &&other) {
	// TODO Auto-generated constructor stub

}

template <typename T, typename TK>
PID<T,TK>& PID <T, TK>::operator=(const PID &other) {
	// TODO Auto-generated method stub

}

template <typename T, typename TK>
PID<T,TK>& PID <T, TK>::operator=(PID &&other) {
	// TODO Auto-generated method stub

}

template <typename T, typename TK>
PID <T, TK> ::PID():  Kp(0), Ki(0), Kd(0), error(0), privious_error(0), delta_error(0), summ_error(0), p_er(0), d_er(0), I_er(0), integral_saturation(0), differential_saturation(0), output(0)
{
}

template <typename T, typename TK>
PID <T, TK> ::PID(TK Kp_, TK Ki_, TK Kd_, T integral_saturation_, T differential_saturation_): Kp(Kp_), Ki(Ki_), Kd(Kd_), error(0), privious_error(0), delta_error(0), summ_error(0), p_er(0), d_er(0), I_er(0), integral_saturation(0), differential_saturation(0), output(0)
{
}

template <typename T, typename TK>
void PID <T, TK>::update(T& x0, T& x, TK& delta_t)
{
	error = x0 - x;
	delta_error = error - privious_error;
	summ_error += error;

	p_er = Kp * error;
	I_er = Ki * summ_error * delta_t;
	d_er = Kd * delta_error / delta_t;

	if(use_integral_saturation)
	{
		I_er = I_er >= integral_saturation ? integral_saturation : I_er;
		I_er = I_er <= -integral_saturation ? -integral_saturation :I_er;
	}

	if(use_differential_saturation)
	{
		d_er = d_er >= differential_saturation ? differential_saturation : d_er;
		d_er = d_er <= -differential_saturation ? -differential_saturation : d_er;
	}

	output = p_er + I_er + d_er;
}

