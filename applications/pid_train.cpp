#include "pid_train.hpp"

PID::PID(
	float dt, float kp, float ki, float kd, float max_out, float max_iout, float alpha,
	bool angular, bool dynamic)
: dt_(dt),
  kp_(kp),
  ki_(ki),
  kd_(kd),
  max_out_(max_out),
  max_iout_(max_iout),
  alpha_(alpha),
  angular_(angular),
  dynamic_(dynamic)
{
}

void PID::calc(float set, float fdb)
{
	if (dout_ready_ == true) {
		// 微分先行
		this->data.dbuf[2] = this->data.dbuf[1];
		this->data.dbuf[1] = this->data.dbuf[0];
		this->data.dbuf[0] =
			angular_ ? sp::limit_angle((this->data.fdb - fdb)) : (this->data.fdb - fdb);
	}
	else {
		dout_ready_ = true;
	}

	// 滤波
	this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

	this->data.err[2] = this->data.err[1];
	this->data.err[1] = this->data.err[0];
	this->data.err[0] = angular_ ? sp::limit_angle(set - fdb) : (set - fdb);

	this->data.set = set;
	this->data.fdb = fdb;

	// Kp
	this->data.pout = kp_ * this->data.err[0];

	// Ki
	this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
	this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0]));		   // 变速积分
	this->data.iout = sp::limit_max(
		this->data.iout + (dynamic_ ? this->data.dynamic_ki : ki_) * this->data.trapezoid,
		max_iout_);

	// Kd
	this->data.dout = kd_ * this->data.dbuf[0] / dt_;

	this->out = sp::limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

void PID::clear()
{
	this->data = {};   // 清空所有的中间状态和缓存数组
	this->out = 0.0f;  // 清空输出
	this->dout_ready_ = false;
}