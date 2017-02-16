#pragma once
#include <unBoard.hpp>
#include <motionData.hpp>
#include <types/ActionCommand.hpp>


namespace MotionAux
{
    static float filtered_fsr_sum = 0;
	ActionCommand::Body Walk (	const float& x,
								const float& y,
								const float& theta);

	ActionCommand::Body Kick (	const float& x,
								const float& y,
								const float& direction = 0.0,
								const ActionCommand::Body::Foot& foot = ActionCommand::Body::LEFT,
								const bool& misaligned = false);

	ActionCommand::Head MoveHead (	const float& pitch,
									const float& yaw,
									const float& pSpeed = 1.0,
									const float& ySpeed = 1.0,
									const bool& isRelative = true);

	ActionCommand::Body Stop();

	ActionCommand::Body Stand();

	ActionCommand::Body Penalize();

	ActionCommand::Body GetUpFront();

	ActionCommand::Body GetUpBack();

	ActionCommand::Body DeadStiffness();

	ActionCommand::Body RefPickup();

	void SendCommand(const Command& command);
}
