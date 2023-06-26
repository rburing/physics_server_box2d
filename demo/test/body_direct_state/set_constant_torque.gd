extends ShowInfo

@export var rb = self

func _integrate_forces(state: PhysicsDirectBodyState2D):
	state.set_constant_torque(20)
	show_info(["constant_torque: %s" % state.get_constant_torque(),
			"constant_torque: %s" % state.get_constant_torque()])
