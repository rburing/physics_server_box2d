extends ShowInfo

@export var rb = self

func _integrate_forces(state: PhysicsDirectBodyState2D):
	state.add_constant_torque(0.005)
	show_info(["constant_torque: %s" % state.get_constant_torque(),
			"constant_torque: %s" % state.get_constant_torque()])
