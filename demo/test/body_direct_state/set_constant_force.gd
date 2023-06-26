extends ShowInfo

@export var rb = self

func _integrate_forces(state: PhysicsDirectBodyState2D):
	state.set_constant_force(Vector2(100,0))
	show_info(["constant_central_force: %s" % state.get_constant_force()])
