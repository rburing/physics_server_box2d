extends ShowInfo

@export var rb = self

func _integrate_forces(state: PhysicsDirectBodyState2D):
	state.add_constant_central_force(Vector2(1,0))
	show_info(["constant_central_force: %s" % state.get_constant_force()])
