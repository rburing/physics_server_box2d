extends ShowInfo

@onready var rb = self

func _integrate_forces(state: PhysicsDirectBodyState2D):
	show_info(["Direct Body State:",
		"sleeping: %s" % str(state.sleeping),
		"contact_count: %s" % str(state.get_contact_count()),
		"contact_0_normal: %s" % str(state.get_contact_local_normal(0)),
		"contact_0_position: %s" % str(state.get_contact_local_position(0)),
		"contact_0_shape: %s" % str(state.get_contact_local_shape(0)),
		"contact_0_collider_position: %s" % str(state.get_contact_collider_position(0)),
		"contact_0_collider_shape: %s" % str(state.get_contact_collider_shape(0)),
		"contact_0_collider_velocity_at_point: %s" % str(state.get_contact_collider_velocity_at_position(0)),
		"contact_0_collider_impulse: %s" % str(state.get_contact_impulse(0)),
		"contact_1_normal: %s" % str(state.get_contact_local_normal(1)),
		"contact_1_position: %s" % str(state.get_contact_local_position(1)),
		"contact_1_shape: %s" % str(state.get_contact_local_shape(1)),
		"contact_1_collider_position: %s" % str(state.get_contact_collider_position(1)),
		"contact_1_collider_shape: %s" % str(state.get_contact_collider_shape(1)),
		"contact_1_collider_velocity_at_point: %s" % str(state.get_contact_collider_velocity_at_position(1)),
		"contact_1_collider_impulse: %s" % str(state.get_contact_impulse(1)),
		])
	var space_state: PhysicsDirectSpaceState2D = state.get_space_state()
