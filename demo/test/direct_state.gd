extends RigidBody2D

func _ready():
	print(get_viewport().find_world_2d().space)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_GRAVITY, 970);

func _integrate_forces(state):
	print(state.get_contact_count())
	print(state.get_contact_local_normal(0))
	print(state.get_contact_local_position(0))
	print(state.get_contact_local_shape(0))
	print(state.get_contact_collider(0))
	print(state.get_contact_collider_id(0))
	print(state.get_contact_collider_object(0))
	print(state.get_contact_collider_position(0))
	print(state.get_contact_collider_shape(0))
	print(state.get_contact_collider_velocity_at_position(0))
	print(state.get_contact_impulse(0))
	var space_state: PhysicsDirectSpaceState2D = state.get_space_state()
	
	#var space_state :PhysicsDirectSpaceState2D = state.get_space_state()
	#space_state.
	#state.
	#print(state.get_contact_count())
	#pass
