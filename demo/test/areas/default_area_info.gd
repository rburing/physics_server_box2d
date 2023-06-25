extends Node2D

func _ready():
	await get_tree().create_timer(2).timeout
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_GRAVITY, -980)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_ANGULAR_DAMP, 20)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE, PhysicsServer2D.AREA_SPACE_OVERRIDE_COMBINE)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_LINEAR_DAMP, 20)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE, PhysicsServer2D.AREA_SPACE_OVERRIDE_COMBINE)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_PRIORITY, 1)
	
	
	await get_tree().create_timer(5).timeout
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_GRAVITY, 980)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_ANGULAR_DAMP, 1)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE, PhysicsServer2D.AREA_SPACE_OVERRIDE_COMBINE)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_LINEAR_DAMP, 0.1)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE, PhysicsServer2D.AREA_SPACE_OVERRIDE_COMBINE)
	PhysicsServer2D.area_set_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_PRIORITY, -1)
