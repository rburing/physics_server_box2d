extends ShowInfo

func _process(delta):
	show_info(["Default Area:",
		"default_gravity %s" % PhysicsServer2D.area_get_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_GRAVITY),
		"angular_damp %s" % PhysicsServer2D.area_get_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_ANGULAR_DAMP),
		"linear_damp %s" % PhysicsServer2D.area_get_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_LINEAR_DAMP),
		"priority %s" % PhysicsServer2D.area_get_param(get_viewport().find_world_2d().space, PhysicsServer2D.AREA_PARAM_PRIORITY)
		])
	
