extends ShowInfo


func _process(delta):
	show_info(["Active Bodies %s" % str(PhysicsServer2D.get_process_info(PhysicsServer2D.INFO_ACTIVE_OBJECTS)),
	"Collision Pairs %s" % str(PhysicsServer2D.get_process_info(PhysicsServer2D.INFO_COLLISION_PAIRS)),
	"Island Count %s" % str(PhysicsServer2D.get_process_info(PhysicsServer2D.INFO_ISLAND_COUNT))])
