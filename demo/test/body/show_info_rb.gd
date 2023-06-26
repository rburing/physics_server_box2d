extends ShowInfo

@onready var rb = self

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	show_info(["RigidBody2D:",
		"rid: %s" % str(rb.get_rid()),
		"mass: %s" % str(rb.mass),
		"inertia: %s" % str(rb.inertia),
		"center_of_mass: %s" % str(rb.center_of_mass),
		"gravity_scale: %s" % str(rb.gravity_scale),
		"sleeping: %s" % str(rb.sleeping),
		"linear_damp: %s" % str(rb.linear_damp),
		"angular_damp: %s" % str(rb.angular_damp),
		"linear_velocity: %s" % str(rb.linear_velocity),
		"angular_velocity: %s" % str(rb.angular_velocity),
		"constant_force: %s" % str(rb.constant_force),
		"constant_torque: %s" % str(rb.constant_torque)])

