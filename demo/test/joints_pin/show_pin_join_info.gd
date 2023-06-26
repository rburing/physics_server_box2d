extends ShowInfo

@export var pin_joint = self

func _process(delta):
	show_info(["node_a: %s" % str(pin_joint.node_a),
		"node_b: %s" % str(pin_joint.node_b)])
