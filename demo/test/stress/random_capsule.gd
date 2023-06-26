extends CollisionShape2D


# Called when the node enters the scene tree for the first time.
func _ready():
	(shape as CapsuleShape2D).radius = randf_range(1, 10)
	(shape as CapsuleShape2D).height = randf_range(5, 50)
