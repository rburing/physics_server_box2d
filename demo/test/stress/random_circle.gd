extends CollisionShape2D


# Called when the node enters the scene tree for the first time.
func _ready():
	(shape as CircleShape2D).radius = randf_range(1, 15)
