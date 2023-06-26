extends CollisionShape2D


# Called when the node enters the scene tree for the first time.
func _ready():
	(shape as RectangleShape2D).size.x = randf_range(5, 25)
	(shape as RectangleShape2D).size.y = randf_range(5, 25)
