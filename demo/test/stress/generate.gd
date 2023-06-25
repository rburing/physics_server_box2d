extends Node2D

@export var total = 1000
@export var node: PackedScene
@export var label: Label

func create():
	var created: RigidBody2D = node.instantiate() as RigidBody2D
	created.linear_velocity.x = randf()
	add_child(created)

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	label.text = str(get_child_count() - 3)
	if get_child_count() - 3 < total:
		create()
		create()
