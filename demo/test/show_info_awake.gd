extends RigidBody2D

var label: Label = Label.new()
var node2d: Node2D = Node2D.new()
var theme: Theme = load("res://new_theme.tres")

# Called when the node enters the scene tree for the first time.
func _ready():
	label.theme = theme
	add_child(node2d)
	node2d.add_child(label)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	node2d.global_rotation = 0
	label.text = "sleeping: %s" % str(sleeping)
